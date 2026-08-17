//
// Created by xiaofan on 24-11-25.
//

#include "main.h"


#include <pcl/filters/voxel_grid.h>
#include <filesystem>

namespace {
volatile std::sig_atomic_t shutdown_requested = 0;

void SigHandle(int)
{
    shutdown_requested = 1;
}
}

/*************************** 表驱动 FSM ****************************/
namespace FSM{
    // 用 pair<State,Event> 做键，建立转换表
    const std::map<Key, Transition> kTransitionTable = {
        // 请按此格式，为每一条“(当前状态, 事件) -> (下一个状态, 动作回调)”添加一行
        {{State::Waitting, Event::IMUTriger}, { State::Initializing, [](){  } }},
        {{State::Initializing, Event::InitEnd}, { State::IMUProcess, [](){  } }},
        {{State::IMUProcess, Event::LidarTriger}, { State::LidarProcess, [](){  } }},
        {{State::LidarProcess, Event::LidarEnd}, { State::IMUProcess, [](){  } }},
        {{State::IMUProcess, Event::ImgTriger}, { State::ImgProcess, [](){  } }},
        {{State::ImgProcess, Event::ImgEnd}, { State::IMUProcess, [](){  } }},
    };
    // Dispatch 函数
    State current_state = State::Waitting;  // 初始状态
    void dispatch(Event e) {
        auto it = kTransitionTable.find({ current_state, e });
        if (it != kTransitionTable.end()) {
            State next = it->second.next_state;
            // ➤ 在这里打印转换信息
            std::cout
              << "[FSM] "
              << to_string(current_state)
              << " + "
              << to_string(e)
              << " -> "
              << to_string(next)
              << std::endl;
            // 执行动作
            it->second.action();
            // 切换状态
            current_state = it->second.next_state;
        } else {
            // 未定义转换时的处理
            std::cerr << "非法转换：状态="
                      << static_cast<int>(current_state)
                      << " 事件="
                      << static_cast<int>(e)
                      << "\n";
        }
    }
}


void publish_visual_world_sub_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub, const PointCloudXYZI::Ptr& cloud) {
    if (cloud->empty()) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = get_ros_time(lidar_end_time);
    msg.header.frame_id = "world";
    pub->publish(msg);
}


/********************************************** Global Variable **********************************************/
auto shared_bufs = std::make_shared<SharedBuffers>();
std::shared_ptr<TopicProcess> p_topic_process(new TopicProcess(shared_bufs));// 指向话题数据的预处理类 topic_process 的智能指针
std::shared_ptr<EskfEstimator> p_eskf_estimator(new EskfEstimator());// 指向 EskfEstimator 类的智能指针
KD_TREE<PointType> ikdtree;
vector<BoxPointType> cub_needrm; // ikd-tree中，地图需要移除的包围盒序列
// MeasureGroup Measures;
vector<State> s_next_v ;
// 存储 lidar 系下的原始点云，在每次接收到点云 topic 后在回调函数中更新
PointCloudXYZI::Ptr cloud_out(new PointCloudXYZI);
// 存储 lidar 系下的有效点云，调用 p_eskf_estimator -> UpdateByLidarIESKF() 后更新，更具体的说，在 UpdateByLidarIESKF() 中的 calculate() 函数中更新
PointCloudXYZI::Ptr down_effect_cloud_lidar(new PointCloudXYZI);
// 存储 world 系下的地图，由 effect_down_cloud_world（ 每一帧 world 系下点云 ） 积累而来
PointCloudXYZI::Ptr map_world(new PointCloudXYZI);
// 在增量式存储下，记录每次增量 PCD 保存的路径
vector<string> incremental_paths;
// lidar_end_time 是根据最后一个点云的时间戳偏移计算的，这是一个全局变量，在每次调用 syncpackage() 函数时更新
double lidar_end_time = 0;
double lastest_imu_time = 0;

Eigen::Vector3d mean_acc;
Eigen::Vector3d mean_gyr;


int NUM_MAX_ITERATIONS = 3;
int MIN_IMG_COUNT = 0;
double IMG_POINT_COV = 100;
int patch_size = 8;
double outlier_threshold = 300;


/********************************************** Constructor **********************************************/

LIONode::LIONode() : rclcpp::Node("lio_node") {
    RCLCPP_INFO(this->get_logger(), "LIONode is running!");
    // 注意: 发布器/订阅器/定时器的创建被移到 initROS() 中,
    // 因为 topic 名称等配置需要先由 load_root_yaml() 从 yaml 文件加载 (与 ROS1 原逻辑顺序一致)
}

void LIONode::initROS() {
    // Interface-compat switch (FastAnchor integration): allow disabling yifanLIO's own
    // TF broadcasts so the FastAnchor/adapter TF tree is the single source of truth.
    this->declare_parameter<bool>("publish_tf", true);
    this->get_parameter("publish_tf", publish_tf_);
    RCLCPP_INFO(this->get_logger(), "yifanLIO publish_tf = %s", publish_tf_ ? "true" : "false");

    /****** Topic Publish (QoS: 点云类可视化输出使用 BestEffort 防止背压阻塞, 里程计/路径使用 Reliable) ******/
    clouds_lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(clouds_lidar_topic_name, rclcpp::SensorDataQoS());
    global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(global_map_topic_name, rclcpp::SensorDataQoS());
    ikdtree_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ikdtree_topic_name, rclcpp::SensorDataQoS());
    /****** String Publish (latched 对应 ROS1 的 latched=true, 使用 TransientLocal) ******/
    map_path_pub_ = this->create_publisher<std_msgs::msg::String>("/LIO/map_path", rclcpp::QoS(1).transient_local().reliable());
    /****** Transform Publish ******/
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    /****** Odometry Publish ******/
    odom_imu_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_imu_topic_name, rclcpp::QoS(10));
    odom_body_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_vehicle_topic_name, rclcpp::QoS(10));
    odom_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/lio_node/path_topic_name", rclcpp::QoS(50));
    pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/LIVO/cloud_registered", rclcpp::SensorDataQoS());
    /****** Topic Subscriber (LiDAR/IMU/轮速 使用 SensorDataQoS 实时优先; depth 与原 ROS1 队列长度保持一致) ******/
    if (lidar_type == LidarType::Livox) {
        pointcloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lidar_topic_name, rclcpp::SensorDataQoS().keep_last(5000), pcl_cbk_custom);
    } else {
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_name, rclcpp::SensorDataQoS().keep_last(5000), pcl_cbk_pc2);
    }
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_name, rclcpp::SensorDataQoS().keep_last(100000), imu_cbk);
    wheel_sub_ = this->create_subscription<lio::msg::WheelInfo>(
        wheel_topic_name, rclcpp::SensorDataQoS().keep_last(5000), wheel_cbk);
    /****** Timer (周期与 ROS1 的 ros::Rate 保持一致: 5000Hz/1000Hz/10Hz/1Hz) ******/
    timer_500HZ_ = this->create_wall_timer(std::chrono::microseconds(200), std::bind(&LIONode::timer_500HZ_callback, this));
    timer_100HZ_ = this->create_wall_timer(std::chrono::microseconds(1000), std::bind(&LIONode::timer_100HZ_callback, this));
    timer_10HZ_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LIONode::timer_10HZ_callback, this));
    timer_1HZ_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LIONode::timer_1HZ_callback, this));
}



/********************************************** Timer Callback **********************************************/

void LIONode::timer_1HZ_callback() {
    /**** 发布静态tf ****/
    publishStaticTransform();
    /*** 发布全局地图 ***/
    publishGlobalMap();
    /*** 发布 ikdtree ***/
    publishIKDTree();
}

void LIONode::timer_10HZ_callback() {

}


#include "IMUProcess.h"
IMUProcess imu_process;


void LIONode::timer_100HZ_callback() {
    // 打包数据
    static PointCloudXYZI::Ptr Dedistort_clouds_lidar (new PointCloudXYZI); // 去畸变点云，lidar系
    static PointCloudXYZI::Ptr down_Dedistort_clouds_lidar (new PointCloudXYZI); // 去畸变点云，lidar系
    static PointCloudXYZI::Ptr pcl_camera_use_world (new PointCloudXYZI); // VIO更新使用点云
    static PointCloudXYZI::Ptr cloud_init_world(new PointCloudXYZI); // 初始化点云，world系
    static PointCloudXYZI::Ptr cloud_init_lidar(new PointCloudXYZI); // 初始化点云，lidar系
    static pcl::VoxelGrid<PointType> downSizeFilter;
    static bool first_lidar = true;
    /*********** 接收到数据后的处理从这里开始 *************/
    switch(FSM::current_state) {
        case FSM::State::Waitting: {
            // 待机
            break;
        }
        case FSM::State::Initializing:{
            // 初始化在500HZ函数里做
            break;
        }
        case FSM::State::IMUProcess:{
            // IMU预积分在 500HZ 函数里
            break;
        }
        case FSM::State::LidarProcess: {
            // 提取激光点云与时间戳
            PointCloudXYZI::Ptr raw_clouds_lidar = shared_bufs->getFrontSafe(shared_bufs->lidar_buf);
            double base_time = shared_bufs->getFrontSafe(shared_bufs->lidar_base_time_buf);
            // double lidar_begin_time = base_time + raw_clouds_lidar->points.front().curvature / double(1000);
            // static double base_time_ = lidar_begin_time;
            // cout << "lidar_begin_time: " << lidar_begin_time - base_time_ << " lidar_end_time: " << lidar_end_time - base_time_<< std::endl;
            // 初始化 kdtree
            if (first_lidar & !relocation_enable) {
                // 降采样参数
                ikdtree.set_downsample_param(filter_size);
                // 建图模式，需要建立 ikdtree
                downSizeFilter.setLeafSize(filter_size,filter_size, filter_size); // 降采样参数
                downSizeFilter.setInputCloud(raw_clouds_lidar); // 获得去畸变后的点云数据
                downSizeFilter.filter(*cloud_init_lidar); //滤波降采样
                cloud_init_world = p_eskf_estimator -> transLidar2World(*cloud_init_lidar); // 转换到 world 系下
                ikdtree.Build(cloud_init_world -> points); // world 系下的点云
                std::cout << "[timer_100HZ_callback] Build Map Model " << std::endl;
                FSM::dispatch(FSM::Event::LidarEnd);
                first_lidar = false;
                shared_bufs->popFrontSafe(shared_bufs->lidar_buf);
                shared_bufs->popFrontSafe(shared_bufs->lidar_base_time_buf);
                shared_bufs->popFrontSafe(shared_bufs->trigger_buf);
                break;
            }
            // 点云去畸变
            std::vector<Pose> imu_pose_vec;
            State lidar_end_state;
            *Dedistort_clouds_lidar = *raw_clouds_lidar;
            imu_process.get_state_at_t(lidar_end_state, lidar_end_time);
            imu_process.get_imu_pose_from_t1_to_t2(imu_pose_vec,base_time,lidar_end_time);
            if (imu_pose_vec.empty()) {
                shared_bufs->popFrontSafe(shared_bufs->lidar_buf);
                shared_bufs->popFrontSafe(shared_bufs->lidar_base_time_buf);
                shared_bufs->popFrontSafe(shared_bufs->trigger_buf);
                FSM::dispatch(FSM::Event::LidarEnd);
                return;
            }
            auto it_pcl = Dedistort_clouds_lidar->points.begin();
            int cnt1 = 0, cnt2 = 0;
            for (auto it_kp = imu_pose_vec.begin(); it_kp + 1 != imu_pose_vec.end(); ++it_kp)
            {
                auto head = it_kp;
                auto tail = it_kp + 1;
                auto q_imu = head->q;
                auto vel_imu = head->v;
                auto pos_imu = head->p;
                auto acc_imu = tail->acc; // 世界系
                auto angvel_imu = tail->gyr;
                cnt1++;
                // 对两帧imu之间的点去畸变
                for(; it_pcl->curvature / double(1000) <= tail->offset_time + 1E-6; ++it_pcl)
                {
                    cnt2++;
                    double dt = it_pcl->curvature / double(1000) - head->offset_time;
                    // W: 世界坐标下，初始时刻的IMU坐标系
                    // L: lidar 坐标系，当前处理的点时刻
                    // I: IMU 坐标系，当前处理的点时刻
                    // IE: IMU Endpoint 坐标系，当前帧最后时刻对应的 IMU 坐标系
                    // LE: lidar Endpoint 坐标系，当前帧最后时刻对应的激光雷达坐标系
                    Eigen::Vector3d P_L(it_pcl->x, it_pcl->y, it_pcl->z); // 激光雷达坐标系下点的位置（观测值）
                    Eigen::Vector3d P_I = imu_R_lidar * P_L + imu_t_lidar; // IMU 坐标系下点的位置
                    Eigen::Matrix3d R_I_W(q_imu * Exp(angvel_imu, dt)); // IMU 当前帧相对世界坐标系的旋转矩阵（对原始矩阵做一些小量更新）
                    Eigen::Vector3d T_I_W(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt); // IMU 当前帧相对世界坐标系的位置（对原始位置的二阶预测近似）
                    Eigen::Vector3d P_W = R_I_W * P_I + T_I_W; // 点在世界坐标系下的位置
                    // 推导至 lidar Endpoint 坐标系
                    Eigen::Vector3d P_IE = lidar_end_state.q.inverse() * (P_W - lidar_end_state.p); // 点在 IMU Endpoint坐标系下的位置
                    Eigen::Vector3d P_LE = imu_R_lidar.transpose() * (P_IE - imu_t_lidar); // 点在 lidar Endpoint坐标系下的位置
                    // save Undistorted points and their rotation
                    it_pcl->x = P_LE(0);
                    it_pcl->y = P_LE(1);
                    it_pcl->z = P_LE(2);
                    // 当点云被遍历完后跳出循环
                    if (it_pcl == Dedistort_clouds_lidar->points.end() - 1) {
                        break;
                    }
                }
            }
            // ASSERT(it_pcl == Dedistort_clouds_lidar->points.end() - 1); // 确保所有点都被遍历
            // Retail_Street.bag 包有问题，在某个时刻的点云数据中 curvature 居然达到了 2590 （两秒）
            // 点云降采样
            if (Dedistort_clouds_lidar->points.size()  > 10) {
                downSizeFilter.setLeafSize(filter_size, filter_size, filter_size); // 降采样参数
                downSizeFilter.setInputCloud(Dedistort_clouds_lidar); // 获得去畸变后的点云数据
                downSizeFilter.filter(*down_Dedistort_clouds_lidar); //滤波降采样
                cout << "[ timer_100HZ_callback ]: downsample ! In num : "<< Dedistort_clouds_lidar->points.size() <<
                    " downsamp " << down_Dedistort_clouds_lidar->points.size() <<endl;
            }else {
                cout << "[ timer_100HZ_callback ]: No downsample ! use previous downsample points: " <<
                     down_Dedistort_clouds_lidar->points.size() <<endl;
            }
            // 这里是因为其他地方还需要用，为了防止报错先添加进去
            p_eskf_estimator ->state_vector_prior_.clear();
            p_eskf_estimator ->state_vector_prior_.push_back(lidar_end_state);
            p_eskf_estimator->set_cur_state(lidar_end_state);
            // 调整局部地图位置
            lasermap_fov_segment();
            // 更新激光雷达状态
            p_eskf_estimator -> UpdateByLidarIESKF(*down_Dedistort_clouds_lidar);
            State post_state = p_eskf_estimator -> get_cur_state();
            pcl_camera_use_world = p_eskf_estimator -> transLidar2World(*Dedistort_clouds_lidar);
            // 更新 imu_process 中的位姿
            imu_process.set_state_at_t(post_state, lidar_end_time);
            shared_bufs->popFrontSafe(shared_bufs->lidar_buf);
            shared_bufs->popFrontSafe(shared_bufs->lidar_base_time_buf);
            shared_bufs->popFrontSafe(shared_bufs->trigger_buf);
            /***** 更新ikdtree *****/
            map_incremental(*down_Dedistort_clouds_lidar);
            auto states = p_eskf_estimator -> get_post_states();
            /***** 发布 odom 和 lidar 系点云 *****/
            if(!states.empty()) {
                for (auto & state : states) {
                    publish_imu_odometry(state);
                    publish_body_odometry(state);
                    // 将位姿记录到文件中
                    if(trajectory_log_enable & log_save_enable)save_odometry(state);
                }
            }
            publish_Dedistort_clouds_lidar(Dedistort_clouds_lidar);
            // *pcl_camera_use_lidar = *Dedistort_clouds_lidar;
            /***  保存单帧点云  ***/
            save_singel_clouds_world(Dedistort_clouds_lidar);
            FSM::dispatch(FSM::Event::LidarEnd);
            break;
        }
        case FSM::State::ImgProcess:{
            FSM::dispatch(FSM::Event::ImgEnd);
            break;
        }
    }
    /***** 轮速更新 ****/
    auto time5 = std::chrono::high_resolution_clock::now();
    if (wheel_enable) {
        std::deque<WheelData> wheel_deq = p_topic_process -> get_wheel_buffer();
        if (!wheel_deq.empty()) {
            p_eskf_estimator -> UpdateByWheel(wheel_deq);
        }else {
            std::cout << "[Warning] [timer_100HZ_callback] wheel_deq is empty" << std::endl;
        }
    }
}

bool InitImu(ImuMsgConst &msg , int init_imu_num ,
             Eigen::Vector3d & mean_acc_,
             Eigen::Vector3d & mean_gyr_,
             State &state) {
    bool coverage_flag = false;
    static std::deque<Eigen::Vector3d> acc_buffer;
    static std::deque<Eigen::Vector3d> gyr_buffer;

    Eigen::Vector3d cur_gyr (msg->angular_velocity.x,
                             msg->angular_velocity.y,
                             msg->angular_velocity.z);
    Eigen::Vector3d cur_acc (msg->linear_acceleration.x,
                             msg->linear_acceleration.y,
                             msg->linear_acceleration.z);

    // 放入buffer中
    acc_buffer.push_back(cur_acc);
    gyr_buffer.push_back(cur_gyr);

    // 计算均值
    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyr = Eigen::Vector3d::Zero();
    for (const auto &acc : acc_buffer) mean_acc += acc;
    for (const auto &gyr : gyr_buffer) mean_gyr += gyr;
    mean_acc /= acc_buffer.size();
    mean_gyr /= gyr_buffer.size();

    // 更新 private 变量
    mean_acc_ = mean_acc;
    mean_gyr_ = mean_gyr;

    if (acc_buffer.size() >= init_imu_num) {
        coverage_flag = true;
        // acc_buffer.clear();
        // gyr_buffer.clear();
        state.bw = mean_gyr_;
        state.g = Eigen::Vector3d(0, 0,  - G_m_s2);
        state.q = Eigen::Quaterniond::FromTwoVectors(- mean_acc_ , state.g); // 初始化姿态
        state.ba = mean_acc_ * G_m_s2 / mean_acc_.norm() + state.q.inverse() * state.g;
        state.p  = Eigen::Vector3d::Zero();
        state.v  = Eigen::Vector3d::Zero();
        state.P.setZero(StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL);
        state.P.block<3,3>(StateIndex::R,  StateIndex::R)  = Eigen::Matrix3d::Identity() * 1e-4;
        state.P.block<3,3>(StateIndex::P,  StateIndex::P)  = Eigen::Matrix3d::Identity() * 1e-2;
        state.P.block<3,3>(StateIndex::V,  StateIndex::V)  = Eigen::Matrix3d::Identity() * 1e-2;
        state.P.block<3,3>(StateIndex::BW, StateIndex::BW) = Eigen::Matrix3d::Identity() * 1e-6;
        state.P.block<3,3>(StateIndex::BA, StateIndex::BA) = Eigen::Matrix3d::Identity() * 1e-6;
        state.P.block<3,3>(StateIndex::G,  StateIndex::G)  = Eigen::Matrix3d::Identity() * 1e-4;
    }
    // std::cout << "should be zero: " << state_.q * (mean_acc_ * G_m_s2 / mean_acc_.norm() - state_.ba) + state_.g << std::endl;
    return coverage_flag;
}

// void InitIKDTree() {
//     /** 注意：
//      * 重定位模式下的 ikdtree 在 build_kdtree_from_file() 函数中已经提前构建了，这里不需要再次构建
//      */
//     pcl::VoxelGrid<PointType> downSizeFilter;
//     PointCloudXYZI::Ptr cloud_init_lidar(new PointCloudXYZI); // 初始化点云，lidar系
//     auto cloud_init_lidar = shared_bufs->ge
//     if(!relocation_enable) {
//         downSizeFilter.setLeafSize(filter_size,filter_size, filter_size); // 降采样参数
//         downSizeFilter.setInputCloud(Measures.lidar); // 获得去畸变后的点云数据
//         downSizeFilter.filter(*cloud_init_lidar); //滤波降采样
//         auto cloud_init_world_part = p_eskf_estimator -> transLidar2World(*cloud_init_lidar); // 转换到 world 系下
//         *cloud_init_world += *cloud_init_world_part;
//         ikdtree.Build(cloud_init_world -> points); // world 系下的点云
//         std::cout << "[timer_100HZ_callback] Build Map Model " << std::endl;
//     }
// }

// ikdtree.set_downsample_param(filter_size);
// if(!relocation_enable) {
//     downSizeFilter.setLeafSize(filter_size,filter_size, filter_size); // 降采样参数
//     downSizeFilter.setInputCloud(Measures.lidar); // 获得去畸变后的点云数据
//     downSizeFilter.filter(*cloud_init_lidar); //滤波降采样
//     auto cloud_init_world_part = p_eskf_estimator -> transLidar2World(*cloud_init_lidar); // 转换到 world 系下
//     *cloud_init_world += *cloud_init_world_part;
//     ikdtree.Build(cloud_init_world -> points); // world 系下的点云
//     std::cout << "[timer_100HZ_callback] Build Map Model " << std::endl;
// }
// std::cout << "[timer_100HZ_callback] initialization finished" << std::endl;

void LIONode::timer_500HZ_callback() {
    switch(FSM::current_state) {
        case FSM::State::Initializing: {
            if (shared_bufs->isEmptySafe(shared_bufs->imu_buf)) break;
            State state;
            ImuMsgConst msg = shared_bufs->getFrontSafe(shared_bufs->imu_buf);
            bool converge = InitImu(msg, 200 , mean_acc , mean_gyr, state);
            // cout << "mean_acc: \n" << mean_acc << endl;
            shared_bufs -> popFrontSafe(shared_bufs->imu_buf);
            if(converge) {
                p_eskf_estimator -> mean_acc_ = mean_acc;
                p_eskf_estimator -> mean_gyr_ = mean_gyr;
                p_eskf_estimator -> set_cur_state(state);
                imu_process.set_init_state(state,  get_time_sec(msg->header.stamp));
                FSM::dispatch(FSM::Event::InitEnd);
            }
            break;
        }
        case FSM::State::IMUProcess: {
            if (! shared_bufs->isEmptySafe(shared_bufs->imu_buf)) {
                ImuMsgConst msg = shared_bufs->getFrontSafe(shared_bufs->imu_buf);
                imu_process.integration(msg, mean_acc);
                lastest_imu_time = get_time_sec(msg->header.stamp);
                // 这里在下一帧lidar到来前，将imu预积分位姿先发布出来，实现高频输出
                if (lastest_imu_time < lidar_end_time + 1.0/lidar_frequency - 0.005 && high_frequency_odom) {
                    State state;
                    imu_process.get_state_at_t(state, lastest_imu_time);
                    publish_imu_odometry(state, lastest_imu_time);
                    publish_body_odometry(state, lastest_imu_time);
                    if(trajectory_log_enable & log_save_enable)save_odometry(state,lastest_imu_time);
                }
                shared_bufs -> popFrontSafe(shared_bufs->imu_buf);
            }
            if (! shared_bufs->isEmptySafe(shared_bufs->trigger_buf)) {
                shared_bufs->sortTriggerBufByTimestamp(shared_bufs->trigger_buf);
                pair trigger = shared_bufs->getFrontSafe(shared_bufs->trigger_buf);
                // 最新的 imu 时间戳大于 点云/图像 时间，则进行 点云/图像 处理
                switch (trigger.first) {
                    case Trigger::Lidar: {
                        // PointCloudXYZI::Ptr raw_clouds_lidar = shared_bufs->getFrontSafe(shared_bufs->lidar_buf);
                        // lidar_end_time = trigger.second + raw_clouds_lidar->points.back().curvature / double(1000);
                        if (lastest_imu_time >= trigger.second) {
                            lidar_end_time = trigger.second;
                            FSM::dispatch(FSM::Event::LidarTriger);
                        }
                        break;
                    }
                    case Trigger::Image: {
                        if (lastest_imu_time >= trigger.second)
                            FSM::dispatch(FSM::Event::ImgTriger);
                        break;
                    }
                }
            }
            break;
        }
    }
}

// 如果是重定位模式，返回地图的路径字符串，如果不是，则返回空字符串
std::string build_kdtree_from_file() {
    std::string pcd_path;
    if (relocation_enable) {
        ikdtree.set_downsample_param(filter_size);
        PointCloudXYZI::Ptr cloud_init_world(new PointCloudXYZI); // 初始化点云，world系
        // 重定位模式，读取 .pcd 文件建立 ikdtree
        pcd_path = string(PACKAGE_ROOT_DIR) + "/PCD/" + pcd_load_name;
        pcl::io::loadPCDFile(pcd_path, *cloud_init_world);
        // 降采样
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setLeafSize(0.1, 0.1, 0.1); // 降采样参数为0.1m
        downSizeFilter.setInputCloud(cloud_init_world); // 获得去畸变后的点云数据
        downSizeFilter.filter(*cloud_init_world); //滤波降采样
        // 构建 ikdtree
        ikdtree.Build(cloud_init_world -> points); // world 系下的点云
        std::cout << "[build_kdtree_from_file] Relocation Map Model " << std::endl;
        // 输出点云数量
        std::cout << "[build_kdtree_from_file] Load Points Numble : " << cloud_init_world->points.size() << std::endl;
    }else{
        pcd_path = "";
    }
    return pcd_path;
}

/********************************************** main **********************************************/

int main(int argc, char** argv)
{
    // 初始化 ROS2 节点
    rclcpp::init(argc, argv);
    signal(SIGINT, SigHandle);
    signal(SIGTERM, SigHandle);

    // 创建节点，用于读取参数（与 ROS1 的私有 NodeHandle 对应）
    auto node = std::make_shared<LIONode>();
    RCLCPP_INFO(node->get_logger(), "Current NodeHandler namespace: %s", node->get_namespace());

    //读取配置文件路径 (ROS2 parameter, 与 ROS1 的 ~config_path 对应)
    node->declare_parameter<std::string>("config_path", "root_config.yaml");
    std::string config_path;
    node->get_parameter("config_path", config_path);
    RCLCPP_INFO(node->get_logger(), "Config path: %s", config_path.c_str());
    load_root_yaml(config_path);
    // 在 yaml 配置加载完成后, 再创建 发布器/订阅器/定时器 (与 ROS1 的 LIONode 构造时机一致)
    node->initROS();
    string map_path = build_kdtree_from_file();
    // 用来保存一些临时文件
    string tempdir1 = string(PACKAGE_ROOT_DIR) + "/PCD/Temp";
    string tempdir2 = string(PACKAGE_ROOT_DIR) + "/temp";
    std::filesystem::create_directories(tempdir1);
    std::filesystem::create_directories(tempdir2);

    node->publishPathOnce(map_path); // 发布一次，如果有订阅者需要则会收到

    // 启动并处理回调。退出信号只置位，由主线程完成后续保存流程。
    // 与 ROS1 的 ros::spinOnce() + ros::Rate(1000) 主循环结构保持一致：
    // 定时器与话题回调都在同一个线程内执行，保持原有并发行为
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::Rate spin_rate(1000);
    while (rclcpp::ok() && !shutdown_requested) {
        executor.spin_some();
        spin_rate.sleep();
    }

    /******** 下面的部分在按下 ctrl + c 后执行 *******/
    // 保存点云
    if (!relocation_enable && global_map_save_enable) {
        std::cout << "start saving global map, incremental pcd count: "
                  << incremental_paths.size() << std::endl;
        string pcd_save_path;
        if (timestamp_prefix) { // 添加系统时间戳
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d-%H%M%S");
            pcd_save_path = string(PACKAGE_ROOT_DIR) + "/PCD/" + ss.str() + "_" + pcd_save_name;
        }
        else { // 不添加时间戳
            pcd_save_path = string(PACKAGE_ROOT_DIR) + "/PCD/" + pcd_save_name;
        }
        std::cout << "global map target path: " << pcd_save_path << std::endl;
        try {
            PointCloudXYZI::Ptr merged_map(new PointCloudXYZI);
            /************** 使能发布全局点云时，全局地图存储到了 map_world 中 ***************/
            if (global_map_pub_enable) {
                *merged_map = *map_world;
            }
            /************** 否则是以文件形式存储，这样可以减小内存占用 ***************/
            else {
                // 合并小的 PCD 文件
                for (auto &pcd_path : incremental_paths) {
                    std::cout << "loading incremental pcd: " << pcd_path << std::endl;
                    PointCloudXYZI::Ptr tmp(new PointCloudXYZI);
                    if (pcl::io::loadPCDFile<PointType>(pcd_path, *tmp) == -1) {
                        PCL_ERROR("Couldn't read file %s \n", pcd_path.c_str());
                        continue;
                    }
                    *merged_map += *tmp;
                }
            }

            std::cout << "merged global map points: " << merged_map->size() << std::endl;

            PointCloudXYZI::Ptr filtered_map(new PointCloudXYZI);
            if (!merged_map->empty()) {
                // 降采样 PCD 文件。PCL filters should not use the same cloud as input and output.
                pcl::VoxelGrid<PointType> downSizeFilter;
                downSizeFilter.setLeafSize(0.01, 0.01, 0.01);
                downSizeFilter.setInputCloud(merged_map);
                downSizeFilter.filter(*filtered_map);
            }

            std::cout << "filtered global map points: " << filtered_map->size() << std::endl;
            pcl::io::savePCDFileBinary(pcd_save_path, *filtered_map);
            std::cout << "save pcd file: " << pcd_save_path << std::endl;

            // 删除 incremental_paths 对应的 PCD 文件
            for (auto &pcd_path : incremental_paths) {
                std::remove(pcd_path.c_str());
            }
        } catch (const std::exception &e) {
            std::cerr << "failed to save global map: " << e.what() << std::endl;
        }
    }

    rclcpp::shutdown();
    return 0;
}
