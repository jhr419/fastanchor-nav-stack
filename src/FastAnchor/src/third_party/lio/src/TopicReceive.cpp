//
// Created by xiaofan on 25-1-1.
//

/** @file TopicProcess.cpp
 * @brief 包含一个TopicProcess类，用于处理接受到的lidar和imu话题，并完成打包
 * receive_points()函数：将点云数据存入点云缓冲队列(cloud_buffer)
 * receive_imu()函数：将IMU数据存入IMU缓冲队列(imu_buffer)
 * sync_packages()函数：将点云数据和IMU数据从缓存队列中弹出，打包成一帧，存入meas中
 * << 注意meas中只能存放一帧数据，每次成功调用sync_packages()都会覆盖之前的数据 >>
 * @date 2025/1/7
 */

#include "../include/TopicProcess.h"
#include <sensor_msgs/msg/point_cloud2.hpp>  // 添加头文件
#include <pcl_conversions/pcl_conversions.h>  // 用于转换 PointCloud2 <-> PCL 点云
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "main.h"
#include "type.h"
// std::mutex mtx_buffer;
// std::condition_variable sig_buffer; //条件变量

const double THRE_DIFF_IMU = 10; // 异常值处理阈值
/**
 * @brief:
 * 将点云数据存入点云缓冲队列(cloud_buffer)
 * 在时间戳出现回溯时，清除缓存（播包时可能出现）
 * 在每次接收到 lidar topic 后执行
 * @param msg: 接收到的点云数据
 */
void TopicProcess::receive_points_ntu(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    if (FSM::current_state == FSM::State::Initializing) return;
    cloud_topic_count_++;
    double timestamp = get_time_sec(msg->header.stamp);

    // 将 PointCloud2 转换为 PCL 格式方便处理
    pcl::PointCloud<ouster_ros::Point>::Ptr pcl_cloud(new pcl::PointCloud<ouster_ros::Point>);
    pcl::fromROSMsg(*msg, *pcl_cloud);
    int plsize = pcl_cloud->size();

    if (plsize <= 1) {
        std::cerr << "[receive_points] too few points\n";
        return;
    }

    static double sum_time = 0;
    static int cnt = 0;
    static bool first = true;

    if (first) {
        last_timestamp_lidar_ = timestamp;
        first = false;
    } else {
        double time_diff = timestamp - last_timestamp_lidar_;
        if (sum_time < 5) {
            cnt++;
            sum_time += time_diff;
        } else {
            std::cout << "[receive_points] Lidar frequency: " << cnt / sum_time << " HZ\n\n";
            cnt = 0;
            sum_time = 0;
        }
        last_timestamp_lidar_ = timestamp;
    }

    PointCloudXYZI cloud_xyzi;  // 假设 PointCloudXYZI 是你自定义的点类型
    cloud_xyzi.reserve(plsize);

    uint valid_num = 0;

    for (int i = 0; i < plsize; ++i)  // 注意索引从 0 开始
    {
        const auto& pt = pcl_cloud->points[i];
        if ((valid_num % point_filter_num == 0))
        {
            pcl::PointXYZINormal pt_normal;
            pt_normal.x = pt.x;
            pt_normal.y = pt.y;
            pt_normal.z = pt.z;
            pt_normal.normal_x = 0;
            pt_normal.normal_y = 0;
            pt_normal.normal_z = 0;
            pt_normal.intensity = pt.intensity;
            pt_normal.curvature = pt.t * 1.e-6f;
            if (use_box_blind){ // 长方体盒子筛选
                // 只要 x, y, z 任意一个维度超出 [min, max] 范围，点就在盒子外
                if (pt.x < box_corner_x[0] || pt.x > box_corner_x[1] ||
                    pt.y < box_corner_y[0] || pt.y > box_corner_y[1] ||
                    pt.z < box_corner_z[0] || pt.z > box_corner_z[1])
                {
                    cloud_xyzi.emplace_back(pt_normal);
                }
            }else { // 球形筛选
                if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > blind * blind) { // 球外的点加入点云
                    cloud_xyzi.emplace_back(pt_normal);
                }
            }
        }
    }

    std::cout << "[receive_points] " << cloud_xyzi.size() << endl;

    if (!cloud_xyzi.empty()) {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI(std::move(cloud_xyzi)));
        buffers_->pushBackDualSafe(buffers_->lidar_buf, ptr,
                                   buffers_->lidar_base_time_buf, timestamp);
        buffers_->pushBackSafe(buffers_->trigger_buf, pair(Trigger::Lidar, timestamp));
        buffers_->notifyAll();
    }
}

void TopicProcess::receive_points_velodyne(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    // === 1. Initial Setup and Frequency Calculation ===
    cloud_topic_count_++;
    double timestamp = get_time_sec(msg->header.stamp);

    // Frequency calculation logic from your original function
    static double sum_time = 0;
    static int cnt = 0;
    static bool first = true;
    constexpr int MAX_LINE_NUM = 64;

    if (first) {
        last_timestamp_lidar_ = timestamp;
        first = false;
    } else {
        double time_diff = timestamp - last_timestamp_lidar_;
        if (sum_time < 5.0) {
            cnt++;
            sum_time += time_diff;
        } else {
            std::cout << "[velodyne_callback] Lidar frequency: " << cnt / sum_time << " HZ\n\n";
            cnt = 0;
            sum_time = 0;
        }
        last_timestamp_lidar_ = timestamp;
    }

    // === 2. Convert ROS Msg to PCL PointCloud ===
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize <= 1) {
        std::cerr << "[velodyne_callback] too few points\n";
        return;
    }

    bool is_first[MAX_LINE_NUM];
    double yaw_fp[MAX_LINE_NUM] = {0};     // yaw of first scan point
    double omega_l = 3.61;                 // scan angular velocity
    float yaw_last[MAX_LINE_NUM] = {0.0};  // yaw of last scan point
    float time_last[MAX_LINE_NUM] = {0.0}; // last offset time

    memset(is_first, true, sizeof(is_first));
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
        if (pl_orig.points[i].ring == layer_first)
        {
            yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
            break;
        }
    }

    PointCloudXYZI cloud_xyzi; // Local variable for processed points
    cloud_xyzi.reserve(plsize);

    // === 4. Process Each Point ===
    for (int i = 0; i < plsize; ++i) {
        pcl::PointXYZINormal pt;

        // Check scan line/ring ID
        int layer = pl_orig.points[i].ring;

        pt.x = pl_orig.points[i].x;
        pt.y = pl_orig.points[i].y;
        pt.z = pl_orig.points[i].z;
        pt.intensity = pl_orig.points[i].intensity;

        // === 5. Handle Offset Time and store it in 'curvature' field (in ms) ===
        // Calculate offset time based on yaw angle
        double yaw_angle = atan2(pt.y, pt.x) * 180.0 / M_PI; // degrees

        if (is_first[layer]) {
            yaw_fp[layer] = yaw_angle;
            is_first[layer] = false;
            pt.curvature = 0.0f; // First point of a scan line has 0 offset
            yaw_last[layer] = yaw_angle;
            time_last[layer] = pt.curvature;
        } else {
            // Angle wrap-around logic from the reference code
            if (yaw_angle <= yaw_fp[layer]) {
                pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (pt.curvature < time_last[layer]) {
                pt.curvature += 360.0 / omega_l;
            }

            yaw_last[layer] = yaw_angle;
            time_last[layer] = pt.curvature;
        }

        // === 6. Filtering (from your original function) ===
        if (i % point_filter_num == 0) { // point_filter_num is a class member
            if (use_box_blind){ // 长方体盒子筛选
                // 只要 x, y, z 任意一个维度超出 [min, max] 范围，点就在盒子外
                if (pt.x < box_corner_x[0] || pt.x > box_corner_x[1] ||
                    pt.y < box_corner_y[0] || pt.y > box_corner_y[1] ||
                    pt.z < box_corner_z[0] || pt.z > box_corner_z[1])
                {
                    cloud_xyzi.emplace_back(pt);
                }
            }else { // 球形筛选
                if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > blind * blind) { // 球外的点加入点云
                    cloud_xyzi.emplace_back(pt);
                }
            }
        }
    }

    // === 7. Buffering (from your original function) ===
    if (!cloud_xyzi.empty()) {
        // Calculate end time using the curvature field, which holds the offset time in ms
        double lidar_end_time = timestamp + cloud_xyzi.back().curvature / 1e3; // Convert ms back to seconds

        PointCloudXYZI::Ptr ptr(new PointCloudXYZI(std::move(cloud_xyzi)));
        buffers_->pushBackDualSafe(buffers_->lidar_buf, ptr,
                                   buffers_->lidar_base_time_buf, timestamp);
        buffers_->pushBackSafe(buffers_->trigger_buf, std::pair(Trigger::Lidar, lidar_end_time));
        buffers_->notifyAll();
    }
}

void TopicProcess::receive_points(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg)
{

    cloud_topic_count_++;
    double timestamp = get_time_sec(msg->header.stamp);
    int plsize = msg->point_num;
    if (plsize <= 1) {
        std::cerr << "[receive_points] too few points\n";
        return;
    }

    static constexpr int N_SCANS = 6;  // TODO: 这里修改了一下
    static double sum_time = 0;
    static int cnt = 0;
    static bool first = true;

    if (first) {
        last_timestamp_lidar_ = timestamp;
        first = false;
    } else {
        double time_diff = timestamp - last_timestamp_lidar_;
        if (sum_time < 5) {
            cnt++;
            sum_time += time_diff;
        } else {
            std::cout << "[receive_points] Lidar frequency: " << cnt / sum_time << " HZ\n\n";
            cnt = 0;
            sum_time = 0;
        }
        last_timestamp_lidar_ = timestamp;
    }

    PointCloudXYZI cloud_xyzi;  // 局部变量，线程安全
    cloud_xyzi.reserve(plsize);

    uint valid_num = 0;
    for (int i = 1; i < plsize; ++i)
    {
        if ((msg->points[i].line < N_SCANS) &&
            ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
        {
            valid_num++;
            if (valid_num % point_filter_num == 0)
            {
                pcl::PointXYZINormal pt;
                pt.x = msg->points[i].x;
                pt.y = msg->points[i].y;
                pt.z = msg->points[i].z;
                pt.intensity = msg->points[i].reflectivity;
                pt.curvature = msg->points[i].offset_time / 1e6f; // ms

                if (use_box_blind){ // 长方体盒子筛选
                    // 只要 x, y, z 任意一个维度超出 [min, max] 范围，点就在盒子外
                    if (pt.x < box_corner_x[0] || pt.x > box_corner_x[1] ||
                        pt.y < box_corner_y[0] || pt.y > box_corner_y[1] ||
                        pt.z < box_corner_z[0] || pt.z > box_corner_z[1])
                    {
                        cloud_xyzi.emplace_back(pt);
                    }
                }else { // 球形筛选
                    if (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > blind * blind) { // 球外的点加入点云
                        cloud_xyzi.emplace_back(pt);
                    }
                }
            }
        }
    }

    double lidar_end_time = get_time_sec(msg->header.stamp) + msg->points.back().offset_time/1E9;

    if (!cloud_xyzi.empty()) {
        PointCloudXYZI::Ptr ptr(new PointCloudXYZI(std::move(cloud_xyzi)));
        buffers_->pushBackDualSafe(buffers_->lidar_buf, ptr,
                                   buffers_->lidar_base_time_buf, timestamp);
        buffers_->pushBackSafe(buffers_->trigger_buf,pair(Trigger::Lidar, lidar_end_time));
        buffers_->notifyAll();
    }
}




/**
 * @brief:
 * 将 IMU 数据存入 IMU 缓冲队列(imu_buffer)
 * 在时间戳出现回溯时，清除缓存（播包时可能出现）
 * 在每次接收到 IMU topic后执行
 * @param msg: 接收到的IMU数据
 */
void TopicProcess::receive_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
    imu_topic_count_++;
    double timestamp = get_time_sec(msg->header.stamp);

    auto now = std::chrono::high_resolution_clock::now();
    double time_sys = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() * 1e-6;


    static sensor_msgs::msg::Imu::ConstSharedPtr last_imu_msg = msg;
    Eigen::Vector3d last_ang(last_imu_msg->angular_velocity.x,
                             last_imu_msg->angular_velocity.y,
                             last_imu_msg->angular_velocity.z);
    Eigen::Vector3d curr_ang(msg->angular_velocity.x,
                             msg->angular_velocity.y,
                             msg->angular_velocity.z);

    if ((curr_ang - last_ang).norm() > THRE_DIFF_IMU) {
        std::cerr << "[receive_imu] imu_acc outliers detected!\n";
        sensor_msgs::msg::Imu::SharedPtr corrected_msg = std::make_shared<sensor_msgs::msg::Imu>(*last_imu_msg);
        corrected_msg->header.stamp = msg->header.stamp;
        buffers_->pushBackSafe(buffers_->imu_buf, corrected_msg);
    } else {
        last_imu_msg = msg;
        buffers_->pushBackSafe(buffers_->imu_buf, msg);
    }

    /************** frequency calculate ****************/
    static bool first = true;
    if (first) {
        last_timestamp_imu_ = timestamp;
        first = false;
    } else {
        double time_diff = timestamp - last_timestamp_imu_;
        static double sum_time = 0;
        static int i = 0;
        if (sum_time < 5) {
            i++;
            sum_time += time_diff;
        } else {
            std::cout << "[receive_imu] IMU frequency: " << i / sum_time << " HZ" << std::endl;
            i = 0;
            sum_time = 0;
        }
        last_timestamp_imu_ = timestamp;
    }

    buffers_->notifyAll();
}

/**
 * @brief:
 * 将轮速计数据存入轮速缓冲队列
 * @param msg: 接收到的轮速计消息
 */
void TopicProcess::receive_wheel(const lio::msg::WheelInfo::ConstSharedPtr& msg)
{
    wheel_topic_count_++;
    /**
     * 按照象限划分，取出四个轮子角度 angle 和速度 v
     * 以车中心为原点，向前为x轴，向左为y轴
     */
    double angle_L = (msg->front_angle_fb_l - msg->rear_angle_fb_l) / 2.0;
    double angle_R = (msg->front_angle_fb_r - msg->rear_angle_fb_r) / 2.0;
    double v_L = (msg->lf_wheel_fb_velocity + msg->lr_wheel_fb_velocity) / 2.0;
    double v_R = (msg->rf_wheel_fb_velocity + msg->rr_wheel_fb_velocity) / 2.0;

    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    WheelData wheel_data{};
    wheel_data.timestamp = timestamp;
    wheel_data.angle_L = angle_L * M_PI / 180.0;
    wheel_data.angle_R = angle_R * M_PI / 180.0;
    wheel_data.v_L = v_L;
    wheel_data.v_R = v_R;

    buffers_->pushBackSafe(buffers_->wheel_buf, wheel_data);

    buffers_->notifyAll();
}






std::deque<WheelData> TopicProcess::get_wheel_buffer() const {
    std::scoped_lock lk(buffers_->mtx);
    return buffers_->wheel_buf;  // ✅ 返回拷贝
}

