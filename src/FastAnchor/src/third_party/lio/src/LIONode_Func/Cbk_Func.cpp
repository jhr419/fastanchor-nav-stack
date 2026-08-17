//
// Created by ss on 25-5-5.
//

#include "main.h"
#include "type.h"

extern std::shared_ptr<TopicProcess> p_topic_process;

/********************************************** Callback Function **********************************************/

/**
 * @brief 接收到点云后的回调函数, 支持livox自定义消息类型
 * @param msg 点云数据
 */
void pcl_cbk_custom(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg) {
    if (FSM::current_state == FSM::State::Initializing || FSM::current_state == FSM::State::Waitting) return;
    p_topic_process->receive_points(msg);
}

/**
 * @brief 接收到点云后的回调函数, 支持标准点云消息类型
 * @param msg 点云数据
 */
void pcl_cbk_pc2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)  {
    if (FSM::current_state == FSM::State::Initializing || FSM::current_state == FSM::State::Waitting) return;
    switch (lidar_type) {
        case LidarType::Livox:
            break;
        case LidarType::Ouster:
            p_topic_process->receive_points_ntu(msg);
            break;
        case LidarType::Velodyne:
            p_topic_process->receive_points_velodyne(msg);
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("lio"), "[LIO] Unknown lidar type.");
    }
}

// #include "IMUProcess.h"
// IMUProcess imu_process;

void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
    static bool first_imu = true;
    if (first_imu) { FSM::dispatch(FSM::Event::IMUTriger); first_imu = false; }
    p_topic_process->receive_imu(msg);
}

void wheel_cbk(const lio::msg::WheelInfo::ConstSharedPtr& msg) {
    p_topic_process->receive_wheel(msg);
    /***** 下面是文件记录 ******/
    if (wheel_data_log_enable & log_save_enable) {
        uint nanosec_num = msg->header.stamp.nanosec;
        uint sec_num = msg->header.stamp.sec;
        static double start_stamp = sec_num + nanosec_num * 1e-9;
        // double rele_stamp = sec_num + nanosec_num * 1e-9 - start_stamp;
        std::ofstream fout;
        static bool frist = true;
        // 第一次运行，清空文件并添加首行
        if (frist) {
            fout.open(string(PACKAGE_ROOT_DIR) + "/wheel_data.txt", std::ios::out);
            fout << "stamp lf_v lr_v rf_v rr_v ang_fl ang_fr ang_rl ang_rr" << endl;
            fout.close();
            frist = false;
        }
        fout.open(string(PACKAGE_ROOT_DIR)+"/wheel_data.txt", ios::app);
        // 设置输出精度
        fout << std::fixed << std::setprecision(9); // 保留9位小数
        fout << start_stamp << " "
             << msg->lf_wheel_fb_velocity << " "
             << msg->lr_wheel_fb_velocity << " "
             << msg->rf_wheel_fb_velocity << " "
             << msg->rr_wheel_fb_velocity << " "
             << msg->front_angle_fb_l << " "
             << msg->front_angle_fb_r << " "
             << msg->rear_angle_fb_l << " "
             << msg->rear_angle_fb_r << endl;
        fout.close();
    }
}

