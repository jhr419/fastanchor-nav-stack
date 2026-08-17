//
// Created by ss on 25-5-5.
//

#include "main.h"
extern std::shared_ptr<EskfEstimator> p_eskf_estimator;

/********************************************** Save Function **********************************************/

void LIONode::save_odometry(State state, double lidar_end_time_) {
    if (lidar_end_time_ < 0) {
        lidar_end_time_ = lidar_end_time;
    }
    static bool frist = true;
    std::ofstream fout;
    // 第一次运行，清空文件并添加首行
    if (frist) {
        fout.open(string(PACKAGE_ROOT_DIR) + "/trajectory.txt", std::ios::out);
        // fout << "x y z qw qx qy qz" << endl;
        fout << "#time vx vy vz x y z qw qx qy qz" << endl;
        fout.close();
        frist = false;
    }
    // 保存轨迹
    fout.open(string(PACKAGE_ROOT_DIR) + "/trajectory.txt", std::ios::app);
    // 设置输出精度
    fout << std::fixed << std::setprecision(9); // 保留9位小数
    // fout << lidar_end_time << " " << state.v(0) << " " << state.v(1) << " " << state.v(2) << " " <<
    //         state.p(0) << " " << state.p(1) << " " << state.p(2) << " " <<
    //         state.q.w() << " " << state.q.x() << " " << state.q.y() << " " << state.q.z() << endl;
    fout << lidar_end_time_ << " " <<  state.p(0) << " " << state.p(1) << " " << state.p(2) <<
         " " << state.q.x() << " " << state.q.y() << " " << state.q.z() << " " << state.q.w() <<endl;
    fout.close();
}

/**
 * @brief 保存单帧点云到指定文件夹，文件夹名为第一次调用函数时的时间，文件名为当前帧点云时间戳
 * 注意传入为lidar系，函数中会转换为world系再输出
 * @param clouds_lidar 激光雷达坐标系下点云
 */
void LIONode::save_singel_clouds_world(PointCloudXYZI::Ptr clouds_lidar) {
    /*** 保存单帧点云 ****/
    if (every_n_frame_save > 0) {
        // lidar系转为world系
        auto Dedistort_clouds_world = p_eskf_estimator -> transLidar2World(*clouds_lidar);
        static int cnt = 0;
        static bool first = true;
        if (cnt++ % every_n_frame_save == 0) {
            static std::string folder_name;
            if (first) {
                // 当前时间作为文件夹名
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream  folder_name_ss;
                folder_name_ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d-%H%M%S");
                folder_name = string(PACKAGE_ROOT_DIR) + "/PCD/" + folder_name_ss.str();
                // 以文件夹名创建文件夹
                mkdir(folder_name.c_str(), 0777);
                first = false;
            }
            // 时间戳保留9位小数
            std::stringstream ss;
            auto time_stamp = get_ros_time(lidar_end_time);
            ss << std::fixed << std::setprecision(9) << get_time_sec(time_stamp);
            // 补充完整路径
            std::string file_path = folder_name + "/" + ss.str()+".pcd";
            cout << "[timer_200HZ_callback] save pcd file: " << file_path << std::endl;
            pcl::io::savePCDFileBinary(file_path, *Dedistort_clouds_world);
        }
    }
}

