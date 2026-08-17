//
// Created by xiaofan on 25-1-1.
//

#ifndef COMMON_LIB_H
#define COMMON_LIB_H

// 启用并行计算
#define MP_EN
#define MP_PROC_NUM 3

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "ikd_Tree.h"
#include "YamlReader.h"
#include "type.h"
#include <fstream>

// 这里是轮速计消息定义 ROS2
// #include "yhs_can_interfaces/msg/chassis_info_fb.hpp"
#include "lio/msg/wheel_info.hpp"

// extern YamlReader reader;




struct State;

// extern std::mutex mtx_buffer; // 在TopicProcess.cpp中定义
extern KD_TREE<PointType> ikdtree; // 在main.cpp中定义
extern vector<State> s_next_v; // 在main.cpp中定义
extern PointCloudXYZI::Ptr down_effect_cloud_lidar; // 在main.cpp中定义
extern double lidar_end_time; // 在main.cpp中定义

/************ variability load from yaml ************/
extern Eigen::Vector3d imu_t_lidar; // lidar 系相对 IMU 系的外参   imu_t_lidar           T 4x4  (R t) -> T
extern Eigen::Matrix3d imu_R_lidar; // lidar 系相对 IMU 系的外参   imu_R_lidar
extern Eigen::Vector3d lidar_t_vehicle; // body 系相对 lidar 系的外参  lidar_t_vehicle
extern Eigen::Matrix3d lidar_R_vehicle; // body 系相对 lidar 系的外参  lidar_R_vehicle
extern int point_filter_num; // 点云订阅时每隔 point_filter_num 个点才读入一次
extern double blind; // 距离原点过近的点会被忽略，半径为 blind
extern bool use_box_blind; // 距离原点过近的点会被忽略，半径为 blind
extern Eigen::Vector2d box_corner_x;
extern Eigen::Vector2d box_corner_y;
extern Eigen::Vector2d box_corner_z;
extern float filter_size; // 点云降采样大小（ikdtree大小）
extern bool global_map_save_enable; // 是否启用点云保存
extern int every_n_frame_save; // 每隔多少帧保存一次单帧地图，-1表示不保存
extern bool timestamp_prefix; // 是否启用时间戳前缀
extern string pcd_save_name; // 点云保存的文件名
extern bool global_map_pub_enable; // 是否发布全局地图
extern bool ikdtree_pub_enable; // 是否发布全局地图

extern string lidar_topic_name;
extern string imu_topic_name;
extern string wheel_topic_name;
extern string image_topic_name;

extern string clouds_lidar_topic_name; // lidar系当前帧点云发布的话题名
extern string global_map_topic_name; // world系全局地图发布的话题名
extern string ikdtree_topic_name; // world系 ikdtree 发布的话题名
extern string odom_imu_topic_name; // imu位姿的话题名
extern string odom_vehicle_topic_name; // body位姿的话题名
extern bool relocation_enable; // 是否启用重定位
extern string pcd_load_name; // 重定位时，加载的点云文件名


extern bool wheel_enable; // 是否启用wheel轮速计更新
extern bool log_save_enable; // 是否记录日志，全局设置
extern bool wheel_data_log_enable; // 是否记录轮速计数据
extern bool trajectory_log_enable; // 是否记录位姿数据
extern bool time_log_enable; // 是否记录程序耗时
extern bool pub_time_log_enable; // 是否记录topic发送耗时
extern bool res_wheel_log_enable; // 是否记录轮速计残差，在 UpdateByWheel() 中使用
extern YamlReader reader;

extern int lidar_type; //     Livox = 1, Ouster = 2

extern bool high_frequency_odom;
extern float lidar_frequency;

// 返回叉乘反对称阵
inline Eigen::Matrix3d skew3d(const Eigen::Vector3d &a) {
    Eigen::Matrix3d A;
    A << 0, -a(2), a(1),
         a(2), 0, -a(0),
         -a(1), a(0), 0;
    return A;
}

void load_lio_yaml( string file_name );

extern string lio_yaml;

void load_root_yaml(string file_name);



/*************************** 枚举量定义 ****************************/
enum SLAM_MODE
{
    ONLY_LO = 0,
    ONLY_LIO = 1,
    LIVO = 2,
    FAST_LIVO1 = 3
  };

enum EKF_STATE
{
    WAIT = 0,
    VIO = 1,
    LIO = 2,
    LO = 3
};
/*************************** 枚举量定义 ****************************/


// 时间戳转换为秒 (ROS2 builtin_interfaces::msg::Time)
inline double get_time_sec(const builtin_interfaces::msg::Time &time)
{
    return time.sec + time.nanosec * 1e-9;
}

// 时间戳转换为ros时间 (ROS2)
inline builtin_interfaces::msg::Time get_ros_time(double timestamp)
{
    int32_t sec = static_cast<int32_t>(std::floor(timestamp));
    uint32_t nanosec = static_cast<uint32_t>((timestamp - std::floor(timestamp)) * 1e9);
    builtin_interfaces::msg::Time t;
    t.sec = sec;
    t.nanosec = nanosec;
    return t;
}

// 计算两点之间的距离的平方
inline float calc_dist2(PointType p1, PointType p2){
    float d2 = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d2;
}

// 用于点云按时间排序
extern bool time_list(PointType &x, PointType &y);



#endif //COMMON_LIB_H
