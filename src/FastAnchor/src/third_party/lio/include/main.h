//
// Created by xiaofan on 25-4-2.
//

#ifndef MAIN_H
#define MAIN_H

// ROS1 -> ROS2 头文件迁移
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include "YamlReader.h"
#include "TopicProcess.h"
#include "ESEKF.h"
#include <nav_msgs/msg/path.hpp>
#include <csignal>  // 包含信号处理相关的头文件

#include <functional>
#include <map>

struct TimeRecord {
    double lidar_end_time;
    double lidar_beg_time;
    double img_beg_time;
    double img_end_time;
    double imu_beg_time;
    double imu_end_time;
};

namespace FSM {
    enum class State { Waitting, Initializing, IMUProcess, LidarProcess, ImgProcess /*…*/ };
    enum class Event { IMUTriger, InitEnd, LidarTriger, LidarEnd, ImgTriger, ImgEnd /*…*/ };
    struct Transition {
        State next_state;
        std::function<void()> action;
    };
    using Key = std::pair<State,Event>;
    // 状态表，和 dispatch() 声明
    extern const std::map<Key, Transition> kTransitionTable;
    extern State current_state;
    void dispatch(Event e);
    // 辅助打印函数
    inline const char* to_string(State s) {
        switch(s) {
            case State::Waitting:      return "Waitting";
            case State::Initializing:  return "Initializing";
            case State::IMUProcess:    return "IMUProcess";
            case State::LidarProcess:  return "LidarProcess";
            case State::ImgProcess:    return "ImgProcess";
            default:                   return "UnknownState";
        }
    }
    inline const char* to_string(Event e) {
        switch(e) {
            case Event::IMUTriger:   return "IMUTriger";
            case Event::InitEnd:     return "InitEnd";
            case Event::LidarTriger: return "LidarTriger";
            case Event::LidarEnd:    return "LidarEnd";
            case Event::ImgTriger:   return "ImgTriger";
            case Event::ImgEnd:      return "ImgEnd";
            default:                 return "UnknownEvent";
        }
    }
}

class LIONode : public rclcpp::Node {
public:
    LIONode();
    void initROS(); // 在 yaml 配置加载后调用, 创建 发布器/订阅器/定时器

    void publishPathOnce(const std::string& path);
    void publish_imu_odometry(State state, double lidar_end_time_ = -1.0);
    void publish_body_odometry(State state, double lidar_end_time_ = -1.0);
    void publish_Dedistort_clouds_lidar(const PointCloudXYZI::Ptr &cloud);
    void publishStaticTransform();
    void publishGlobalMap();
    void publishIKDTree();
    void save_odometry(State state, double lidar_end_time_ = -1.0);
    void save_singel_clouds_world(PointCloudXYZI::Ptr clouds_lidar);
    static void lasermap_fov_segment();
    void timer_1HZ_callback();
    void timer_10HZ_callback();
    void timer_100HZ_callback();
    void timer_500HZ_callback();


private:

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clouds_lidar_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ikdtree_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_body_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
    rclcpp::SubscriptionBase::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<lio::msg::WheelInfo>::SharedPtr wheel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_500HZ_;
    rclcpp::TimerBase::SharedPtr timer_100HZ_;
    rclcpp::TimerBase::SharedPtr timer_10HZ_;
    rclcpp::TimerBase::SharedPtr timer_1HZ_;

    // Interface-compat switch: when FastAnchor provides the TF tree (map->odom->base)
    // and the yifan_lio_adapter mirrors FAST-LIO's camera_init->body, yifanLIO's own
    // world/IMU/lidar/body TF would create a second, disconnected tree. publish_tf=false
    // disables yifanLIO's dynamic and static TF broadcasts without touching the algorithm.
    bool publish_tf_ = true;

    static void map_incremental(PointCloudXYZI & lidar_clouds);
};

// Callback Function (ROS2)
void pcl_cbk_custom(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg);
void pcl_cbk_pc2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
void wheel_cbk(const lio::msg::WheelInfo::ConstSharedPtr& msg);


#endif //MAIN_H
