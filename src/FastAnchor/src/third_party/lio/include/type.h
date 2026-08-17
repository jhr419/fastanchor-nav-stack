//
// Created by xiaofan on 25-4-27.
//

#ifndef TYPE_H
#define TYPE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/imu.hpp>

// 触发器枚举（判断来了哪一个数据）
enum class Trigger {
    Lidar = 0,
    Image = 1,
};

// 轮速数据结构体，转化为了阿克曼模型，原有4个角度和4个速度均被缩减到的2个
struct WheelData
{
    double timestamp; // 时间戳
    double angle_L; // 左前轮角度
    double angle_R; // 右前轮角度
    double v_L ; // 左前轮速度
    double v_R ; // 右前轮速度
};

// imu frame
struct State {
    Eigen::Quaterniond q;  // w_q_imu
    Eigen::Vector3d p; // w_p_imu
    Eigen::Vector3d v; // w_v_imu
    Eigen::Vector3d bw;
    Eigen::Vector3d ba;
    Eigen::Vector3d g; // ??  world
    Eigen::MatrixXd P;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


// 对于 IMU 预积分中的 IMU_POSE 来说，offset_time 是相对于 lidar_begin_time 的偏移时间，可能为负值
struct Pose {
    Eigen::Quaterniond q; // world
    Eigen::Vector3d p; // world
    Eigen::Vector3d v; // world
    Eigen::Vector3d gyr; // body
    Eigen::Vector3d acc; // world
    double offset_time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/* =====  常用点云类型别名 ===== */
using PointType = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;
using PointCloudXYZIPtr = PointCloudXYZI::Ptr; // 共享指针，不必担心局部定义而被被释放
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
/* =====  其他消息 / 图像别名（可按需增加） ===== */
using ImuMsg      = sensor_msgs::msg::Imu;
using ImuMsgConst = sensor_msgs::msg::Imu::ConstSharedPtr;


using StateVec = std::vector<State, Eigen::aligned_allocator<State>>;



class Img_Update;
typedef std::shared_ptr<Img_Update> ImgUpdatePtr;
typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;

#endif //TYPE_H
