//
// Created by xiaofan on 25-1-7.
//

#ifndef ESEKF_H
#define ESEKF_H

#define G_m_s2 (9.80)

#include "common_lib.h"
#include "TopicProcess.h"
#include <omp.h>


enum StateIndex : int {
    R = 0,                   // (3 dimention) rotation in world frame
    P = 3,                   // (3 dimention) position in world frame
    V = 6,                   // (3 dimention) velocity in world frame
    BW = 9,                 // (3 dimention) IMU gyroscope bias
    BA = 12,                 // (3 dimention) IMU acceleration bias
    G = 15,                 // (3 dimention) Gravity in world frame
    STATE_TOTAL = 18
};

enum StateNoiseIndex : int {
    GYRO_NOISE = 0,        // angular velocity change
    ACC_NOISE = 3,         // linear acceleration change
    GYRO_RANDOM_WALK = 6,  // IMU gyroscope bias random walk
    ACC_RANDOM_WALK = 9,   // IMU aceleration bias random walk
    NOISE_TOTAL = 12
};

Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, double dt);

using namespace Eigen;


class EskfEstimator {
public:
    EskfEstimator();
    ~EskfEstimator();
    void UpdateByLidarIESKF(PointCloudXYZI &Dedistort_clouds_lidar);
    void UpdateByLidarESKF(PointCloudXYZI &Dedistort_clouds_lidar);
    void UpdateByWheel(std::deque<WheelData> buf);
    bool InitImu(std::deque<ImuMsgConst> imu_msgs , int init_imu_num);
    std::vector<State, Eigen::aligned_allocator<State>> get_prior_states();
    std::vector<State, Eigen::aligned_allocator<State>> get_post_states();

    State get_cur_state();
    void set_cur_state(State state_now);

    PointType transLidar2World(const PointType &point);
    PointCloudXYZIPtr transLidar2World(const PointCloudXYZI &points_lidar); // 使用指针避免内存分配，不然耗时很长
    vector<PointVector>  Nearest_Points; //每个点的最近点序列
    double last_lidar_end_time_{}; //上一帧结束时间戳

    Eigen::Vector3d mean_acc_;//加速度均值,用于计算方差
    Eigen::Vector3d mean_gyr_;//角速度均值，用于计算方差

    std::vector<State, Eigen::aligned_allocator<State>> state_vector_prior_; // 先验状态向量组，大小与一帧中IMU数据数量相同（预积分得到的状态）
    std::vector<State, Eigen::aligned_allocator<State>> state_vector_post_; // 后验状态向量组，目前大小为1，保存了一帧中最后时刻的状态（更新后得到的状态）

private:
    void PredictOnce(Eigen::Vector3d &acc_imu, Eigen::Vector3d &gyr_imu, const double& dt , const double& stamp);
    // bool calculate(const State&state, PointCloudXYZI &clouds_lidar, Eigen::MatrixXd & Z,Eigen::MatrixXd & H);
    bool calculate(const State&state, PointCloudXYZI &clouds_lidar, Eigen::MatrixXd & Z,Eigen::MatrixXd & H, bool kd_research_en);
    bool calculateEKF(const State&state, PointCloudXYZI &clouds_lidar, Eigen::MatrixXd & Z,Eigen::MatrixXd & H);
    static Eigen::Matrix<double,18,1> getErrorState18(const State &s1, const State &s2);
    State state_; // 当前状态（位置和姿态基于IMU系）

    std::vector<Pose> IMU_Pose_vector_;
    Eigen::MatrixXd Q_; // variance matrix of imu

    Eigen::Matrix3d Lidar_R_wrt_IMU;// lidar到IMU的旋转外参
    Eigen::Vector3d Lidar_T_wrt_IMU;// lidar到IMU的位置外参
    Eigen::Vector3d angvel_last;//上一帧角速度
    Eigen::Vector3d acc_s_last;//上一帧加速度

    double start_timestamp_{}; //开始时间戳
    
    int    vaild_points_num_ = 0; // 更新过程中有效点数量
    bool   b_first_frame_ = true; //是否是第一帧的第一个数据
    bool   imu_need_init_ = true; //是否需要初始化imu

    double ACC_NOISE_VAR = 1E-1;
    double GYRO_NOISE_VAR = 1E-1;
    double ACC_RANDOM_WALK_VAR = 1E-4;
    double GYRO_RANDOM_WALK_VAR = 1E-4;
};

#endif //ESEKF_H
