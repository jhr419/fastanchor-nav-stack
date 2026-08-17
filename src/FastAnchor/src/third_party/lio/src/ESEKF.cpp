//
// Created by xiaofan on 25-1-7.
//




#include "ESEKF.h"
#include <cmath>
#include "common_lib.h"

// #include <tbb/parallel_reduce.h>

// 用于点云按时间排序
bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

static Eigen::Vector3d SO3Log(const Eigen::Matrix3d&SO3 ){
    double theta = (SO3.trace()>3-1e6)?0:acos((SO3.trace()-1)/2);
    Eigen::Vector3d so3(SO3(2,1)-SO3(1,2),SO3(0,2)-SO3(2,0),SO3(1,0)-SO3(0,1));
    return fabs(theta)<0.001?(0.5*so3):(0.5*theta/sin(theta)*so3);
}


static Eigen::Matrix3d A_T(const Eigen::Vector3d& v){
    Eigen::Matrix3d res;
    double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    double norm = std::sqrt(squaredNorm);
    // 避免下溢
    if(norm <1e-11){
        res = Eigen::Matrix3d::Identity();
    }
    else{
        // res = Eigen::Matrix3d::Identity() + (1 - std::cos(norm)) / squaredNorm * skew3d(v) + (1 - std::sin(norm) / norm) / squaredNorm * skew3d(v) * skew3d(v);
        res = Eigen::Matrix3d::Identity() + 1/2 * skew3d(v);
    }
    return res;
}


Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, double dt)
{
    double ang_vel_norm = ang_vel.norm();
    Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();

    if (ang_vel_norm < 0.0000001)
    {
        cerr << "[Exp] ang_vel_norm is too small" << endl;
        return Eye3;

    }
    Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix3d K;

    K << skew3d(r_axis);
    double r_ang = ang_vel_norm * dt;

    /// Roderigues Transformation
    return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
}

template<typename PointType,typename T>
static PointType transformPoint(PointType point,const Eigen::Quaternion<T> &q,const Eigen::Matrix<T,3,1> &t) {
    Eigen::Matrix<T,3,1> ep = {point.x,point.y,point.z};
    ep = q * ep + t;
    point.x = ep.x();
    point.y = ep.y();
    point.z = ep.z();
    return point;
}
template<typename PointType,typename T>
static PointType transformPoint(PointType point,const Eigen::Matrix<T,3,3> &transMatrix,const Eigen::Matrix<T,3,1> &t){
    Eigen::Matrix<T,3,1> ep = {point.x,point.y,point.z};
    ep = transMatrix * ep + t;
    point.x = ep.x();
    point.y = ep.y();
    point.z = ep.z();
    return point;
}



EskfEstimator::EskfEstimator() {
    state_.q = Eigen::Quaterniond(1, 0, 0, 0);
    state_.p = Eigen::Vector3d(0, 0, 0);
    state_.v = Eigen::Vector3d(0, 0, 0);
    state_.bw = Eigen::Vector3d(0, 0, 0);
    state_.ba = Eigen::Vector3d(0, 0, 0);
    state_.g = Eigen::Vector3d(0, 0, -G_m_s2);
    state_.P = Eigen::Matrix<double, StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL>::Identity();
    state_.P.block<3,3>(StateIndex::P, StateIndex::P) = Eigen::Matrix3d::Identity() * 0.01;
    state_.P.block<3,3>(StateIndex::V, StateIndex::V) = Eigen::Matrix3d::Identity() * 0.01;
    state_.time = 0;
    mean_acc_ = Eigen::Vector3d(0, 0, 0);
    mean_gyr_ = Eigen::Vector3d(0, 0, 0);
    Q_ = Eigen::Matrix<double, StateNoiseIndex::NOISE_TOTAL, StateNoiseIndex::NOISE_TOTAL>::Identity();
    Q_.block<3, 3>(StateNoiseIndex::ACC_NOISE, StateNoiseIndex::ACC_NOISE) =
            ACC_NOISE_VAR * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::GYRO_NOISE, StateNoiseIndex::GYRO_NOISE) =
            GYRO_NOISE_VAR * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::ACC_RANDOM_WALK, StateNoiseIndex::ACC_RANDOM_WALK) =
            ACC_RANDOM_WALK_VAR * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK) =
            GYRO_RANDOM_WALK_VAR * Eigen::Matrix3d::Identity();
}

EskfEstimator::~EskfEstimator() = default;


/**
 * @brief 利用imu数据进行一次状态预测，结果压入state_vector中
 * @param acc_imu 加速度
 * @param gyr_imu 角速度
 * @param dt 时间步长
 * @param stamp 距离起始时刻的时间
 */
void EskfEstimator::PredictOnce(Eigen::Vector3d &acc_imu, Eigen::Vector3d &gyr_imu, const double& dt , const double& stamp) {
    /*--------------------------------- 1. 预处理 ---------------------------------*/
    acc_imu = acc_imu * G_m_s2 / mean_acc_.norm();
    // 输出mean_acc_.norm()  ——  0.992646  ——  0.995824
    Eigen::Matrix3d R = state_.q.toRotationMatrix();
    /*--------------------------------- 2. A、U、F、V ---------------------------------*/
    // 创建一个 18x18 的全零矩阵
    Eigen::MatrixXd mat_A(static_cast<int>(StateIndex::STATE_TOTAL), static_cast<int>(StateIndex::STATE_TOTAL)); // g?
    mat_A.setZero(); // 确保矩阵是全零的
    // 创建临时使用的矩阵和向量
    Eigen::Matrix3d submat;
    Eigen::Vector3d temp_vec;
    /* A(δθ,δθ) */
    temp_vec = gyr_imu - state_.bw;
    submat = -skew3d(temp_vec);
    mat_A.block<3,3>(StateIndex::R,StateIndex::R) = submat;
    /* A(δθ,δbw) */
    mat_A.block<3,3>(StateIndex::R,StateIndex::BW) = -Eigen::Matrix3d::Identity();
    /* A(δp,δv) */
    mat_A.block<3,3>(StateIndex::P,StateIndex::V) = Eigen::Matrix3d::Identity();
    /* A(δv,δθ) */
    temp_vec = acc_imu - state_.ba;
    submat = R * -skew3d(temp_vec);
    mat_A.block<3,3>(StateIndex::V,StateIndex::R) = submat;
    /* A(δv,δba) */
    mat_A.block<3,3>(StateIndex::V,StateIndex::BA) = -R;
    /* 把重力设成可估计状态 */
    mat_A.block<3,3>(StateIndex::V, StateIndex::G) = Eigen::Matrix3d::Identity();
    /* --------- U --------- */
    // 创建一个 18x12 的全零矩阵
    Eigen::MatrixXd mat_U(static_cast<int>(StateIndex::STATE_TOTAL), static_cast<int>(StateNoiseIndex::NOISE_TOTAL));
    mat_U.setZero(); // 确保矩阵是全零的
    mat_U.block<3,3>(StateIndex::R,StateIndex::R) = -Eigen::Matrix3d::Identity();
    mat_U.block<3,3>(StateIndex::V,StateIndex::P) = -R;
    mat_U.block<3,3>(StateIndex::BW,StateIndex::V) = Eigen::Matrix3d::Identity();
    mat_U.block<3,3>(StateIndex::BA,StateIndex::BW) = Eigen::Matrix3d::Identity(); // StateIndex
    /* --------- 离散化 --------- */
    Eigen::MatrixXd mat_F(static_cast<int>(StateIndex::STATE_TOTAL), static_cast<int>(StateIndex::STATE_TOTAL));
    mat_F = Eigen::Matrix<double,StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL>::Identity() + dt * mat_A;
    Eigen::MatrixXd mat_V(static_cast<int>(StateIndex::STATE_TOTAL), static_cast<int>(StateIndex::BA));
    mat_V = dt * mat_U;
    /*--------------------------------- 3. 均值预测 ---------------------------------*/
    /* 3.1  姿态 (四元数左乘) */
    Eigen::Vector3d dtheta = (gyr_imu - state_.bw) * dt;
    double th = dtheta.norm();
    Eigen::Quaterniond dq(1.0, 0.0, 0.0, 0.0);
    if (th > 1e-12) {
        Eigen::Vector3d axis = dtheta / th;
        double half = 0.5 * th;
        dq.w() = std::cos(half);
        dq.vec() = axis * std::sin(half);
    }
    state_.q = (state_.q * dq).normalized();
    /* 3.2  线加速度（世界系） */
    Eigen::Vector3d acc_world = state_.q * (acc_imu - state_.ba) + state_.g;
    /* 3.3  位置、速度 */
    state_.p += state_.v * dt + 0.5 * acc_world * dt * dt;
    state_.v += acc_world * dt;
    /*--------------------------------- 4. 协方差传播 ---------------------------------*/
    state_.P = mat_F * state_.P * mat_F.transpose() + mat_V * Q_ * mat_V.transpose();
    state_.time = stamp;
    /*--------------------------------- 5. 记录 ---------------------------------*/

    // //将state的内容输出为txt
    // static bool first = true;
    // std::ofstream fout;
    //
    // // 第一次运行，清空文件并添加首行
    // if (first) {
    //     fout.open(std::string(PACKAGE_ROOT_DIR) + "/states.txt", std::ios::out);
    //     fout << "time,qx,qy,qz,qw,px,py,pz,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz,gx,gy,gz" << std::endl;
    //     fout.close();
    //     first = false;
    // }
    //
    // // 保存状态
    // fout.open(std::string(PACKAGE_ROOT_DIR) + "/states.txt", std::ios::app);
    // // 设置输出精度
    // fout << std::fixed << std::setprecision(9); // 保留9位小数
    // // 输出 State 的内容
    // fout << state_.time << ","
    //      << state_.q.x() << ","  << state_.q.y() << ","  << state_.q.z() << "," << state_.q.w() << ","
    //      << state_.p.x() << ","  << state_.p.y() << ","  << state_.p.z() << ","
    //      << state_.v.x() << ","  << state_.v.y() << ","  << state_.v.z() << ","
    //      << state_.bw.x() << "," << state_.bw.y() << "," << state_.bw.z() << ","
    //      << state_.ba.x() << "," << state_.ba.y() << "," << state_.ba.z() << ","
    //      << state_.g.x() << ","  << state_.g.y() << ","  << state_.g.z() << std::endl;
    // fout.close();


    state_vector_prior_.push_back(state_);
    // 保存IMU的位姿
    Pose IMU_Pose;
    IMU_Pose.q = state_.q;
    IMU_Pose.p = state_.p;
    IMU_Pose.v = state_.v;
    IMU_Pose.acc = acc_world;
    IMU_Pose.gyr = gyr_imu - state_.bw;
    IMU_Pose.offset_time = stamp;

    IMU_Pose_vector_.push_back(IMU_Pose);
}


static Eigen::Matrix3d so3Exp(const Eigen::Vector3d &so3 ){
    Eigen::Matrix3d  SO3;
    double so3_norm = so3.norm();
    if (so3_norm<=0.0000001)
    {
        SO3.setIdentity();
        return SO3;
    }

    Eigen::Matrix3d so3_skew_sym = skew3d(so3);
    SO3 = Eigen::Matrix3d::Identity()+(so3_skew_sym/so3_norm)*sin(so3_norm)+(so3_skew_sym*so3_skew_sym/(so3_norm*so3_norm))*(1-cos(so3_norm));
    return SO3;
}

// ESKF 版本，与 IESKF 相区分
void EskfEstimator::UpdateByLidarESKF(PointCloudXYZI &Dedistort_clouds_lidar) {
    state_vector_post_.clear();
    State s_next = state_;
    const double delta = 1E-5;
    int point_num = Dedistort_clouds_lidar.points.size();
    Nearest_Points.resize(point_num);
    if (point_num < 10) {
        state_vector_post_.push_back(s_next);
        cout << "too few points" << endl;
        return;
    }
    Eigen::MatrixXd K;
    Eigen::MatrixXd H_k;
    Eigen::MatrixXd Z_k;
    /******* 计算残差z_k和雅可比H_k ********/
    Eigen::MatrixXd R_inv;
    // 调用函数计算 Z 和 H ，其中 Z_k 是一个列向量
    if (!calculate(s_next,Dedistort_clouds_lidar,Z_k,H_k,true) || H_k.rows() == 0 || Z_k.rows() != H_k.rows()) {
        state_vector_post_.push_back(s_next);
        return;
    }
    // 输出残差
    Eigen::MatrixXd H_kt = H_k.transpose();
    // R为 0.001E
    K = (H_kt * H_k + s_next.P.inverse() * 0.001 ).inverse() * H_kt;
    // 计算 X 的增量
    Eigen::MatrixXd left = -1 * K * Z_k;
    Eigen::MatrixXd update_x = left;
    // update_x 是一个 18*1 的列向量，使用 maxCoeff() 函数来获取最大值
    if (update_x.maxCoeff() > delta)
    {
        // cout << "update_x: " << update_x.maxCoeff() << endl;
    }
    // 更新X
    s_next.q = s_next.q.toRotationMatrix() * so3Exp(update_x.block<3,1>(StateIndex::R,0));
    s_next.q.normalize();
    s_next.p  = s_next.p  + update_x.block<3,1>(StateIndex::P,0);
    s_next.v  = s_next.v  + update_x.block<3,1>(StateIndex::V,0);
    s_next.bw = s_next.bw + update_x.block<3,1>(StateIndex::BW,0);
    s_next.ba = s_next.ba + update_x.block<3,1>(StateIndex::BA,0);
    s_next.g  = s_next.g  + update_x.block<3,1>(StateIndex::G,0);
    s_next.P = ( Eigen::Matrix<double,StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL>::Identity() - K * H_k ) * s_next.P;
    s_next_v.push_back(s_next);
    state_ = s_next;
    state_vector_post_.push_back(s_next);

}


// void EskfEstimator::UpdateByLidarESKF(PointCloudXYZI &Dedistort_clouds_lidar) {
//     state_vector_post_.clear();
//     State s_next = state_;
//     int point_num = Dedistort_clouds_lidar.points.size();
//     Nearest_Points.resize(point_num);
//
//     Eigen::MatrixXd H_k;
//     Eigen::MatrixXd Z_k;
//     calculate(s_next, Dedistort_clouds_lidar, Z_k, H_k); // 批量计算 H 和 Z
//
//     Eigen::Matrix<double, static_cast<int>(StateIndex::STATE_TOTAL), static_cast<int>(StateIndex::STATE_TOTAL)> I_KH;
//     Eigen::Matrix<double, 1, 1> lidar_noise {0.001};
//
//     for (int i = 0; i < Z_k.rows(); ++i) {
//         // 每个点的观测残差 z_i
//         Eigen::Matrix<double, 1, 1> z_i = Z_k.block<1,1>(i,0);
//
//         // 每个点的观测雅可比 H_i（1 × STATE_TOTAL）
//         Eigen::Matrix<double, 1, static_cast<int>(StateIndex::STATE_TOTAL)> H_i = H_k.block<1,static_cast<int>(StateIndex::STATE_TOTAL)>(i,0);
//
//         // 卡尔曼增益 K_i：STATE_TOTAL × 1
//         Eigen::Matrix<double, static_cast<int>(StateIndex::STATE_TOTAL), 1> K_i =
//             s_next.P * H_i.transpose() * (H_i * s_next.P * H_i.transpose() + lidar_noise).inverse();
//
//         // 误差状态更新量
//         Eigen::Matrix<double, static_cast<int>(StateIndex::STATE_TOTAL), 1> delta_x = -K_i * z_i(0,0);
//
//         // 状态注入
//         s_next.q = s_next.q.toRotationMatrix() * so3Exp(delta_x.block<3,1>(StateIndex::R, 0));
//         s_next.q.normalize();
//         s_next.p  = s_next.p  + delta_x.block<3,1>(StateIndex::P,0);
//         s_next.v  = s_next.v  + delta_x.block<3,1>(StateIndex::V,0);
//         s_next.bw = s_next.bw + delta_x.block<3,1>(StateIndex::BW,0);
//         s_next.ba = s_next.ba + delta_x.block<3,1>(StateIndex::BA,0);
//         s_next.g  = s_next.g  + delta_x.block<3,1>(StateIndex::G,0);
//
//         // 协方差更新
//         I_KH = Eigen::Matrix<double, static_cast<int>(StateIndex::STATE_TOTAL), static_cast<int>(StateIndex::STATE_TOTAL)>::Identity() - K_i * H_i;
//         s_next.P = I_KH * s_next.P;
//     }
//
//     state_ = s_next;
//     state_vector_post_.push_back(s_next);
// }


/**
 * @brief 利用lidar数据进行更新，寻找最近邻点、匹配对应点、迭代 kalman 更新当前状态 state_，迭代过程存储在s_next_v中
 * @param Dedistort_clouds_lidar 去畸变点云，lidar坐标系
 */
void EskfEstimator::UpdateByLidarIESKF(PointCloudXYZI &Dedistort_clouds_lidar) {
    state_vector_post_.clear();
    State s_next = state_;
    auto finish_without_lidar_update = [&](const char *reason, const Eigen::MatrixXd &covariance) {
        state_ = s_next;
        state_.P = covariance;
        state_vector_post_.push_back(state_);
        if (down_effect_cloud_lidar) {
            down_effect_cloud_lidar->clear();
        }
        cout << "[UpdateByLidarIESKF] skip lidar update: " << reason << endl;
    };

    const int iter_num = 3;
    const double delta = 1E-5;
    int point_num = Dedistort_clouds_lidar.points.size();
    Nearest_Points.resize(point_num);
    size_t finite_point_num = 0;
    for (const auto &point : Dedistort_clouds_lidar.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            finite_point_num++;
        }
    }
    if (finite_point_num < 10) {
        finish_without_lidar_update("too few finite lidar points", state_.P);
        return;
    }
    Eigen::MatrixXd K;
    Eigen::MatrixXd H_k;
    Eigen::MatrixXd Z_k;
    bool converge;
    int i;
    Eigen::MatrixXd J_k_inv = Eigen::Matrix<double,StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL>::Identity();
    bool kd_research_en = true;
    for (i = 0; i< iter_num; i++) {
        /******* 计算J_k和更新P ********/
        // 计算迭代后的状态与初始状态差值，为计算J_k做铺垫
        Eigen::Matrix<double,static_cast<int>(StateIndex::STATE_TOTAL),1> error_s = getErrorState18(s_next,state_);
        // 计算J_k
        J_k_inv.block<3,3>(0,0) = A_T(error_s.block<3,1>(0,0));
        // cout << "error_s: " << error_s.norm() << endl;
        // 更新P
        Eigen::MatrixXd P_k = J_k_inv * s_next.P * J_k_inv.transpose();
        /******* 计算残差z_k和雅可比H_k ********/
        Eigen::MatrixXd R_inv;
        // 调用函数计算 Z 和 H ，其中 Z_k 是一个列向量
        if (!calculate(s_next,Dedistort_clouds_lidar,Z_k,H_k,kd_research_en) ||
            H_k.rows() == 0 || Z_k.rows() != H_k.rows()) {
            finish_without_lidar_update("no valid lidar constraints", P_k);
            return;
        }
        // 输出残差
        Eigen::MatrixXd H_kt = H_k.transpose();
        // R为 0.001E
        K = (H_kt * H_k + P_k.inverse() * 0.001 ).inverse() * H_kt;
        // 计算 X 的增量
        Eigen::MatrixXd left = -1 * K * Z_k;
        Eigen::MatrixXd right = -1 * (Eigen::Matrix<double,StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL>::Identity() - K * H_k) * J_k_inv * error_s;
        Eigen::MatrixXd update_x = left + right;
        if (!K.allFinite() || !update_x.allFinite()) {
            finish_without_lidar_update("non-finite lidar update", P_k);
            return;
        }
        // 收敛判断
        converge = true;

        kd_research_en = true;
        if (update_x.maxCoeff() < 2E-4 || i == iter_num - 2) {
            kd_research_en = true;
        }

        // update_x 是一个 18*1 的列向量，使用 maxCoeff() 函数来获取最大值
        if (update_x.maxCoeff() > delta)
        {
            // cout << "[" << i << "]" << "update_x: " << update_x.maxCoeff() << endl;
            converge = false;
        }
        // 更新X
        s_next.q = s_next.q.toRotationMatrix() * so3Exp(update_x.block<3,1>(StateIndex::R,0));
        s_next.q.normalize();
        s_next.p  = s_next.p  + update_x.block<3,1>(StateIndex::P,0);
        s_next.v  = s_next.v  + update_x.block<3,1>(StateIndex::V,0);
        s_next.bw = s_next.bw + update_x.block<3,1>(StateIndex::BW,0);
        s_next.ba = s_next.ba + update_x.block<3,1>(StateIndex::BA,0);
        s_next.g  = s_next.g  + update_x.block<3,1>(StateIndex::G,0);
        // s_next_v.push_back(s_next);
        if(converge){
            break;
        }
    }
    // 输出迭代次数
    // cout << "iter_num: " << i << endl;
    //注释雷达更新
    state_ = s_next;
    state_vector_post_.push_back(s_next);
    //注释雷达更新
    state_.P = ( Eigen::Matrix<double,StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL>::Identity() - K * H_k ) * J_k_inv * state_.P * J_k_inv.transpose();
}

void EskfEstimator::UpdateByWheel(std::deque<WheelData> buf) {
    const double L1 = 0.25;
    const double L2 = 0.35;
    // 找到 LIO 输出时刻 last_lidar_end_time_ 对应的轮速计的值
    // cout << "buf.size(): " << buf.size() << endl;
    int i = 0;
    //while (buf[i].timestamp < last_lidar_end_time_ && i < buf.size()) {
    while (buf[i].timestamp < last_lidar_end_time_ && static_cast<size_t>(i) < buf.size()) {
        i++;
    }
    // cout << "i: "<< i << endl;
    // 使用最近邻数据
    WheelData wheel_data_1 = buf[i-1];
    WheelData wheel_data_2 = buf[i];
    double dt_1 = abs(wheel_data_1.timestamp - last_lidar_end_time_);
    double dt_2 = abs(wheel_data_2.timestamp - last_lidar_end_time_);
    WheelData wheel_data = dt_1 < dt_2 ? wheel_data_1:wheel_data_2;
    // 计算观测速度
    double vx_1 = wheel_data.v_L * cos(wheel_data.angle_L) + L2/L1 * wheel_data.v_L * sin(wheel_data.angle_L);
    double vx_2 = wheel_data.v_R * cos(wheel_data.angle_R) - L2/L1 * wheel_data.v_R * sin(wheel_data.angle_R);
    double vx = (vx_1+vx_2)/2;
    // 计算角速度
    double dtheta_1 = -2 * wheel_data.v_L * sin(wheel_data.angle_L) / L2;
    double dtheta_2 = -2 * wheel_data.v_R * sin(wheel_data.angle_R) / L2;
    double dtheta  = (dtheta_1+dtheta_2)/2;
    double L = 0.16;
    double vy = -dtheta * L;

    // 从本体坐标系转到Lidar系
    Eigen::Quaterniond q_B_L (0.70711,0,0,0.70711); // 绕 z 轴顺时针旋转90度

    // cout << "vx " << vx << "vy " << vy << "dtheta " << dtheta << endl;

    // 使用 wheel_data 计算更新状态
    Eigen::MatrixXd mat_C(3,18);
    mat_C.setZero();
    // 创建临时使用的矩阵和向量
    Eigen::Matrix3d submat;
    Eigen::Vector3d temp_vec;
    Eigen::Quaterniond q_L_W = state_.q;
    Eigen::Quaterniond q_B_W = q_B_L * q_L_W;
    // 第一子块
    temp_vec = q_B_W.conjugate() * state_.v ;
    submat = skew3d(temp_vec);
    mat_C.block<3,3>(0,0) = submat;
    // 第三子块
    submat = q_B_W.conjugate().toRotationMatrix();
    mat_C.block<3,3>(0,6) = submat;
    /******这里是W矩阵******/
    Eigen::MatrixXd mat_W = Eigen::MatrixXd::Identity(3, 3);
    // 计算增益
    Eigen::MatrixXd K(18,3);
    K.setZero();
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * 1E-5;
    K = state_.P * mat_C.transpose() * (mat_C * state_.P * mat_C.transpose() +
        mat_W * R * mat_W.transpose()).inverse();
    Eigen::VectorXd delta_dstate (18);
    // 计算残差
    Eigen::Vector3d res_wheel =  Eigen::Vector3d(vx,vy,0) - q_B_W.conjugate() * state_.v;
    delta_dstate = K * res_wheel;

    // cout << "K" << K << endl;
    // cout << "res_wheel" << res_wheel << endl;

    Eigen::Quaterniond dq (1,
        0.5*delta_dstate(0),
        0.5*delta_dstate(1),
        0.5*delta_dstate(2));
    state_.q = (state_.q * dq).normalized();
    state_.p += delta_dstate.segment<3>(3);
    state_.v += delta_dstate.segment<3>(6);
    state_.bw += delta_dstate.segment<3>(9);
    state_.ba += delta_dstate.segment<3>(12);
    state_.g += delta_dstate.segment<3>(15);
    // 更新协方差
    state_.P = state_.P - K * mat_C * state_.P;

    // 将z保存到文件中
    if (res_wheel_log_enable & log_save_enable) {
        ofstream outFile;
        static bool frist = true;
        // 第一次运行，清空文件并添加首行
        if (frist) {
            outFile.open(string(PACKAGE_ROOT_DIR) + "/res_wheel.txt", std::ios::out);
            outFile << "res_x res_y res_z" << endl;
            outFile.close();
            frist = false;
        }
        outFile.open(string(PACKAGE_ROOT_DIR)+"/res_wheel.txt",ios::app);
        outFile << res_wheel(0) << " " << res_wheel(1) << " " << res_wheel(2)<< endl;
        outFile.close();
    }
    for (int j =0 ;j < i ; j++) {
        buf.pop_front();
    }
}


Eigen::Matrix<double,18,1> EskfEstimator::getErrorState18(const State &s1, const  State &s2){
    Eigen::Matrix<double,18,1> es;
    es.setZero();
    es.block<3,1>(0,0)  = SO3Log(s2.q.toRotationMatrix().transpose() * s1.q.toRotationMatrix());
    es.block<3,1>(3,0)  = s1.p  - s2.p;
    es.block<3,1>(6,0)  = s1.v  - s2.v;
    es.block<3,1>(9,0)  = s1.bw - s2.bw;
    es.block<3,1>(12,0) = s1.ba - s2.ba;
    es.block<3,1>(15,0) = s1.g  - s2.g;
    return es;
}

template<typename _first, typename _second, typename _thrid>
struct triple{
    _first first;
    _second second;
    _thrid thrid;
};

struct normvec {
    Eigen::Vector3d norm_vec; // 平面的法向量
    Eigen::Vector3d point_imu_xyz; // imu坐标系下点的坐标
    double d; // 点到平面的距离
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// 并行计算需要事先分配好内存（提前调用 resize）
std::vector<normvec, Eigen::aligned_allocator<normvec>> Norm_Vector;
std::vector<bool> is_effect_point;

/**
 * @brief 利用点进行平面拟合，输出为ax+by+cz+d=0，d=-1/norm，注意法向量的方向
 * @param points 用于平面拟合的点
 * @param pabcd pabcd(0) = a（法向量的 x 分量）pabcd(1) = b（法向量的 y 分量）pabcd(2) = c（法向量的 z 分量），均归一化\n
 *              pabcd(3) = -1/normal，其中 normal 是法向量的模长，用以将法向量单位化。
 * @param threhold 平面拟合的阈值
 * @return bool量，是否是平面
 */
static bool planarFittingAndCheck(const PointVector & points, Eigen::Vector4d &pabcd, float threhold) {
    Eigen::Vector3d normal_vector;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    int point_num = points.size();
    A.resize(point_num,3);
    b.resize(point_num);
    b.setOnes();
    for (int i = 0; i < point_num; i++) {
        A(i,0) = points[i].x;
        A(i,1) = points[i].y;
        A(i,2) = points[i].z;
    }
    /**
     * 解超定方程 Ax = b 【其中A的每一行是一个点，x是待拟合平面的单位法向量，b是全为1的列向量】
     * 注意，此时的 normal_vector 并不是单位向量，还需要归一化
     * 此时解的方程相当于 ax + by + cz = 1
     */
    normal_vector = A.colPivHouseholderQr().solve(b);

    for (int j = 0; j < point_num; j++) {
        // 点到面的距离不能大于阈值，如果大于阈值，则不是平面
        if (fabs(normal_vector(0) * points[j].x + normal_vector(1) * points[j].y + normal_vector(2) * points[j].z - 1.0f) > threhold)
        {
            return false;
        }
    }
    double normal = normal_vector.norm();
    normal_vector.normalize();
    pabcd(0) = normal_vector(0);
    pabcd(1) = normal_vector(1);
    pabcd(2) = normal_vector(2);
    pabcd(3) = -1/normal;

    return true;
}

/**
 * @brief 将点从Lidar坐标系转化到世界坐标系
 * @param point Lidar坐标系中的点
 * @return point_world 世界坐标系中的点
 */
PointType EskfEstimator::transLidar2World(const PointType &point) {
    Eigen::Quaterniond q = state_.q;
    Eigen::Vector3d p = state_.p;
    // Lidar -> IMU -> World
    PointType point_world = transformPoint(transformPoint(point,imu_R_lidar,imu_t_lidar),q,p);
    return point_world;
}

/**
 * @brief 将点从Lidar坐标系转化到世界坐标系 【 注意，这里是手动实现，只附带了 curvature 和 intensity 信息】
 * @param points_lidar Lidar坐标系下的点云
 * @return points_world 世界坐标系下的点云
 */

PointCloudXYZIPtr EskfEstimator::transLidar2World(const PointCloudXYZI &points_lidar) {
    auto start = std::chrono::high_resolution_clock::now();
    /********** ！注意是先平移后旋转！ **********/
    // 先应用从 Lidar 系到 IMU 系的变换   imu_T_lidar
    Eigen::Matrix4d imu_T_lidar = Eigen::Matrix4d::Identity();
    imu_T_lidar.block<3, 3>(0, 0) = imu_R_lidar;
    imu_T_lidar.block<3, 1>(0, 3) = imu_t_lidar;
    // 再应用从 IMU 系到 World 系的变换  world_T_imu * imu_T_lidar
    Eigen::Matrix4d world_T_imu = Eigen::Matrix4d::Identity();
    world_T_imu.block<3, 3>(0, 0) = state_.q.toRotationMatrix();
    world_T_imu.block<3, 1>(0, 3) = state_.p;
    // 应用总的变换矩阵
    Eigen::Matrix4d world_T_lidar = world_T_imu * imu_T_lidar;
    PointCloudXYZIPtr points_world (new PointCloudXYZI);
    // 使用 Eigen 库 + 并行化 比使用 pcl::transformPointCloud 快一些
    points_world->points.resize(points_lidar.size());
    for (size_t i = 0; i < points_lidar.size(); ++i) {
        const auto &pt = points_lidar.points[i];
        Eigen::Vector4d p(pt.x, pt.y, pt.z, 1.0);
        Eigen::Vector4d pw = world_T_lidar * p;
        auto &out_pt = points_world->points[i];
        out_pt.x = pw.x();
        out_pt.y = pw.y();
        out_pt.z = pw.z();
        out_pt.curvature = pt.curvature;
        out_pt.intensity = pt.intensity;
    }
    // 手动设置元信息
    points_world->width = points_world->points.size();  // 设置 width
    points_world->height = 1;                            // 点云是无组织的（非图像结构）
    points_world->is_dense = true;                       // 没有 NaN 或无效点
    // 结束时间
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> diff_ms = end - start;
    // 将时间（单位 ms）和 points_lidar 的点云数量写入文件中
    // if (log_save_enable) {
    //     ofstream outFile;
    //     outFile.open(string(PACKAGE_ROOT_DIR) + "/trans_time_log.txt", std::ios::app);
    //     outFile << diff_ms.count() << " " << points_lidar.size() << endl;
    //     outFile.close();
    // }
    return points_world;
}
//
// PointCloudXYZIPtr EskfEstimator::transLidar2World(const PointCloudXYZI& points_lidar)
// {
//     /* —— 1. 组合齐次变换 —— */
//     Eigen::Matrix4d world_T_lidar = Eigen::Matrix4d::Identity();
//
//     // imu_T_lidar
//     world_T_lidar.block<3,3>(0,0) = state_.q.toRotationMatrix() * imu_R_lidar;
//     world_T_lidar.block<3,1>(0,3) = state_.q.toRotationMatrix() * imu_t_lidar + state_.p;
//
//     /* —— 2. 逐点变换 —— */
//     auto points_world = std::make_shared<PointCloudXYZI>();
//     points_world->points.resize(points_lidar.size());
//
//     for (size_t i = 0; i < points_lidar.size(); ++i)
//     {
//         const auto&  src = points_lidar.points[i];
//         auto&        dst = points_world->points[i];
//
//         Eigen::Vector4d p(src.x, src.y, src.z, 1.0);
//         Eigen::Vector4d pw = world_T_lidar * p;
//
//         dst.x = pw.x();
//         dst.y = pw.y();
//         dst.z = pw.z();
//         dst.intensity = src.intensity;
//         dst.curvature = src.curvature;
//     }
//
//     /* —— 3. 填充元信息 —— */
//     points_world->width     = static_cast<uint32_t>(points_world->points.size());
//     points_world->height    = 1;
//     points_world->is_dense  = true;
//
//     return points_world;
// }


// 残差和雅可比计算函数，ESKF版本，与IESKF对应
bool EskfEstimator::calculateEKF(const State&state, PointCloudXYZI &clouds_lidar, Eigen::MatrixXd & Z,Eigen::MatrixXd & H) {
    using loss_type = triple<Eigen::Vector3d,Eigen::Vector3d,double>;
    size_t point_num = clouds_lidar.size();
    if (point_num == 0) {
        H.resize(0, StateIndex::STATE_TOTAL);
        Z.resize(0, 1);
        if (down_effect_cloud_lidar) {
            down_effect_cloud_lidar->clear();
        }
        return false;
    }
    if (Nearest_Points.size() != point_num) {
        Nearest_Points.resize(point_num);
    }
    Norm_Vector.resize(point_num);
    std::vector<bool> is_effect_point(point_num,false);
    std::vector<loss_type> loss_v;
    int vaild_points_num = 0;
    int NEAR_POINTS_NUM = 5; // 搜索的最近邻的数量

/**** 为下一个for循环使能多线程并行计算 ****/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif

    /**
     * 有效点的判断及参数预处理
     * 1. 将当前点变换到世界系下
     * 2. 使用 ikd_tree 临近搜索 NEAR_POINTS_NUM 个点
     * 3. 判断这些点是否构成平面，如果构成平面，计算平面法向量和点到平面距离
     * 4. 判断点离这个平面够不够近(达到阈值)
     * 5. 满足上述条件，设置为有效点。
    */
    /**  注意！多线程计算时尽量避免对同一个变量进行操作，防止线程冲突  **/
    for (size_t  i = 0; i < point_num; i++)
    {
        const auto &raw_point = clouds_lidar.points[i];
        if (!std::isfinite(raw_point.x) || !std::isfinite(raw_point.y) || !std::isfinite(raw_point.z)) {
            continue;
        }
        // 1. 按照当前状态变换到世界系  Lidar系 -> IMU系 -> 世界系
        PointType point_imu = transformPoint(clouds_lidar.points[i], imu_R_lidar, imu_t_lidar);
        PointType point_world = transformPoint(point_imu,state.q,state.p);
        // 2. 最近邻搜索
        auto &points_near = Nearest_Points[i];
        std::vector<float> distance(NEAR_POINTS_NUM);
        ikdtree.Nearest_Search(point_world, NEAR_POINTS_NUM, points_near, distance);

        // 2.1 如果搜索不到足够的点，或者最近邻中最后一个点离当前点太远，就跳过
        //if (points_near.size() < NEAR_POINTS_NUM || distance[NEAR_POINTS_NUM-1]>5)
        if (points_near.size() < static_cast<size_t>(NEAR_POINTS_NUM) || distance[NEAR_POINTS_NUM-1]>5)
        {
            continue;
        }

        Eigen::Vector4d pabcd;

        // 3. 计算平面标准方程(ax + by + cz + d = 0)并判断是否有太过离群的点
        if (planarFittingAndCheck(points_near,pabcd,0.1))
        {
            // 4. 计算点到平面距离
            double pd = point_world.x * pabcd(0) + point_world.y * pabcd(1) + point_world.z * pabcd(2) + pabcd(3);
            // 记录残差
            if (isnan(pd)) {
                std::cerr << "[calculate] pd is nan!" << std::endl;
                continue;
            }
            /** s决定了有效性，距离平面较近的点才有效，同时随着距离原点距离变远，该阈值会变大，意味着远的点可以偏移平面更远一点
             * fabs(pd) 点到平面的距离
             * p_lidar.norm() 点到原点的距离
             */
            Eigen::Vector3d p_lidar = clouds_lidar.points[i].getVector3fMap().cast<double>();
            // 5. 如果距离平面较近，就认为是有效点
            double threshold = 0.07 + 0.02 * p_lidar.norm();  // 允许误差和距离线性增长
            if (fabs(pd) < threshold){
                // 在多线程中使用push_back会报错： double free or corruption (out)
                // 所以先存储结果，之后再计算有效点个数，剔除无效点
                Norm_Vector[i].norm_vec = pabcd.block<3,1>(0,0); // 平面法向量 用于求H
                Norm_Vector[i].point_imu_xyz = Eigen::Vector3d (point_imu.x,point_imu.y,point_imu.z); // imu系下点的坐标，用于求H
                Norm_Vector[i].d = pd; // 点到平面的距离
                is_effect_point[i] = true;
            }
        }
    }

    // 计算有效点的数量
    for (size_t i = 0; i < point_num; i++) {
        if (is_effect_point[i]) {
            vaild_points_num ++;
        }
    }

    // 输出有效点和实际点的数量
    // std::cout << "[calculate] Effective Points: " << vaild_points_num << " / " << point_num << std::endl;

    // 如果没有有效点，返回false
    if (vaild_points_num < 1)
    {
        std::cerr << "[calculate] No Effective Points!" << std::endl;
        H.resize(0, StateIndex::STATE_TOTAL);
        Z.resize(0, 1);
        if (down_effect_cloud_lidar) {
            down_effect_cloud_lidar->clear();
        }
        return false;
    }

    // 根据有效点的数量分配 H Z的大小
    H = Eigen::MatrixXd::Zero(vaild_points_num, 18);
    Z.resize(vaild_points_num,1);

    // 调整全局变量 down_effect_cloud_lidar 的大小
    down_effect_cloud_lidar->clear();
    down_effect_cloud_lidar->reserve(vaild_points_num);

    // 计算完成后根据有效点 is_effect_point 数组重新存储
    int effect_i = 0;
    for (size_t vi = 0; vi < point_num; vi++)
    {
        if (is_effect_point[vi]) {
            // 将有效点存储到新的点云中 down_effect_cloud_lidar
            down_effect_cloud_lidar->points.push_back(clouds_lidar.points[vi]);
            /*** 计算导数 H 和残差 Z ***/
            Eigen::Vector3d C(state.q.conjugate() * Norm_Vector[vi].norm_vec);
            Eigen::Vector3d A(skew3d(Norm_Vector[vi].point_imu_xyz) * C);
            Eigen::Vector3d point_lidar_xyz = clouds_lidar.points[vi].getVector3fMap().cast<double>();
            Eigen::Vector3d B(skew3d(point_lidar_xyz) * imu_R_lidar.conjugate() * C);
            // H 记录雅可比
            Eigen::Vector3d dr = - Norm_Vector[vi].norm_vec.transpose() * state.q.toRotationMatrix() * skew3d(Norm_Vector[vi].point_imu_xyz);
            H.block<1,3>(effect_i,0) = dr.transpose();
            H.block<1,3>(effect_i,3) = Norm_Vector[vi].norm_vec.transpose();
            // H.block<1,3>(effect_i,3) = Norm_Vector[vi].norm_vec.transpose();
            // H.block<1,3>(effect_i,0) = A.transpose();
            /**** 这里的外参是IMU-Lidar下的外参，如果想要估计该外参，需要把外参也加入到状态变量中，现在实际上还没有这个功能 ****/
            bool extrinsic_est_en = false;
            if (extrinsic_est_en) {
                H.block<1,3>(effect_i,6) = B.transpose();
                H.block<1,3>(effect_i,9) = C.transpose();
            }
            else {
                H.block<1,6>(effect_i,6) = Eigen::VectorXd::Zero(6).transpose();
            }
            // Z 记录距离
            Z(effect_i,0) = Norm_Vector[vi].d;
            effect_i ++;
        }
    }

    // 最后 effect_i 和 vaild_points_num 一定相等
    if (effect_i != vaild_points_num) cerr << "[calculate] effect_i != vaild_points_num" << std::endl;
    return true;
}

/**
 * @brief 根据当前状态计算点云与最近点构成平面间残差（忽略异常点），并给出雅可比矩阵用于迭代kalman滤波（类似梯度下降）
 * @param state 当前迭代状态
 * @param clouds_lidar Lidar系下当前帧点云
 * @param Z 残差
 * @param H 雅可比矩阵
 * @return bool量，表示是否有有效点
 */
bool EskfEstimator::calculate(const State&state, PointCloudXYZI &clouds_lidar, Eigen::MatrixXd & Z,Eigen::MatrixXd & H, bool kd_research_en) {
    size_t point_num = clouds_lidar.size();
    if (point_num == 0) {
        H.resize(0, StateIndex::STATE_TOTAL);
        Z.resize(0, 1);
        if (down_effect_cloud_lidar) {
            down_effect_cloud_lidar->clear();
        }
        return false;
    }
    // kd_research_en = true;
    Norm_Vector.resize(point_num);
    is_effect_point.resize(point_num);
    if (kd_research_en)
        std::fill(is_effect_point.begin(), is_effect_point.end(), false);
    int vaild_points_num = 0;
    int NEAR_POINTS_NUM = 5; // 搜索的最近邻的数量


/**** 为下一个for循环使能多线程并行计算 ****/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
#endif

    /**
     * 有效点的判断及参数预处理
     * 1. 将当前点变换到世界系下
     * 2. 使用 ikd_tree 临近搜索 NEAR_POINTS_NUM 个点
     * 3. 判断这些点是否构成平面，如果构成平面，计算平面法向量和点到平面距离
     * 4. 判断点离这个平面够不够近(达到阈值)
     * 5. 满足上述条件，设置为有效点。
    */
    /**  注意！多线程计算时尽量避免对同一个变量进行操作，防止线程冲突  **/
    for (size_t  i = 0; i < point_num; i++)
    {
        const auto &raw_point = clouds_lidar.points[i];
        if (!std::isfinite(raw_point.x) || !std::isfinite(raw_point.y) || !std::isfinite(raw_point.z)) {
            is_effect_point[i] = false;
            continue;
        }
        // 1. 按照当前状态变换到世界系  Lidar系 -> IMU系 -> 世界系
        PointType point_imu = transformPoint(clouds_lidar.points[i], imu_R_lidar, imu_t_lidar);
        PointType point_world = transformPoint(point_imu,state.q,state.p);
        // 2. 最近邻搜索
        if (kd_research_en) {
            auto &points_near = Nearest_Points[i];
            std::vector<float> distance(NEAR_POINTS_NUM, 0.0f);

            ikdtree.Nearest_Search(point_world, NEAR_POINTS_NUM, points_near, distance);

            // 2.1 如果搜索不到足够的点，或者最近邻中最后一个点离当前点太远，就跳过
            //if (points_near.size() < NEAR_POINTS_NUM || distance[NEAR_POINTS_NUM-1]>5)
            if (points_near.size() < static_cast<size_t>(NEAR_POINTS_NUM) || distance[NEAR_POINTS_NUM-1]>5)
            {
                continue;
            }

            Eigen::Vector4d pabcd;

            // 3. 计算平面标准方程(ax + by + cz + d = 0)并判断是否有太过离群的点
            if (planarFittingAndCheck(points_near,pabcd,0.1))
            {
                // 4. 计算点到平面距离
                double pd = point_world.x * pabcd(0) + point_world.y * pabcd(1) + point_world.z * pabcd(2) + pabcd(3);
                // 记录残差
                if (isnan(pd)) {
                    std::cerr << "[calculate] pd is nan!" << std::endl;
                    continue;
                }
                /** s决定了有效性，距离平面较近的点才有效，同时随着距离原点距离变远，该阈值会变大，意味着远的点可以偏移平面更远一点
                 * fabs(pd) 点到平面的距离
                 * p_lidar.norm() 点到原点的距离
                 */
                Eigen::Vector3d p_lidar = clouds_lidar.points[i].getVector3fMap().cast<double>();
                // double s = 1 - 0.9 * fabs(pd) / sqrt(p_lidar.norm());
                // 5. 如果距离平面较近，就认为是有效点
                // if(s > 0.9 ) {
                double threshold = 0.07 + 0.02 * p_lidar.norm();  // 允许误差和距离线性增长
                if (fabs(pd) < threshold){
                    // 在多线程中使用push_back会报错： double free or corruption (out)
                    // 所以先存储结果，之后再计算有效点个数，剔除无效点
                    Norm_Vector[i].norm_vec = pabcd.block<3,1>(0,0); // 平面法向量 用于求H
                    Norm_Vector[i].point_imu_xyz = Eigen::Vector3d (point_imu.x,point_imu.y,point_imu.z); // imu系下点的坐标，用于求H
                    Norm_Vector[i].d = pd; // 点到平面的距离
                    is_effect_point[i] = true;
                }
            }
        }
    }

    // 计算有效点的数量
    for (size_t i = 0; i < point_num; i++) {
        if (is_effect_point[i]) {
            vaild_points_num ++;
        }
    }

    // 输出有效点和实际点的数量
    // std::cout << "[calculate] Effective Points: " << vaild_points_num << " / " << point_num << std::endl;

    // 如果没有有效点，返回false
    if (vaild_points_num < 1)
    {
        std::cerr << "[calculate] No Effective Points!" << std::endl;
        H.resize(0, StateIndex::STATE_TOTAL);
        Z.resize(0, 1);
        if (down_effect_cloud_lidar) {
            down_effect_cloud_lidar->clear();
        }
        return false;
    }

    // 根据有效点的数量分配 H Z的大小
    H = Eigen::MatrixXd::Zero(vaild_points_num, 18);
    Z.resize(vaild_points_num,1);

    // 调整全局变量 down_effect_cloud_lidar 的大小
    down_effect_cloud_lidar->clear();
    down_effect_cloud_lidar->reserve(vaild_points_num);

    // 计算完成后根据有效点 is_effect_point 数组重新存储
    int effect_i = 0;
    for (size_t vi = 0; vi < point_num; vi++)
    {
        if (is_effect_point[vi]) {
            // 将有效点存储到新的点云中 down_effect_cloud_lidar
            down_effect_cloud_lidar->points.push_back(clouds_lidar.points[vi]);
            /*** 计算导数 H 和残差 Z ***/
            Eigen::Vector3d C(state.q.conjugate() * Norm_Vector[vi].norm_vec);
            Eigen::Vector3d A(skew3d(Norm_Vector[vi].point_imu_xyz) * C);
            Eigen::Vector3d point_lidar_xyz = clouds_lidar.points[vi].getVector3fMap().cast<double>();
            Eigen::Vector3d B(skew3d(point_lidar_xyz) * imu_R_lidar.conjugate() * C);
            // H 记录雅可比
            Eigen::Vector3d dr = - Norm_Vector[vi].norm_vec.transpose() * state.q.toRotationMatrix() * skew3d(Norm_Vector[vi].point_imu_xyz);
            H.block<1,3>(effect_i,0) = dr.transpose();
            H.block<1,3>(effect_i,3) = Norm_Vector[vi].norm_vec.transpose();
            // H.block<1,3>(effect_i,3) = Norm_Vector[vi].norm_vec.transpose();
            // H.block<1,3>(effect_i,0) = A.transpose();
            /**** 这里的外参是IMU-Lidar下的外参，如果想要估计该外参，需要把外参也加入到状态变量中，现在实际上还没有这个功能 ****/
            bool extrinsic_est_en = false;
            if (extrinsic_est_en) {
                H.block<1,3>(effect_i,6) = B.transpose();
                H.block<1,3>(effect_i,9) = C.transpose();
            }
            else {
                H.block<1,6>(effect_i,6) = Eigen::VectorXd::Zero(6).transpose();
            }
            // Z 记录距离
            Z(effect_i,0) = Norm_Vector[vi].d;
            effect_i ++;
        }
    }

    // 最后 effect_i 和 vaild_points_num 一定相等
    if (effect_i != vaild_points_num) cerr << "[calculate] effect_i != vaild_points_num" << std::endl;
    return true;
}
void undistortPointCloudOld(
    std::vector<Pose>& IMU_Pose_vector_,
    PointCloudXYZI & cloud,
    const State& imu_back_state)
{
    Eigen::Vector3d imu_back_pos = imu_back_state.p;
    Eigen::Quaterniond q_imu;
    Eigen::Vector3d vel_imu, pos_imu, acc_imu, angvel_imu;
    auto it_pcl = cloud.points.end() - 1;
    for (auto it_kp = IMU_Pose_vector_.end() - 1; it_kp != IMU_Pose_vector_.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        q_imu = head->q;
        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
        // IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
        vel_imu = head->v;
        pos_imu = head->p;
        acc_imu = tail->acc; // 已经去除了重力矢量
        angvel_imu = tail->gyr;
        // 对两帧imu之间的点去畸变
        for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
        {
            // std::cout << "offset_time" << head->offset_time << std::endl;
            double dt = it_pcl->curvature / double(1000) - head->offset_time;
            // std::cout << "dt: " << dt << std::endl;
            // W: 世界坐标下，初始时刻的IMU坐标系
            // L: lidar 坐标系，当前处理的点时刻
            // I: IMU 坐标系，当前处理的点时刻
            // IE: IMU Endpoint 坐标系，当前帧最后时刻对应的IMU坐标系
            // LE: lidar Endpoint 坐标系，当前帧最后时刻对应的激光雷达坐标系
            Eigen::Vector3d P_L(it_pcl->x, it_pcl->y, it_pcl->z); // 激光雷达坐标系下点的位置（观测值）
            Eigen::Vector3d P_I = imu_R_lidar * P_L + imu_t_lidar; // IMU 坐标系下点的位置
            Eigen::Matrix3d R_I_W(q_imu * Exp(angvel_imu, dt)); // IMU 当前帧相对世界坐标系的旋转矩阵（对原始矩阵做一些小量更新）
            Eigen::Vector3d T_I_W(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt); // IMU 当前帧相对世界坐标系的位置（对原始位置的二阶预测近似）
            Eigen::Vector3d P_W = R_I_W * P_I + T_I_W; // 点在世界坐标系下的位置
            // 推导至 lidar Endpoint 坐标系
            Eigen::Vector3d P_IE = imu_back_state.q.inverse() * (P_W - imu_back_pos); // 点在 IMU Endpoint坐标系下的位置
            Eigen::Vector3d P_LE = imu_R_lidar.transpose() * (P_IE - imu_t_lidar); // 点在 lidar Endpoint坐标系下的位置
            // save Undistorted points and their rotation
            it_pcl->x = P_LE(0);
            it_pcl->y = P_LE(1);
            it_pcl->z = P_LE(2);
            // 当点云被遍历完后跳出循环
            if (it_pcl == cloud.points.begin()) {
                cout << "【undistortPointCloudOld】 it_pcl == cloud.points.begin()" << std::endl;
                break;
            }
        }
    }
    int cnt1 = 0;
    for (; it_pcl != cloud.points.begin(); it_pcl --) {
        cnt1 ++;
    }
    cout << "【undistortPointCloudOld】 cnt1: " << cnt1 << std::endl;
}


void undistortPointCloudNew(
    std::vector<Pose>& IMU_Pose_vector_,
    PointCloudXYZI & cloud,
    const State& imu_back_state)
{
    auto it_pcl = cloud.points.begin();
    Eigen::Vector3d imu_back_pos = imu_back_state.p;
    Eigen::Quaterniond q_imu;
    Eigen::Vector3d vel_imu, pos_imu, acc_imu, angvel_imu;

    for (auto it_kp = IMU_Pose_vector_.begin(); it_kp != IMU_Pose_vector_.end() - 1 ; it_kp++)
    {
        auto head = it_kp;
        auto tail = it_kp + 1;
        q_imu = head->q;
        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
        // IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
        vel_imu = head->v;
        pos_imu = head->p;
        acc_imu = tail->acc; // 已经去除了重力矢量
        angvel_imu = tail->gyr;
        // 对两帧imu之间的点去畸变
        for(; it_pcl->curvature / double(1000) <= tail->offset_time; it_pcl ++)
        {
            // std::cout << "offset_time" << head->offset_time << std::endl;
            double dt = it_pcl->curvature / double(1000) - head->offset_time;
            // if (dt < 0) {
            //     std::cout << "[undistortPointCloudNew] dt < 0 : " << dt << std::endl;
            // }
            // W: 世界坐标下，初始时刻的IMU坐标系
            // L: lidar 坐标系，当前处理的点时刻
            // I: IMU 坐标系，当前处理的点时刻
            // IE: IMU Endpoint 坐标系，当前帧最后时刻对应的IMU坐标系
            // LE: lidar Endpoint 坐标系，当前帧最后时刻对应的激光雷达坐标系
            Eigen::Vector3d P_L(it_pcl->x, it_pcl->y, it_pcl->z); // 激光雷达坐标系下点的位置（观测值）
            Eigen::Vector3d P_I = imu_R_lidar * P_L + imu_t_lidar; // IMU 坐标系下点的位置
            Eigen::Matrix3d R_I_W(q_imu * Exp(angvel_imu, dt)); // IMU 当前帧相对世界坐标系的旋转矩阵（对原始矩阵做一些小量更新）
            Eigen::Vector3d T_I_W(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt); // IMU 当前帧相对世界坐标系的位置（对原始位置的二阶预测近似）
            Eigen::Vector3d P_W = R_I_W * P_I + T_I_W; // 点在世界坐标系下的位置
            // 推导至 lidar Endpoint 坐标系
            Eigen::Vector3d P_IE = imu_back_state.q.inverse() * (P_W - imu_back_pos); // 点在 IMU Endpoint坐标系下的位置
            Eigen::Vector3d P_LE = imu_R_lidar.transpose() * (P_IE - imu_t_lidar); // 点在 lidar Endpoint坐标系下的位置
            // 保存去畸变点云
            it_pcl->x = P_LE(0);
            it_pcl->y = P_LE(1);
            it_pcl->z = P_LE(2);
            // 当点云被遍历完后跳出循环
            if (it_pcl == cloud.points.end()-1) break;
        }
    }
    int cnt1 = 0;
    auto head = IMU_Pose_vector_.end() - 1;
    for (; it_pcl != cloud.points.end()-1; it_pcl ++){
        cnt1 ++;
        // double dt = it_pcl->curvature / double(1000) - head->offset_time;
        // Eigen::Vector3d P_L(it_pcl->x, it_pcl->y, it_pcl->z); // 激光雷达坐标系下点的位置（观测值）
        // Eigen::Vector3d P_I = imu_R_lidar * P_L + imu_t_lidar; // IMU 坐标系下点的位置
        // Eigen::Matrix3d R_I_W(q_imu * Exp(angvel_imu, dt)); // IMU 当前帧相对世界坐标系的旋转矩阵（对原始矩阵做一些小量更新）
        // Eigen::Vector3d T_I_W(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt); // IMU 当前帧相对世界坐标系的位置（对原始位置的二阶预测近似）
        // Eigen::Vector3d P_W = R_I_W * P_I + T_I_W; // 点在世界坐标系下的位置
        // // 推导至 lidar Endpoint 坐标系
        // Eigen::Vector3d P_IE = imu_back_state.q.inverse() * (P_W - imu_back_pos); // 点在 IMU Endpoint坐标系下的位置
        // Eigen::Vector3d P_LE = imu_R_lidar.transpose() * (P_IE - imu_t_lidar); // 点在 lidar Endpoint坐标系下的位置
        // // 保存去畸变点云
        // it_pcl->x = P_LE(0);
        // it_pcl->y = P_LE(1);
        // it_pcl->z = P_LE(2);
        // // 当点云被遍历完后跳出循环
        // if (it_pcl == cloud.points.end()-1) break;
    }
    cout << "[undistortPointCloudNew] cnt1: " << cnt1 << std::endl;
}

bool comparePointClouds(const PointCloudXYZI& c1, const PointCloudXYZI& c2, double eps = 1e-6)
{
    if (c1.size() != c2.size()) return false;
    for (size_t i = 100; i < c1.size() - 100; ++i)
    {
        if ((std::abs(c1[i].x - c2[i].x) > eps) ||
            (std::abs(c1[i].y - c2[i].y) > eps) ||
            (std::abs(c1[i].z - c2[i].z) > eps))
        {
            std::cout << "❗ Point " << i << " mismatch:"
                      << "\n   cloud1: (" << c1[i].x << ", " << c1[i].y << ", " << c1[i].z << ")"
                      << "\n   cloud2: (" << c2[i].x << ", " << c2[i].y << ", " << c2[i].z << ")"
                      << "\n   diff: ("
                      << std::abs(c1[i].x - c2[i].x) << ", "
                      << std::abs(c1[i].y - c2[i].y) << ", "
                      << std::abs(c1[i].z - c2[i].z) << ")" << std::endl;
            return false;
        }
    }
    return true;
}





// 返回后验状态向量组
std::vector<State, Eigen::aligned_allocator<State>> EskfEstimator::get_post_states() {
    return state_vector_post_;
}

// 返回先验状态向量组
std::vector<State, Eigen::aligned_allocator<State>> EskfEstimator::get_prior_states() {
    return state_vector_prior_;
}

// 获取当前状态（在雷达更新前使用 get_prior_states 会报错，这里使用 get_cur_state）
State EskfEstimator::get_cur_state() {
    return state_;
}

// 更新当前状态
void EskfEstimator::set_cur_state(State state_now) {
    state_ = state_now;
}
