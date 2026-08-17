//
// Created by yczhang on 25-5-20.
//
#include "IMUProcess.h"
#include <iostream>
#include "ESEKF.h"
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <utility>

using namespace std;

/**
 * @brief 积分一个 IMU 帧，并更新环形缓冲区
 * @param IMU_acc
 * @param IMU_gyr
 * @param time_stamp
 */
//TODO 初始化问题
void IMUProcess::integration(Vector3d IMU_acc, Vector3d IMU_gyr, double time_stamp) {

    /* ---------- 1. 写入环形缓冲 ---------- */
    imu_acc_buffer[head_]  = std::move(IMU_acc);
    imu_gyr_buffer[head_]  = std::move(IMU_gyr);
    time_buffer[head_] = time_stamp;

    /* ---------- 2. 第一帧：初始化 ---------- */
    if (count_ == 0) {
        // 已经在外部调用 set_init_state 初始化了
        if (!initialized)initialize(time_stamp);
        imu_gap_buffer[head_] = false;
        // 第一帧不计算，仅写入状态
    }
    else {
        /* 2.1 上一帧索引与 Δt */
        int    prev = (head_ - 1 + MAX_LEN) % MAX_LEN;
        double dt   = time_stamp - time_buffer[prev];

        /* 2.2 梯形法求平均量 */
        Eigen::Vector3d acc_avg = 0.5 * (imu_acc_buffer[prev] + imu_acc_buffer[head_]);
        Eigen::Vector3d gyr_avg = 0.5 * (imu_gyr_buffer[prev] + imu_gyr_buffer[head_]);
       // Eigen::Vector3d acc_avg = 0.5 * (imu_acc_buffer[prev] + IMU_acc);
       // Eigen::Vector3d gyr_avg = 0.5 * (imu_gyr_buffer[prev] + IMU_gyr);

        /* 2.3 预测下一状态 */
        states_imu[head_] = propagateOrHold(states_imu[prev], acc_avg, gyr_avg, dt,
                                            time_stamp, imu_gap_buffer[head_]);

        /* 2.4 调试日志  */
        logFrame();
    }

    /* ---------- 3. 更新环形指针 ---------- */
    head_  = (head_ + 1) % MAX_LEN;
    count_ = std::min(count_ + 1, MAX_LEN);
}

void IMUProcess::set_init_state(State state_init ,double time_stamp) {
    /* ---------- 填充环形缓冲 ---------- */
    std::fill(states_imu.begin(), states_imu.end(), state_init);
    std::fill(time_buffer.begin(), time_buffer.end(), time_stamp);
    std::fill(imu_gap_buffer.begin(), imu_gap_buffer.end(), false);
    std::fill(imu_acc_buffer.begin(),  imu_acc_buffer.end(),  Eigen::Vector3d::Zero());
    std::fill(imu_gyr_buffer.begin(),  imu_gyr_buffer.end(),  Eigen::Vector3d::Zero());

    /* ---------- 过程噪声 Q (12×12) ---------- */
    Q_.setZero(StateNoiseIndex::NOISE_TOTAL,StateNoiseIndex::NOISE_TOTAL);
    Q_.block<3,3>(StateNoiseIndex::ACC_NOISE,        StateNoiseIndex::ACC_NOISE)         = ACC_NOISE_VAR        * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_NOISE,       StateNoiseIndex::GYRO_NOISE)        = GYRO_NOISE_VAR       * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::ACC_RANDOM_WALK,  StateNoiseIndex::ACC_RANDOM_WALK)   = ACC_RANDOM_WALK_VAR  * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK)  = GYRO_RANDOM_WALK_VAR * Eigen::Matrix3d::Identity();

    initialized = true;
}

void IMUProcess::integration(ImuMsgConst imu_msg, Eigen::Vector3d mean_acc) {
    Eigen::Vector3d cur_gyr (imu_msg->angular_velocity.x,
                             imu_msg->angular_velocity.y,
                             imu_msg->angular_velocity.z);
    Eigen::Vector3d cur_acc (imu_msg->linear_acceleration.x,
                             imu_msg->linear_acceleration.y,
                             imu_msg->linear_acceleration.z);
    cur_acc = cur_acc * G_m_s2 / mean_acc.norm();
    double time_stamp = get_time_sec(imu_msg->header.stamp);
    integration(cur_acc, cur_gyr, time_stamp);
}

/**
 * @brief  根据上一状态 + 平均 IMU（body 系） + dt 预测下一状态并传播协方差
 * @param  prev     上一时刻状态（均值 & 协方差）
 * @param  acc_b    本区间平均线加速度  [m/s²]  (body frame, 未去 bias)
 * @param  gyr_b    本区间平均角速度    [rad/s] (body frame, 未去 bias)
 * @param  dt       区间长度           [s]
 * @param  stamp    输出状态的时间戳    [s]
 * @return          预测后的 State
 */
State IMUProcess::predictOnce(const State            &prev,
                              const Eigen::Vector3d  &acc_b,
                              const Eigen::Vector3d  &gyr_b,
                              double                  dt,
                              double                  stamp) const
{
    /* === 0. 复制旧状态，准备写新量 === */
    State nxt = prev;      // bias、g 等直接拷贝
    nxt.time = stamp;      // 最后统一写入

    /* === 1. 姿态更新（四元数左乘） === */
    Eigen::Vector3d dtheta = (gyr_b - prev.bw) * dt;     // 去 gyro-bias
    double th = dtheta.norm();
    Eigen::Quaterniond dq(1.0, 0.0, 0.0, 0.0);
    if (th > 1e-12) {
        Eigen::Vector3d axis = dtheta / th;
        double half = 0.5 * th;
        dq.w()   = std::cos(half);
        dq.vec() = axis * std::sin(half);
    }
    nxt.q = (prev.q * dq).normalized();

    /* === 2. 世界系线加速度 === */
    Eigen::Vector3d acc_w = nxt.q * (acc_b - prev.ba) + prev.g;

    /* === 3. 位置 & 速度 === */
    nxt.p = prev.p + prev.v * dt + 0.5 * acc_w * dt * dt;
    nxt.v = prev.v + acc_w * dt;

    /* ----------------------------------------------------------------
     *                       协  方  差  传  播
     * ---------------------------------------------------------------- */
    /* === 4. 构建连续时间雅可比 A 和 噪声耦合 U === */
    Eigen::Matrix<double, STATE_TOTAL, STATE_TOTAL> A;
    A.setZero();

    // A(δθ,δθ)
    A.block<3,3>(R, R) = -skew3d(gyr_b - prev.bw);
    // A(δθ,δbw)
    A.block<3,3>(R, BW) = -Eigen::Matrix3d::Identity();

    // A(δp,δv)
    A.block<3,3>(P, V) = Eigen::Matrix3d::Identity();

    // A(δv,δθ)
    A.block<3,3>(V, R) = nxt.q.toRotationMatrix() * -skew3d(acc_b - prev.ba);
    // A(δv,δba)
    A.block<3,3>(V, BA) = -nxt.q.toRotationMatrix();
    // A(δv,δg)
    A.block<3,3>(V, G) =  Eigen::Matrix3d::Identity();

    /* --- U (18×12) --- */
    Eigen::Matrix<double, STATE_TOTAL, NOISE_TOTAL> U;
    U.setZero();
    U.block<3,3>(R, R)   = -Eigen::Matrix3d::Identity();           // gyro-noise
    U.block<3,3>(V, P)   = -nxt.q.toRotationMatrix();              // acc-noise
    U.block<3,3>(BW, V) =  Eigen::Matrix3d::Identity();           // gyro-bias random walk
    U.block<3,3>(BA, BW) =  Eigen::Matrix3d::Identity();           // acc-bias random walk

    /* === 5. 离散化：F ≈ I + A·dt,  Vd ≈ U·dt === */
    Eigen::Matrix<double, STATE_TOTAL, STATE_TOTAL> F =
        Eigen::Matrix<double, STATE_TOTAL, STATE_TOTAL>::Identity() + dt * A;

    Eigen::Matrix<double, STATE_TOTAL, NOISE_TOTAL> Vd = dt * U;

    /* === 6. 协方差传播 === */
    nxt.P = F * prev.P * F.transpose() + Vd * Q_ * Vd.transpose();

    return nxt;
}

State IMUProcess::propagateOrHold(const State            &prev,
                                  const Eigen::Vector3d  &acc_b,
                                  const Eigen::Vector3d  &gyr_b,
                                  double                  dt,
                                  double                  stamp,
                                  bool                   &imu_gap) const
{
    imu_gap = dt <= 0.0 || dt > MAX_IMU_INTERVAL_SEC;
    if (!imu_gap) {
        return predictOnce(prev, acc_b, gyr_b, dt, stamp);
    }

    State nxt = prev;
    nxt.time = stamp;
    if (dt > 0.0) {
        // The motion during a missing interval is unknown. Keep the last
        // nominal state, but make the loss of confidence visible to updates.
        nxt.P += Eigen::MatrixXd::Identity(STATE_TOTAL, STATE_TOTAL)
                 * IMU_GAP_COVARIANCE_RATE * dt;
    }
    std::cerr << "[IMUProcess] Warning: invalid IMU interval dt = "
              << dt << " s. Hold state and reject LiDAR frames crossing the gap."
              << std::endl;
    return nxt;
}

/* --------------------------------------------------------------------
 *  IMUProcess::initialize
 *  初始化环形缓冲、初始状态、协方差、过程噪声
 * ------------------------------------------------------------------*/
void IMUProcess::initialize(double t0 )
{
    initialized = true;
    /* ---------- 0. 初始 State ---------- */
    State init;
    init.q  = Eigen::Quaterniond::Identity();
    init.p  = Eigen::Vector3d::Zero();
    init.v  = Eigen::Vector3d::Zero();
    init.bw = Eigen::Vector3d::Zero();
    init.ba = Eigen::Vector3d::Zero();
    init.g  = Eigen::Vector3d(0, 0, -G_m_s2);
    init.time = t0;

    /* 协方差 P (18×18) */
    init.P.setZero(StateIndex::STATE_TOTAL,StateIndex::STATE_TOTAL);
    init.P.block<3,3>(StateIndex::R,  StateIndex::R)  = Eigen::Matrix3d::Identity() * 1e-4;
    init.P.block<3,3>(StateIndex::P,  StateIndex::P)  = Eigen::Matrix3d::Identity() * 1e-2;
    init.P.block<3,3>(StateIndex::V,  StateIndex::V)  = Eigen::Matrix3d::Identity() * 1e-2;
    init.P.block<3,3>(StateIndex::BW, StateIndex::BW) = Eigen::Matrix3d::Identity() * 1e-6;
    init.P.block<3,3>(StateIndex::BA, StateIndex::BA) = Eigen::Matrix3d::Identity() * 1e-6;
    init.P.block<3,3>(StateIndex::G,  StateIndex::G)  = Eigen::Matrix3d::Identity() * 1e-4;

    /* ---------- 1. 填充环形缓冲 ---------- */
    std::fill(states_imu.begin(), states_imu.end(), init);
    std::fill(time_buffer.begin(), time_buffer.end(), t0);
    std::fill(imu_gap_buffer.begin(), imu_gap_buffer.end(), false);
    std::fill(imu_acc_buffer.begin(),  imu_acc_buffer.end(),  Eigen::Vector3d::Zero());
    std::fill(imu_gyr_buffer.begin(),  imu_gyr_buffer.end(),  Eigen::Vector3d::Zero());

    /* ---------- 2. 过程噪声 Q (12×12) ---------- */
    Q_.setZero(StateNoiseIndex::NOISE_TOTAL,StateNoiseIndex::NOISE_TOTAL);
    Q_.block<3,3>(StateNoiseIndex::ACC_NOISE,        StateNoiseIndex::ACC_NOISE)         = ACC_NOISE_VAR        * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_NOISE,       StateNoiseIndex::GYRO_NOISE)        = GYRO_NOISE_VAR       * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::ACC_RANDOM_WALK,  StateNoiseIndex::ACC_RANDOM_WALK)   = ACC_RANDOM_WALK_VAR  * Eigen::Matrix3d::Identity();
    Q_.block<3,3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK)  = GYRO_RANDOM_WALK_VAR * Eigen::Matrix3d::Identity();

    std::cout << "[IMUProcess] Initialized at t = " << t0 << " s\n";
}

std::ofstream IMUProcess::csv_log_;

/* ========= 静态 ofstream 定义 ========= */
void IMUProcess::logFrame()
{
    if (!DEBUG || count_ == 0) return;                // 未启用或无数据

    /* 1) 若首次调用则创建文件并写表头 */
    if (!csv_log_.is_open()) {
        csv_log_.open(string(PACKAGE_ROOT_DIR) + "/imu_state_log.csv", std::ios::out | std::ios::trunc);
        csv_log_ << std::fixed << std::setprecision(9)
                 << "stamp,"
                 << "ax,ay,az,"
                 << "gx,gy,gz,"
                 << "px,py,pz,"
                 << "vx,vy,vz,"
                 << "qw,qx,qy,qz,"
                 << "bw_x,bw_y,bw_z,"
                 << "ba_x,ba_y,ba_z\n";
    }

    /* 2) 取出“最新帧”索引 */
    int idx = (head_ - 1 + MAX_LEN) % MAX_LEN;

    /* 3) 写入数据 */
    csv_log_
        << time_buffer[idx] << ','
        << imu_acc_buffer[idx].x() << ',' << imu_acc_buffer[idx].y() << ',' << imu_acc_buffer[idx].z() << ','
        << imu_gyr_buffer[idx].x() << ',' << imu_gyr_buffer[idx].y() << ',' << imu_gyr_buffer[idx].z() << ','
        << states_imu[idx].p.x()  << ',' << states_imu[idx].p.y()  << ',' << states_imu[idx].p.z()  << ','
        << states_imu[idx].v.x()  << ',' << states_imu[idx].v.y()  << ',' << states_imu[idx].v.z()  << ','
        << states_imu[idx].q.w()  << ',' << states_imu[idx].q.x()  << ',' << states_imu[idx].q.y()  << ',' << states_imu[idx].q.z() << ','
        << states_imu[idx].bw.x() << ',' << states_imu[idx].bw.y() << ',' << states_imu[idx].bw.z() << ','
        << states_imu[idx].ba.x() << ',' << states_imu[idx].ba.y() << ',' << states_imu[idx].ba.z()
        << '\n';
}

// 环形缓冲索引映射：保证返回值在 [0, MAX_LEN)
inline int IMUProcess::wrapIndex(int idx) const {
    idx %= MAX_LEN;
    return idx < 0 ? idx + MAX_LEN : idx;
}

void IMUProcess::set_state_at_t(State &state_in, double time_stamp) {
    if (time_stamp > time_buffer[wrapIndex(head_ - 1)])
        cerr  << "[IMUProcess] Error: time_stamp is larger than the latest time stamp in the buffer." << endl;

    // 1. 查找插入位置和插入模式
    int index = -1;
    // 逆序查找第一个时间戳小于目标时间戳的索引位置
    int lower_index = -1;
    for (int i = 0; i < count_; ++i) {
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp - 1E-6 ) {
            lower_index = time_index;
            break;
        }
    }

    // 如果所有时间戳都大于目标时间戳，则插入在第一个位置，同时警告
    if (lower_index == -1) {
        index = wrapIndex(head_ - count_ );
        cerr << "[IMUProcess] Warning: timestamp was pushed to the front of the queue, which means the previous"
                "imu may have be aborted. assert_time_stamp : "
             << std::fixed << std::setprecision(6) << time_stamp
             << " The earliest time : "
             << std::fixed << std::setprecision(6) << time_buffer[index] << endl;
    }

    // 3. 检查是否有完全重合的时间戳
    int next_index = wrapIndex(lower_index + 1);
    Mode mode = std::abs(time_buffer[next_index] - time_stamp) < 1E-6 ? REPLACE :INSERT;

    if (mode == REPLACE) {
        index = next_index;
    }else {
        index = lower_index;
    }

    // 2. 更新目标位置的状态和时间戳
    states_imu[index] = state_in;
    time_buffer[index] = time_stamp;

    // 3. 从目标位置开始重新积分后续状态
    for (int i = 1; wrapIndex(index + i) != head_; ++i) {
        int index_ = wrapIndex(index + i);
        int pre_index_ = wrapIndex(index + i - 1);
        double dt = time_buffer[index_] - time_buffer[pre_index_];


        /* 2.2 梯形法求平均量 */
        Eigen::Vector3d acc_avg = 0.5 * (imu_acc_buffer[pre_index_] + imu_acc_buffer[index_]);
        Eigen::Vector3d gyr_avg = 0.5 * (imu_gyr_buffer[pre_index_] + imu_gyr_buffer[index_]);

        // 使用预测函数计算下一个状态
        states_imu[index_] = propagateOrHold(states_imu[pre_index_], acc_avg, gyr_avg, dt,
                                             time_buffer[index_], imu_gap_buffer[index_]);
    }
}

void IMUProcess::get_state_at_t(State &state_out, double time_stamp) {
    // 1. 时间范围检查
    if (time_stamp >= time_buffer[wrapIndex(head_-1)] + 1E-6 ||
        time_stamp <= time_buffer[wrapIndex(head_-count_)] - 1E-6) {
        cerr << "[IMUProcess] Error: time_stamp is out of range." << endl;
        return;
    }

    // 2. 逆序查找第一个时间戳小于目标时间戳的索引位置
    int lower_index = -1;
    for (int i = 0; i < count_; ++i) {
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp - 1E-6 ) {
            lower_index = time_index;
            break;
        }
    }

    // 3. 检查是否有完全重合的时间戳
    int next_index = wrapIndex(lower_index + 1);
    Mode mode = std::abs(time_buffer[next_index] - time_stamp) < 1E-6 ? REPLACE :INSERT;

     if (mode == REPLACE) {
        state_out = states_imu[next_index];
    }else {
        int left_index = lower_index;
        int right_index = next_index;
        // 4. 计算插值比例
        double t_left = time_buffer[left_index];
        double t_right = time_buffer[right_index];
        double ratio = (time_stamp - t_left) / (t_right - t_left);
        // 5. 进行状态插值
        const State& left_state = states_imu[left_index];
        const State& right_state = states_imu[right_index];
        // 位置、速度、bias和重力使用线性插值
        state_out.p = (1.0 - ratio) * left_state.p + ratio * right_state.p;
        state_out.v = (1.0 - ratio) * left_state.v + ratio * right_state.v;
        state_out.ba = (1.0 - ratio) * left_state.ba + ratio * right_state.ba;
        state_out.bw = (1.0 - ratio) * left_state.bw + ratio * right_state.bw;
        state_out.g = (1.0 - ratio) * left_state.g + ratio * right_state.g;
        // 姿态使用四元数球面线性插值
        state_out.q = left_state.q.slerp(ratio, right_state.q);
        // 协方差矩阵取较大的
        state_out.P = (ratio < 0.5) ? left_state.P : right_state.P;
        state_out.time = time_stamp;
    }
}

inline Pose IMUProcess::get_pose(int index , double base_time) {
    Pose IMU_Pose;
    State state_ = states_imu[index];
    IMU_Pose.q = state_.q;
    IMU_Pose.p = state_.p;
    IMU_Pose.v = state_.v;
    Eigen::Vector3d acc_world = state_.q * (imu_acc_buffer[index] - state_.ba) + state_.g;
    Eigen::Vector3d gyr = imu_gyr_buffer[index];
    IMU_Pose.acc = acc_world ;
    IMU_Pose.gyr = gyr - state_.bw;
    IMU_Pose.offset_time = time_buffer[index] - base_time;
    return IMU_Pose;
}

void IMUProcess::get_imu_pose_from_t1_to_t2(std::vector<Pose> &IMU_Pose_vec, double time_stamp_1, double time_stamp_2) {
    // 1. 清空输出容器
    IMU_Pose_vec.clear();

    // 2. 检查时间范围有效性
    if (time_stamp_1 > time_stamp_2) {
        cerr << "[IMUProcess] Warnning: time_stamp_1 is larger than time_stamp_2. Already been reversed." << endl;
        double time_temp = time_stamp_1;
        time_stamp_1 = time_stamp_2;
        time_stamp_2 = time_temp;
    }

    if (rangeCrossesImuGap(time_stamp_1, time_stamp_2)) {
        cerr << "[IMUProcess] Warning: Requested LiDAR interval crosses an IMU gap. "
             << "Drop this LiDAR frame." << endl;
        return;
    }

    // 3. 检查请求时间是否在缓冲区范围内
    double min_time = time_buffer[wrapIndex(head_ - count_)];
    double max_time = time_buffer[wrapIndex(head_ - 1)];
    if (time_stamp_1 < min_time - 1E-6 || time_stamp_2 > max_time + 1E-6) {
        cerr << "[IMUProcess] Error: Requested time range is out of range." << endl;
        return;
    }

    // 4. 逆序查找时间戳小于目标时间戳的索引位置
    int left_index = -1,  right_index = -1 , i ;
    
    for (i = 0; i < count_; ++i) { // 第一次查找小于 time_stamp_2 的索引
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp_2 - 1E-6 ) {
            right_index = time_index;
            break;
        }
    }
    
    for (; i < count_; ++i) { // 第二次查找小于 time_stamp_1 的索引
        int time_index = wrapIndex(head_ - 1 - i);
        if (time_buffer[time_index] < time_stamp_1 - 1E-6 ) {
            left_index = time_index;
            break;
        }
    }

    // 5. 未找到索引
    if (left_index == -1 || right_index == -1) {
        cerr << "[IMUProcess] Error: IMU deque size may be too small or time span is too large. "
        << "IMU deque size = " << MAX_LEN << " time span = " << time_stamp_2 - time_stamp_1 << endl;
        return;
    }

    // 检查左侧是否存在重合时间戳
    State state_begin;
    int next_index = wrapIndex(left_index + 1);
    Mode mode = std::abs(time_buffer[next_index] - time_stamp_1) < 1E-6 ? REPLACE :INSERT;

    if (mode == REPLACE) {
        // 添加范围内的完整状态【注意这里已经把 right_index 对应的插入了】
        for (int j = wrapIndex(left_index + 1); j != wrapIndex(right_index + 1); j = wrapIndex(j+1)) {
            IMU_Pose_vec.push_back(get_pose(j,time_stamp_1));
        }
    }else {
        // 多插值一个起始状态
        double ratio = (time_stamp_1 - time_buffer[left_index]) /
                       (time_buffer[next_index] - time_buffer[left_index]);
        const State& prev_state = states_imu[left_index];
        const State& next_state = states_imu[next_index];

        state_begin.p = (1.0 - ratio) * prev_state.p + ratio * next_state.p;
        state_begin.v = (1.0 - ratio) * prev_state.v + ratio * next_state.v;
        state_begin.ba = (1.0 - ratio) * prev_state.ba + ratio * next_state.ba;
        state_begin.bw = (1.0 - ratio) * prev_state.bw + ratio * next_state.bw;
        state_begin.g = (1.0 - ratio) * prev_state.g + ratio * next_state.g;
        state_begin.q = prev_state.q.slerp(ratio, next_state.q);
        state_begin.P = (ratio < 0.5) ? prev_state.P : next_state.P;
        state_begin.time = time_stamp_1;

        Eigen::Vector3d imu_acc_begin = (1.0 - ratio) * imu_acc_buffer[left_index] + ratio * imu_acc_buffer[next_index];
        Eigen::Vector3d imu_gyr_begin = (1.0 - ratio) * imu_gyr_buffer[left_index] + ratio * imu_gyr_buffer[next_index];

        Pose IMU_Pose;
        IMU_Pose.q = state_begin.q;
        IMU_Pose.p = state_begin.p;
        IMU_Pose.v = state_begin.v;
        Eigen::Vector3d acc_world = state_begin.q * (imu_acc_begin - state_begin.ba) + state_begin.g;
        Eigen::Vector3d gyr = imu_gyr_begin;
        IMU_Pose.acc = acc_world ;
        IMU_Pose.gyr = gyr - state_begin.bw;
        IMU_Pose.offset_time = 0;

        IMU_Pose_vec.push_back(IMU_Pose);
        for (int j = wrapIndex(left_index + 1); j != wrapIndex(right_index + 1); j = wrapIndex(j+1)) {
            IMU_Pose_vec.push_back(get_pose(j,time_stamp_1));
        }
    }

    // 检查右侧是否存在重合时间戳
    State state_end;
    next_index = wrapIndex(right_index + 1);
    mode = std::abs(time_buffer[next_index] - time_stamp_2) < 1E-6 ? REPLACE : INSERT;

    if (mode == REPLACE) {
        IMU_Pose_vec.push_back(get_pose(next_index,time_stamp_2));
    }else {
        // 多插值一个结束状态
        double ratio = (time_stamp_2 - time_buffer[right_index]) /
                      (time_buffer[next_index] - time_buffer[right_index]);
        const State& prev_state = states_imu[right_index];
        const State& next_state = states_imu[next_index];

        state_end.p = (1.0 - ratio) * prev_state.p + ratio * next_state.p;
        state_end.v = (1.0 - ratio) * prev_state.v + ratio * next_state.v;
        state_end.ba = (1.0 - ratio) * prev_state.ba + ratio * next_state.ba;
        state_end.bw = (1.0 - ratio) * prev_state.bw + ratio * next_state.bw;
        state_end.g = (1.0 - ratio) * prev_state.g + ratio * next_state.g;
        state_end.q = prev_state.q.slerp(ratio, next_state.q);
        state_end.P = (ratio < 0.5) ? prev_state.P : next_state.P;
        state_end.time = time_stamp_2;

        Eigen::Vector3d imu_acc_end = (1.0 - ratio) * imu_acc_buffer[right_index] + ratio * imu_acc_buffer[next_index];
        Eigen::Vector3d imu_gyr_end = (1.0 - ratio) * imu_gyr_buffer[right_index] + ratio * imu_gyr_buffer[next_index];

        Pose IMU_Pose;
        IMU_Pose.q = state_begin.q;
        IMU_Pose.p = state_begin.p;
        IMU_Pose.v = state_begin.v;
        Eigen::Vector3d acc_world = state_begin.q * (imu_acc_end - state_begin.ba) + state_begin.g;
        Eigen::Vector3d gyr = imu_gyr_end;
        IMU_Pose.acc = acc_world ;
        IMU_Pose.gyr = gyr - state_begin.bw;
        IMU_Pose.offset_time = time_stamp_2 - time_stamp_1;

        IMU_Pose_vec.push_back(IMU_Pose);
    }
}

bool IMUProcess::rangeCrossesImuGap(double time_stamp_1, double time_stamp_2) const {
    for (int i = 0; i < count_; ++i) {
        int current_index = wrapIndex(head_ - 1 - i);
        if (!imu_gap_buffer[current_index]) {
            continue;
        }

        int previous_index = wrapIndex(current_index - 1);
        double gap_begin = time_buffer[previous_index];
        double gap_end = time_buffer[current_index];
        if (time_stamp_1 < gap_end - 1E-6 && time_stamp_2 > gap_begin + 1E-6) {
            return true;
        }
    }
    return false;
}
