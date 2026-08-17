//
// Created by ss on 25-4-27.
//

#ifndef SHARED_BUFFERS_H
#define SHARED_BUFFERS_H

#include <deque>
#include <mutex>
#include <condition_variable>
#include "common_lib.h"
#include "type.h"

struct SharedBuffers
{

    // ========= 生产者 / 消费者共用的数据 =========
    std::deque<ImuMsgConst>                imu_buf;
    std::deque<PointCloudXYZI::Ptr>        lidar_buf;
    std::deque<double>                     lidar_base_time_buf;
    std::deque<double>                     img_time_buf;
    std::deque<WheelData>                  wheel_buf;
    std::deque<std::pair<Trigger, double>> trigger_buf;

    // ========= 互斥锁 & 条件变量 =========
    std::mutex                mtx;
    std::condition_variable   sig;


    // 某些时候包内的时间顺序和实际接收的时间顺序不一致，需要重新排序
    void sortTriggerBufByTimestamp(std::deque<std::pair<Trigger, double>>& buf) {
        // std::scoped_lock lk(mtx);
        std::sort(
            buf.begin(), buf.end(),
            [](const std::pair<Trigger, double>& a,
               const std::pair<Trigger, double>& b) {
                return a.second < b.second;  // 按 double 升序
            }
        );
    }

    // ========= 通用线程安全访问接口 =========

    // 安全 pop_front
    template<typename BufferType>
    void popFrontSafe(BufferType& buffer) {
        // std::scoped_lock lk(mtx);
        if (!buffer.empty()) {
            buffer.pop_front();
        }
    }

    // 安全 push_back
    template<typename BufferType, typename T>
    void pushBackSafe(BufferType& buffer, T&& value) {
        // std::scoped_lock lk(mtx);
        buffer.push_back(std::forward<T>(value));
    }

    // 批量安全 push_back 两个 buffer
    template<typename BufferType1, typename T1,
             typename BufferType2, typename T2>
    void pushBackDualSafe(BufferType1& buffer1, T1&& value1,
                          BufferType2& buffer2, T2&& value2)
    {
        // std::scoped_lock lk(mtx);
        buffer1.push_back(std::forward<T1>(value1));
        buffer2.push_back(std::forward<T2>(value2));
    }

    // 安全 empty 检查
    template<typename BufferType>
    bool isEmptySafe(const BufferType& buffer) {
        // std::scoped_lock lk(mtx);
        return buffer.empty();
    }

    // 安全取 front 元素
    template<typename BufferType>
    typename BufferType::value_type getFrontSafe(const BufferType& buffer) {
        // std::scoped_lock lk(mtx);
        return buffer.front();
    }

    // 安全取 back 元素
    template<typename BufferType>
    typename BufferType::value_type getBackSafe(const BufferType& buffer) {
        // std::scoped_lock lk(mtx);
        return buffer.back();
    }

    // 安全获取 size
    template<typename BufferType>
    size_t getSizeSafe(const BufferType& buffer) {
        // std::scoped_lock lk(mtx);
        return buffer.size();
    }

    // 安全 clear
    template<typename BufferType>
    void clearSafe(BufferType& buffer) {
        // std::scoped_lock lk(mtx);
        buffer.clear();
    }

    // 批量取出所有元素并清空（move）
    template<typename BufferType>
    BufferType moveOutAllSafe(BufferType& buffer) {
        // std::scoped_lock lk(mtx);
        BufferType tmp;
        std::swap(tmp, buffer);
        return tmp;
    }

    // 唤醒所有等待线程
    void notifyAll() {
        sig.notify_all();
    }
};



#endif //SHARED_BUFFERS_H
