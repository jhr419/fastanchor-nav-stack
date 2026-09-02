// ...existing code...
#ifndef ZSIBIOT_DEFINE_H
#define ZSIBIOT_DEFINE_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace zsibot
{
using float32_t = float;

struct SpeedInfo
{
    float32_t speed = 0.0f;        // 前后速度
    float32_t angle_speed = 0.0f;  // 角速度
    float32_t shift_speed = 0.0f;  // 左右速度
    float32_t angle = 0.0f;        // 狗头角度
};

struct VersionInfo
{
    std::string mc_version;        // 运控版本
    std::string dog_task_version;  // 任务版本
};

enum class CmdCode
{
    CMD_NULL = 0,                         // 无效命令
    CMD_EMERGENCY_STOP = 0X5A,            // 紧急停止
    CMD_STAND_UP = 0X7A,                  // 站立
    CMD_SIT_DOWN = 0X6A,                  // 趴下
    CMD_MOVE_MODE = 0X8A,                 // 移动模式
    CMD_BALANCE_STAND_MODE = 0X9A,        // 平衡站立模式
    CMD_LOCK_MODE = 0X9B,                 // 锁定模式
    CMD_SLOW_SPEED = 0XAE,                // 慢速
    CMD_NORMAL_SPEED = 0XAF,              // 正常速度
    CMD_FAST_SPEED = 0xB0,                // 快速
    CMD_JUMP = 0x01,                      // 跳跃
    CMD_FORWARD_JUMP = 0x02,              // 前跳
    CMD_BACK_FLIP = 0x03,                 // 后空翻
    CMD_GREET = 0x04,                     // 招手
    CMD_TWO_LEG_STAND = 0x05,             // 双腿站立
    CMD_CRAWL_FORWARD = 0x09,             // 匍匐前进
    CMD_CLIMBING_HIGH_PLATFORM = 0x0A,    // 爬高台
    CMD_HAND_STAND = 0x0B,                // 双腿倒立
    CMD_UNLOAD_SQUAT = 0x0C,              // 卸货下蹲
    CMD_ENTER_LAB_MODE = 0xC1,            // 进入实验室模式
    CMD_EXIT_LAB_MODE = 0xC2,             // 退出实验室模式
    CMD_ENTER_ENTERTAINMENT_MODE = 0xC3,  // 进入文娱模式
    CMD_EXIT_ENTERTAINMENT_MODE = 0xC4,   // 退出文娱模式
    CMD_REMOTE_CONTROL_RIGHT = 0xAA,      // 请求遥控控制
    CMD_SDK_CONTROL_RIGHT = 0xB3,         // 请求SDK控制
    CMD_ROAMERX_CONTROL_RIGHT = 0xB4,     // 请求漫游控制
    CMD_GENERAL_SDK_CONTROL_RIGHT = 0xB5  // 请求通用SDK控制
};

struct FaultInfo
{
    std::string module;       // 故障所属模块
    std::string submodule;    // 故障所属子模块
    uint32_t error_code = 0;  // 故障代码
    std::string level;        // 故障等级
    std::string info;         // 故障信息
};

struct WifiInfo
{
    std::string ssid;      // wifi名称
    std::string password;  // wifi密码
};

// 电池信息
struct BatteryInfo
{
    float32_t current = 0.0f;  // 电流 (A)
    int32_t error = 0;         // 错误码
    int32_t power = 0;         // 电量百分比
    float32_t temp = 0.0f;     // 温度 (摄氏度)
    float32_t volt = 0.0f;     // 电压 (V)
};

enum class SpeedLevel
{
    SL_NULL = 0,    // 无效值
    SL_SLOW = 1,    // 慢速
    SL_NORMAL = 2,  // 正常速度
    SL_FAST = 3     // 快速
};

enum class Model
{
    MODEL_XG = 0,      // 小狗点足
    MODEL_XGW = 1,     // 小狗轮足
    MODEL_XGWHSPD = 2  // 小狗高速轮足
};

enum class FunctionMode
{
    FM_NULL = 0,         // 无效值
    FM_REMOTE = 1,       // 遥控模式
    FM_TRACE = 2,        // 追踪模式
    FM_SDK = 3,          // SDK模式
    FM_GENERAL_SDK = 4,  // 通用SDK模式
    FM_ROAMER = 5        // 漫游模式
};

enum class ControlMode
{
    CM_NULL = 0,                // 无效值
    CM_STAND_UP = 1,            // 站立
    CM_SIT_DOWN = 2,            // 趴下
    CM_LOCK_MODE = 3,           // 锁定模式
    CM_EMERGENCY_STOP = 4,      // 紧急停止
    CM_MOVE_MODE = 5,           // 移动模式
    CM_BALANCE_STAND_MODE = 6,  // 平衡站立模式
    CM_INIT = 7                 // 初始化

};

enum class MotionMode
{
    MM_NULL = 0,     // 无效值
    MM_RUNNING = 1,  // 执行中 在进行特技动作
    MM_REST = 2,     // 休息   未进行特技动作
    MM_FORBID = 3    // 禁止   不在实验室模式
};

enum class MotionType
{
    MT_NULL = 0,            // 无效值
    MT_JUMP = 1,            // 跳跃
    MT_FORWARD_JUMP = 2,    // 前跳
    MT_BACK_FLIP = 3,       // 后空翻
    MT_GREET = 4,           // 招手
    MT_TWO_LEG_STAND = 5,   // 双腿站立
    MT_FORBID = 6,          // 禁止    未开启狂暴模式
    MT_UNKNOWN = 7,         // 未知    未进行特技动作
    MT_UNLOADING_SQUAT = 8  // 卸货下蹲
};

enum class Role
{
    ROLE_NULL = 0,    // 无效值
    ROLE_REMOTE = 1,  // 遥控用户
    ROLE_SDK = 2      // SDK用户
};
}  // namespace zsibot

#endif  // ZSIBIOT_DEFINE_H