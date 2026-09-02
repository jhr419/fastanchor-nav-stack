/**
 * @file sdk_control.cpp
 * @brief SDK 模式示例 - 使用 ROLE_SDK 角色控制机器狗
 *
 * 本示例演示如何通过 SDK 模式控制机器狗。
 * 与遥控模式不同，SDK 模式需要先调用 CMD_SDK_CONTROL_RIGHT 申请控制权。
 *
 * 按键说明同 remote_control.cpp
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <bitset>
#include <iostream>
#include <thread>

#include "zsibot_sdk/zsibot_api.h"

using namespace zsibot;

// 设置终端为非阻塞模式
void set_conio_terminal_mode()
{
    struct termios new_termios;
    tcgetattr(STDIN_FILENO, &new_termios);
    new_termios.c_lflag &= ~(ICANON | ECHO);  // 关闭规范模式和回显
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

// 检查是否有输入
int kbhit()
{
    struct termios oldt, newt;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    int ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

int main()
{
    set_conio_terminal_mode();  // 设置终端为非阻塞模式

    // 使用默认的机器狗IP 192.168.234.1  使用默认的发送端口 8081 , 默认的接收端口8080, 此程序的的角色 ROLE_SDK
    ZsibotExecutor zsibot_exec(Role::ROLE_SDK);

    // 显示设备基本信息
    // std::cout << "设备序列号: " << zsibot_exec.GetSn() << std::endl;
    // std::cout << "设备名称: " << zsibot_exec.GetDevName() << std::endl;
    // std::cout << "当前电量: " << zsibot_exec.GetPower() << "%" << std::endl;
    // std::cout << "设备温度: " << zsibot_exec.GetTemperature() << "°C" << std::endl;
    // std::cout << "WiFi SSID: " << zsibot_exec.GetWifiSsid() << std::endl;

    // VersionInfo version = zsibot_exec.GetVersion();
    // std::cout << "主控版本: " << version.mc_version << std::endl;
    // std::cout << "任务版本: " << version.dog_task_version << std::endl;

    // 按键状态跟踪
    std::bitset<256> pressed_keys;

    //!! 注意： 本程序角色是SDK， 需要先调用CMD_SDK_CONTROL_RIGHT进入SDK模式，才可以操作机器狗
    //!! 注意： 跳跃、向上跳、打招呼、后空翻、双腿站立、卸货下蹲是实验室模式，
    //! ！       需要先调用CMD_ENTER_LAB_MODE进入实验室模式，才可以触发
    //!! 注意： 小狗轮足不能使用跳跃、向上跳、打招呼、后空翻、双腿站立功能，可以使用卸货下蹲功能

    while (true)
    {
        // 检测键盘输入
        if (kbhit())
        {
            char ch = getchar();  // 获取按键

            // 记录按键状态
            pressed_keys.set(static_cast<unsigned char>(ch), true);

            switch (ch)
            {
                case 'w':  // 向前移动
                    zsibot_exec.SetRemote({0.5, 0, 0, 0}, std::array<float32_t, 14>{0});
                    std::cout << "向前移动" << std::endl;
                    break;
                case 's':  // 向后移动
                    zsibot_exec.SetRemote({-0.5, 0, 0, 0}, std::array<float32_t, 14>{0});
                    std::cout << "向后移动" << std::endl;
                    break;
                case 'a':  // 向左移动
                    zsibot_exec.SetRemote({0, 0, 0.5, 0}, std::array<float32_t, 14>{0});
                    std::cout << "向左移动" << std::endl;
                    break;
                case 'd':  // 向右移动
                    zsibot_exec.SetRemote({0, 0, -0.5, 0}, std::array<float32_t, 14>{0});
                    std::cout << "向右移动" << std::endl;
                    break;
                case 'q':  // 左转
                    zsibot_exec.SetRemote({0, 0.5, 0, 0}, std::array<float32_t, 14>{0});
                    std::cout << "左转" << std::endl;
                    break;
                case 'e':  // 右转
                    zsibot_exec.SetRemote({0, -0.5, 0, 0}, std::array<float32_t, 14>{0});
                    std::cout << "右转" << std::endl;
                    break;
                case 'c':  // 停止移动
                    zsibot_exec.SetRemote({0, 0, 0, 0}, std::array<float32_t, 14>{0});
                    std::cout << "停止移动" << std::endl;
                    break;
                case '0':  // 切换到失能状态，机器狗软急停,完全爬下
                    zsibot_exec.SetCmd(CmdCode::CMD_EMERGENCY_STOP);
                    std::cout << "切换到失能状态，机器狗软急停,完全爬下" << std::endl;
                    break;
                case '1':  // 切换到匍匐
                    zsibot_exec.SetCmd(CmdCode::CMD_SIT_DOWN);
                    std::cout << "切换到匍匐" << std::endl;
                    break;
                case '2':  // 切换到站立
                    zsibot_exec.SetCmd(CmdCode::CMD_STAND_UP);
                    std::cout << "切换到站立" << std::endl;
                    break;
                case '3':  // 切换到跳跃
                    zsibot_exec.SetCmd(CmdCode::CMD_JUMP);
                    std::cout << "切换到跳跃" << std::endl;
                    break;
                case '4':  // 切换到向前跳跃
                    zsibot_exec.SetCmd(CmdCode::CMD_FORWARD_JUMP);
                    std::cout << "切换到向前跳跃" << std::endl;
                    break;
                case '5':  // 切换到打招呼
                    zsibot_exec.SetCmd(CmdCode::CMD_GREET);
                    std::cout << "切换到打招呼" << std::endl;
                    break;
                case '6':  // 切换到后空翻
                    zsibot_exec.SetCmd(CmdCode::CMD_BACK_FLIP);
                    std::cout << "切换到后空翻" << std::endl;
                    break;
                case '7':  // 切换到实验室模式
                    zsibot_exec.SetCmd(CmdCode::CMD_ENTER_LAB_MODE);
                    std::cout << "切换到实验室模式" << std::endl;
                    break;
                case '8':  // 切换到退出实验室模式
                    zsibot_exec.SetCmd(CmdCode::CMD_EXIT_LAB_MODE);
                    std::cout << "切换到退出实验室模式" << std::endl;
                    break;
                case '9':  // 切换到双腿站立 再按一次是取消双腿站立
                    zsibot_exec.SetCmd(CmdCode::CMD_TWO_LEG_STAND);
                    std::cout << "切换到双腿站立" << std::endl;
                    break;
                case 'k':  // 进入到sdk模式
                    zsibot_exec.SetCmd(CmdCode::CMD_SDK_CONTROL_RIGHT);
                    std::cout << "切换到sdk模式" << std::endl;
                    break;
                case 'g':  // 进入到通用sdk模式
                    zsibot_exec.SetCmd(CmdCode::CMD_GENERAL_SDK_CONTROL_RIGHT);
                    std::cout << "切换到通用sdk模式" << std::endl;
                    break;
                case 'r':  // 进入到remote模式
                    zsibot_exec.SetCmd(CmdCode::CMD_REMOTE_CONTROL_RIGHT);
                    std::cout << "切换到remote模式" << std::endl;
                    break;
                case 'x':  // 进入到roamerX模式
                    zsibot_exec.SetCmd(CmdCode::CMD_ROAMERX_CONTROL_RIGHT);
                    std::cout << "切换到roamerX模式" << std::endl;
                    break;
                case 'b':  // 进入平衡站立模式
                    zsibot_exec.SetCmd(CmdCode::CMD_BALANCE_STAND_MODE);
                    std::cout << "切换到平衡站立模式" << std::endl;
                    break;
                case 'l':  // 切换到文娱模式
                    zsibot_exec.SetCmd(CmdCode::CMD_ENTER_ENTERTAINMENT_MODE);
                    std::cout << "切换到文娱模式" << std::endl;
                    break;
                case 'm':  // 切换到退出文娱模式
                    zsibot_exec.SetCmd(CmdCode::CMD_EXIT_ENTERTAINMENT_MODE);
                    std::cout << "切换到退出文娱模式" << std::endl;
                    break;
                case 'u':  // 切换到卸货下蹲模式
                    zsibot_exec.SetCmd(CmdCode::CMD_UNLOAD_SQUAT);
                    std::cout << "切换到卸货下蹲模式" << std::endl;
                    break;
                case 'p':  // 切换到锁定模式
                    zsibot_exec.SetCmd(CmdCode::CMD_LOCK_MODE);
                    std::cout << "切换到锁定模式" << std::endl;
                    break;
                case 'i':  // 显示设备当前状态信息
                {
                    std::cout << "\n========== 设备状态信息 ==========" << std::endl;
                    std::cout << "电量: " << zsibot_exec.GetPower() << "%" << std::endl;
                    std::cout << "电池错误码: " << zsibot_exec.GetBatteryInfo().error << std::endl;
                    std::cout << "电池电压: " << zsibot_exec.GetBatteryInfo().volt << " V" << std::endl;
                    std::cout << "电池电流: " << zsibot_exec.GetBatteryInfo().current << " A" << std::endl;
                    std::cout << "电池温度: " << zsibot_exec.GetBatteryInfo().temp << " °C" << std::endl;
                    std::cout << "电池电量: " << zsibot_exec.GetBatteryInfo().power << " %" << std::endl;
                    std::cout << "温度: " << zsibot_exec.GetTemperature() << "°C" << std::endl;
                    std::cout << "速度等级: " << static_cast<int>(zsibot_exec.GetSpeedLevel()) << std::endl;
                    std::cout << "功能模式: " << static_cast<int>(zsibot_exec.GetFunctionMode()) << std::endl;
                    std::cout << "控制模式: " << static_cast<int>(zsibot_exec.GetControlMode()) << std::endl;
                    std::cout << "运动模式: " << static_cast<int>(zsibot_exec.GetMotionMode()) << std::endl;
                    std::cout << "运动类型: " << static_cast<int>(zsibot_exec.GetMotionType()) << std::endl;

                    SpeedInfo speed = zsibot_exec.GetSpeed();
                    std::cout << "速度: " << speed.speed << " m/s" << std::endl;
                    std::cout << "角速度: " << speed.angle_speed << " rad/s" << std::endl;
                    std::cout << "平移速度: " << speed.shift_speed << " m/s" << std::endl;
                    std::cout << "角度: " << speed.angle << " rad" << std::endl;

                    std::array<float32_t, 16> motor_temps = zsibot_exec.GetMotorTemp();
                    std::cout << "电机温度: ";
                    for (size_t i = 0; i < motor_temps.size() && i < 4; ++i)
                    {
                        std::cout << "M" << i << ": " << motor_temps[i] << "°C ";
                    }
                    std::cout << std::endl;

                    std::vector<FaultInfo> faults = zsibot_exec.GetFaultInfo();
                    if (faults.empty())
                    {
                        std::cout << "无故障信息" << std::endl;
                    }
                    else
                    {
                        std::cout << "故障信息:" << std::endl;
                        for (const auto &fault : faults)
                        {
                            std::cout << "  模块: " << fault.module << ", 子模块: " << fault.submodule
                                      << ", 错误码: " << fault.error_code << ", 等级: " << fault.level
                                      << ", 信息: " << fault.info << std::endl;
                        }
                    }

                    auto abad_joint = zsibot_exec.GetLegAbadJoint();
                    std::cout << " abad_joint: " << abad_joint[0] << std::endl;
                    std::cout << " hip_joint: " << abad_joint[1] << std::endl;
                    std::cout << " knee_joint: " << abad_joint[2] << std::endl;
                    std::cout << " foot_joint: " << abad_joint[3] << std::endl;
                    auto hip_joint = zsibot_exec.GetLegHipJoint();
                    std::cout << " hip_joint: " << hip_joint[0] << std::endl;
                    std::cout << " knee_joint: " << hip_joint[1] << std::endl;
                    std::cout << " foot_joint: " << hip_joint[2] << std::endl;
                    std::cout << " foot_joint: " << hip_joint[3] << std::endl;
                    auto knee_joint = zsibot_exec.GetLegKneeJoint();
                    std::cout << " knee_joint: " << knee_joint[0] << std::endl;
                    std::cout << " foot_joint: " << knee_joint[1] << std::endl;
                    std::cout << " foot_joint: " << knee_joint[2] << std::endl;
                    std::cout << " foot_joint: " << knee_joint[3] << std::endl;
                    auto foot_joint = zsibot_exec.GetLegFootJoint();
                    std::cout << " foot_joint: " << foot_joint[0] << std::endl;
                    std::cout << " foot_joint: " << foot_joint[1] << std::endl;
                    std::cout << " foot_joint: " << foot_joint[2] << std::endl;
                    std::cout << " foot_joint: " << foot_joint[3] << std::endl;
                    auto abad_joint_vel = zsibot_exec.GetLegAbadJointVel();
                    std::cout << " abad_joint_vel: " << abad_joint_vel[0] << std::endl;
                    std::cout << " abad_joint_vel: " << abad_joint_vel[1] << std::endl;
                    std::cout << " abad_joint_vel: " << abad_joint_vel[2] << std::endl;
                    std::cout << " abad_joint_vel: " << abad_joint_vel[3] << std::endl;
                    auto hip_joint_vel = zsibot_exec.GetLegHipJointVel();
                    std::cout << " hip_joint_vel: " << hip_joint_vel[0] << std::endl;
                    std::cout << " hip_joint_vel: " << hip_joint_vel[1] << std::endl;
                    std::cout << " hip_joint_vel: " << hip_joint_vel[2] << std::endl;
                    std::cout << " hip_joint_vel: " << hip_joint_vel[3] << std::endl;
                    auto knee_joint_vel = zsibot_exec.GetLegKneeJointVel();
                    std::cout << " knee_joint_vel: " << knee_joint_vel[0] << std::endl;
                    std::cout << " knee_joint_vel: " << knee_joint_vel[1] << std::endl;
                    std::cout << " knee_joint_vel: " << knee_joint_vel[2] << std::endl;
                    std::cout << " knee_joint_vel: " << knee_joint_vel[3] << std::endl;
                    auto foot_joint_vel = zsibot_exec.GetLegFootJointVel();
                    std::cout << " foot_joint_vel: " << foot_joint_vel[0] << std::endl;
                    std::cout << " foot_joint_vel: " << foot_joint_vel[1] << std::endl;
                    std::cout << " foot_joint_vel: " << foot_joint_vel[2] << std::endl;
                    std::cout << " foot_joint_vel: " << foot_joint_vel[3] << std::endl;
                    auto abad_joint_torque = zsibot_exec.GetLegAbadJointTorque();
                    std::cout << " abad_joint_torque: " << abad_joint_torque[0] << std::endl;
                    std::cout << " abad_joint_torque: " << abad_joint_torque[1] << std::endl;
                    std::cout << " abad_joint_torque: " << abad_joint_torque[2] << std::endl;
                    std::cout << " abad_joint_torque: " << abad_joint_torque[3] << std::endl;
                    auto hip_joint_torque = zsibot_exec.GetLegHipJointTorque();
                    std::cout << " hip_joint_torque: " << hip_joint_torque[0] << std::endl;
                    std::cout << " hip_joint_torque: " << hip_joint_torque[1] << std::endl;
                    std::cout << " hip_joint_torque: " << hip_joint_torque[2] << std::endl;
                    std::cout << " hip_joint_torque: " << hip_joint_torque[3] << std::endl;
                    auto knee_joint_torque = zsibot_exec.GetLegKneeJointTorque();
                    std::cout << " knee_joint_torque: " << knee_joint_torque[0] << std::endl;
                    std::cout << " knee_joint_torque: " << knee_joint_torque[1] << std::endl;
                    std::cout << " knee_joint_torque: " << knee_joint_torque[2] << std::endl;
                    std::cout << " knee_joint_torque: " << knee_joint_torque[3] << std::endl;
                    auto foot_joint_torque = zsibot_exec.GetLegFootJointTorque();
                    std::cout << " foot_joint_torque: " << foot_joint_torque[0] << std::endl;
                    std::cout << " foot_joint_torque: " << foot_joint_torque[1] << std::endl;
                    std::cout << " foot_joint_torque: " << foot_joint_torque[2] << std::endl;
                    std::cout << " foot_joint_torque: " << foot_joint_torque[3] << std::endl;
                    std::cout << "================================\n" << std::endl;
                    break;
                }

                case 'o':  // 设置WiFi信息示例（不会实际执行，因为需要真实的WiFi信息）
                    std::cout << "设置WiFi功能演示 (不会实际执行)" << std::endl;
                    // WifiInfo wifi_info;
                    // wifi_info.ssid = "example_ssid";
                    // wifi_info.password = "example_password";
                    // zsibot_exec.SetWifi(wifi_info);
                    break;

                default:
                    // 重置按键状态
                    break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));  // 限制发送频率
    }

    return 0;
}
