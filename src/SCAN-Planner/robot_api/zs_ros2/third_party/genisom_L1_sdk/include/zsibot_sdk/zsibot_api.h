#ifndef ZSIBOT_API_H
#define ZSIBOT_API_H

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "zsibot_define.h"

namespace zsibot
{
class ZsibotExecutorImpl;
class ZsibotExecutor
{
   public:
    ZsibotExecutor(const Role role = Role::ROLE_REMOTE, const std::string &send_ip = "192.168.234.1",
                   const uint32_t send_port = 8081, const uint32_t recv_port = 8080);
    ~ZsibotExecutor();

    bool IsConnected() const;

    void SetCmd(const CmdCode cmd_code);
    void SetRemote(const std::array<float32_t, 4> &joy_stick, const std::array<float32_t, 14> &button);
    void SetWifi(const WifiInfo &wifi_info);

    uint32_t GetPower() const;
    float32_t GetTemperature() const;
    std::string GetSn() const;
    SpeedInfo GetSpeed() const;
    std::string GetDevName() const;
    std::string GetWifiSsid() const;
    VersionInfo GetVersion() const;
    Model GetModel() const;
    SpeedLevel GetSpeedLevel() const;
    FunctionMode GetFunctionMode() const;
    ControlMode GetControlMode() const;
    MotionMode GetMotionMode() const;
    MotionType GetMotionType() const;
    std::array<float32_t, 4> GetQuaternion() const;
    std::array<float32_t, 3> GetRPY() const;
    std::array<float32_t, 3> GetBodyAcc() const;
    std::array<float32_t, 3> GetBodyGyro() const;
    std::array<float32_t, 3> GetPosition() const;
    std::array<float32_t, 3> GetWorldVelocity() const;
    std::array<float32_t, 3> GetBodyVelocity() const;
    std::array<float32_t, 16> GetMotorTemp() const;
    std::array<float32_t, 4> GetLegAbadJoint() const;
    std::array<float32_t, 4> GetLegHipJoint() const;
    std::array<float32_t, 4> GetLegKneeJoint() const;
    std::array<float32_t, 4> GetLegFootJoint() const;
    std::array<float32_t, 4> GetLegAbadJointVel() const;
    std::array<float32_t, 4> GetLegHipJointVel() const;
    std::array<float32_t, 4> GetLegKneeJointVel() const;
    std::array<float32_t, 4> GetLegFootJointVel() const;
    std::array<float32_t, 4> GetLegAbadJointTorque() const;
    std::array<float32_t, 4> GetLegHipJointTorque() const;
    std::array<float32_t, 4> GetLegKneeJointTorque() const;
    std::array<float32_t, 4> GetLegFootJointTorque() const;
    std::vector<FaultInfo> GetFaultInfo() const;
    BatteryInfo GetBatteryInfo() const;
    std::optional<bool> GetWifiResutlf() const;
    bool IsRecvWifiResult() const;

   private:
    std::unique_ptr<ZsibotExecutorImpl> m_impl;
};
}  // namespace zsibot

#endif  // ZSIBOT_API_H