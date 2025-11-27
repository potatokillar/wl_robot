#include "quadruped.hpp"

#include <iostream>

#include "apiDevice.hpp"
#include "apiQuadruped.hpp"
#include "metaData.hpp"
#include "sdkProtocolClient.hpp"

using namespace std;
namespace iiri::qr
{
Quadruped::Quadruped(const std::string &ip)
{
    rpcCli_ = make_shared<SdkProtocol>(ip);
    apiDev_ = make_unique<ApiDevice>(rpcCli_);
    apiQuad_ = make_unique<ApiQuadruped>(rpcCli_);
    cout << meta() << endl;  // 打印元数据
}

// 析构和移动使用默认
Quadruped::~Quadruped() {}
Quadruped::Quadruped(Quadruped &&rhs) = default;
Quadruped &Quadruped::operator=(Quadruped &&rhs) = default;

/**
 * @description: 元数据
 * @return {}
 */
const std::string_view &Quadruped::meta() { return SDK_VERSION; }

/**
 * @description: 设置线速度
 * @param x x方向，m/s
 * @param y y方向，m/s
 * @param z z方向，m/s
 * @return {}
 */
RetState Quadruped::SetLinearVelocity(double x, double y, double z) { return apiQuad_->SetLinearVelocity(x, y, z); }
RetState Quadruped::SetLinearVelocity(const LinearVelocityData &v) { return apiQuad_->SetLinearVelocity(v.x, v.y, v.z); }
/**
 * @description: 获取线速度
 * @param &x
 * @param &y
 * @param &z
 * @return {}
 */
RetState Quadruped::GetLinearVelocity(double *x, double *y, double *z) { return apiQuad_->GetLinearVelocity(*x, *y, *z); }
std::pair<RetState, LinearVelocityData> Quadruped::GetLinearVelocity()
{
    std::pair<RetState, LinearVelocityData> ret;
    ret.first = apiQuad_->GetLinearVelocity(ret.second.x, ret.second.y, ret.second.z);
    return ret;
}
RetState Quadruped::SubscribeLinearVelocity(std::function<void(const LinearVelocityData &)> func)
{
    return apiQuad_->SubscribeLinearVelocity(func);
}

/**
 * @description: 获取线速度的取值范围，单位m/s
 * @param &x
 * @param &y
 * @param &z
 * @return {}
 */
RetState Quadruped::GetLinearVelocityRange(ValueRange<double> *x, ValueRange<double> *y, ValueRange<double> *z)
{
    return apiQuad_->GetLinearVelocityInfo(*x, *y, *z);
}
std::tuple<RetState, ValueRange<double>, ValueRange<double>, ValueRange<double>> Quadruped::GetLinearVelocityRange()
{
    ValueRange<double> x, y, z;
    RetState ret = apiQuad_->GetLinearVelocityInfo(x, y, z);
    return {ret, x, y, z};
}
/**
 * @description: 设置角速度
 * @param x
 * @param y
 * @param z
 * @return {}
 */
RetState Quadruped::SetAngularVelocity(double x, double y, double z) { return apiQuad_->SetAngularVelocity(x, y, z); }
RetState Quadruped::SetAngularVelocity(const AngularVelocityData &v) { return apiQuad_->SetAngularVelocity(v.x, v.y, v.z); }

/**
 * @description: 获取角速度
 * @param &x
 * @param &y
 * @param &z
 * @return {}
 */

RetState Quadruped::GetAngularVelocity(double *x, double *y, double *z) { return apiQuad_->GetAngularVelocity(*x, *y, *z); }
std::pair<RetState, AngularVelocityData> Quadruped::GetAngularVelocity()
{
    std::pair<RetState, AngularVelocityData> ret;
    ret.first = apiQuad_->GetAngularVelocity(ret.second.x, ret.second.y, ret.second.z);
    return ret;
}
RetState Quadruped::SubscribeAngularVelocity(std::function<void(const AngularVelocityData &)> func)
{
    return apiQuad_->SubscribeAngularVelocity(func);
}

/**
 * @description: 获取角速度的取值范围
 * @param &x
 * @param &y
 * @param &z
 * @return {}
 */
RetState Quadruped::GetAngularVelocityRange(ValueRange<double> *x, ValueRange<double> *y, ValueRange<double> *z)
{
    return apiQuad_->GetAngularVelocityInfo(*x, *y, *z);
}
std::tuple<RetState, ValueRange<double>, ValueRange<double>, ValueRange<double>> Quadruped::GetAngularVelocityRange()
{
    ValueRange<double> x, y, z;
    RetState ret = apiQuad_->GetAngularVelocityInfo(x, y, z);
    return {ret, x, y, z};
}

/**
 * @description: 设置姿态
 * @param roll
 * @param pitch
 * @param yaw
 * @return {}
 */
RetState Quadruped::SetPose(double roll, double pitch, double yaw) { return apiQuad_->SetPose(roll, pitch, yaw); }
RetState Quadruped::SetPose(const PoseData &p) { return apiQuad_->SetPose(p.roll, p.pitch, p.yaw); }
RetState Quadruped::GetPose(double *roll, double *pitch, double *yaw) { return apiQuad_->GetPose(*roll, *pitch, *yaw); }
std::pair<RetState, PoseData> Quadruped::GetPose()
{
    std::pair<RetState, PoseData> ret;
    ret.first = apiQuad_->GetPose(ret.second.roll, ret.second.pitch, ret.second.yaw);
    return ret;
}
RetState Quadruped::SubscribePose(std::function<void(const PoseData &)> func) { return apiQuad_->SubscribePose(func); }

/**
 * @description: 获取姿态范围
 * @param &roll
 * @param &pitch
 * @param &yaw
 * @return {}
 */
RetState Quadruped::GetPoseRange(ValueRange<double> *roll, ValueRange<double> *pitch, ValueRange<double> *yaw)
{
    return apiQuad_->GetPoseInfo(*roll, *pitch, *yaw);
}
std::tuple<RetState, ValueRange<double>, ValueRange<double>, ValueRange<double>> Quadruped::GetPoseRange()
{
    ValueRange<double> roll, pitch, yaw;
    RetState ret = apiQuad_->GetPoseInfo(roll, pitch, yaw);
    return {ret, roll, pitch, yaw};
}

/**
 * @description: 设置高度
 * @param set
 * @return {}
 */
RetState Quadruped::SetHeight(double set) { return apiQuad_->SetHeight(set); }
std::pair<RetState, double> Quadruped::GetHeight()
{
    std::pair<RetState, double> ret;
    ret.first = apiQuad_->GetHeight(ret.second);
    return ret;
}
RetState Quadruped::SubscribeHeight(std::function<void(double)> func) { return apiQuad_->SubscribeHeight(func); }

std::pair<RetState, ValueRange<double>> Quadruped::GetHeightRange()
{
    std::pair<RetState, ValueRange<double>> ret;
    ret.first = apiQuad_->GetHeightInfo(ret.second);
    return ret;
}

/**
 * @description: 设置运行状态
 * 运行状态的改变，有可能导致线速度角速度姿态高度上下限的改变，建议重新获取
 * @param sta
 * @return {}
 */
RetState Quadruped::SetRunState(RunState sta) { return apiQuad_->SetRunState(sta); }
std::pair<RetState, RunState> Quadruped::GetRunState()
{
    std::pair<RetState, RunState> ret;
    ret.first = apiQuad_->GetRunState(ret.second);
    return ret;
}
RetState Quadruped::SubscribeRunState(std::function<void(RunState)> func) { return apiQuad_->SubscribeRunState(func); }

/**
 * @description: 获取运行模式的取值范围
 * @param &state
 * @return {}
 */
std::pair<RetState, std::vector<std::string>> Quadruped::GetRunStateRange()
{
    std::pair<RetState, std::vector<std::string>> ret;
    ret.first = apiQuad_->GetRunStateInfo(ret.second);
    return ret;
}

/**
 * @description: 设置步行模式
 * 步行模式的改变，有可能导致线速度角速度姿态高度上下限的改变，建议重新获取
 * @param mode
 * @return {}
 */
RetState Quadruped::SetWalkMode(WalkMode mode) { return apiQuad_->SetWalkMode(mode); }
std::pair<RetState, WalkMode> Quadruped::GetWalkMode()
{
    std::pair<RetState, WalkMode> ret;
    ret.first = apiQuad_->GetWalkMode(ret.second);
    return ret;
}
RetState Quadruped::SubscribeWalkMode(std::function<void(WalkMode)> func) { return apiQuad_->SubscribeWalkMode(func); }

std::pair<RetState, std::vector<std::string>> Quadruped::GetWalkModeRange()
{
    std::pair<RetState, std::vector<std::string>> ret;
    ret.first = apiQuad_->GetWalkModeInfo(ret.second);
    return ret;
}

/**
 * @description: 获取电池信息，例如电量，状态等
 * @param &info
 * @return {}
 */
std::pair<RetState, BatteryInfo> Quadruped::GetBatteryInfo()
{
    std::pair<RetState, BatteryInfo> ret;
    ret.first = apiDev_->GetBatteryInfo(ret.second);
    return ret;
}
std::pair<RetState, DeviceInfo> Quadruped::GetDeviceInfo()
{
    std::pair<RetState, DeviceInfo> ret;
    ret.first = apiDev_->GetDeviceInfo(ret.second);
    return ret;
}
/**
 * @description: 设置急停
 * @param set
 * @return {}
 */
RetState Quadruped::SetEmergStop(bool set)
{
    if (set == true) {
        return apiQuad_->SetRobotCtrl(RobotCtrl::enterEmgstop);
    }
    return apiQuad_->SetRobotCtrl(RobotCtrl::exitEmgstop);
}

RetState Quadruped::SubscribetRobotState(std::function<void(RobotState)> func) { return apiQuad_->SubscribetRobotState(func); }

std::pair<RetState, bool> Quadruped::GetEmergStop()
{
    std::pair<RetState, bool> ret;
    RobotState state;
    ret.first = apiQuad_->GetRobotState(state);
    if (state == RobotState::emgstop) {
        ret.second = true;
    } else {
        ret.second = false;
    }
    return ret;
}

std::pair<RetState, RobotState> Quadruped::GetRobotState()
{
    std::pair<RetState, RobotState> ret;
    ret.first = apiQuad_->GetRobotState(ret.second);
    return ret;
}

RetState Quadruped::SetLoadMass(double set) { return apiQuad_->SetLoadMass(set); }
std::pair<RetState, double> Quadruped::GetLoadMass()
{
    std::pair<RetState, double> ret;
    ret.first = apiQuad_->GetLoadMass(ret.second);
    return ret;
}
std::pair<RetState, ValueRange<double>> Quadruped::GetLoadMassRange()
{
    std::pair<RetState, ValueRange<double>> ret;
    ret.first = apiQuad_->GetLoadMassInfo(ret.second);
    return ret;
}

Result<qr::ImuRet> Quadruped::GetImuData() { return apiQuad_->GetImuData(); }

RetState Quadruped::SubscribetImuData(std::function<void(const ImuRet &)> func) { return apiQuad_->SubscribImuData(func); }
RetState Quadruped::SubscribeBatteryInfo(std::function<void(const BatteryInfo &)> func) { return apiDev_->SubscribeBatteryInfo(func); }
RetState Quadruped::SubscribeGamepadCmd(std::function<void(const GamepadCmd &)> func) { return apiDev_->SubscribeGamepadCmd(func); }

RetState Quadruped::SetDance(const std::string &name, uint32_t ms) { return apiQuad_->SetDance(name, ms); }

void Quadruped::SubscribeEvent(std::function<void(RobotEvent evt)> func) { apiDev_->SubscribeRobotEvent(func); }

RetState Quadruped::SetRgbLedToSolidColor(uint8_t red, uint8_t green, uint8_t blue)
{
    return apiDev_->SetRgbLedToSolidColor(red, green, blue);
}

RetState Quadruped::SetRgbLedToBreathing(uint8_t redStart, uint8_t greenStart, uint8_t blueStart, uint8_t redEnd, uint8_t greenEnd,
                                         uint8_t blueEnd, uint8_t cycle)
{
    return apiDev_->SetRgbLedToBreathing(redStart, greenStart, blueStart, redEnd, greenEnd, blueEnd, cycle);
}

RetState Quadruped::SetRgbLedToAutoBreathing(uint8_t autoCycle, uint8_t autoBrightnessPercentage)
{
    return apiDev_->SetRgbLedToAutoBreathing(autoCycle, autoBrightnessPercentage);
}

}  // namespace iiri::qr
