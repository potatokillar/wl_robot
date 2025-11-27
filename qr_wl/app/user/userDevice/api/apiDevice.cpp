#include "apiDevice.hpp"

#include "baseline.hpp"
#include "canBridge.hpp"
#include "deviceCustomParam.hpp"
#include "robotMedia.hpp"
#include "robotState.hpp"

using namespace std;

std::pair<bool, std::vector<BoardInfo>> ApiDevice::GetPCBNumber(void) { return hardwareAPI_.GetPCBNumber(); }

std::pair<bool, PowerInfo> ApiDevice::GetMotorPowerStatus() { return hardwareAPI_.GetMotorPowerStatus(); }

std::pair<bool, PowerInfo> ApiDevice::GetBMSPowerStatus() { return hardwareAPI_.GetBMSPowerStatus(); }

std::pair<bool, PowerInfo> ApiDevice::SetMotorPowerOn() { return hardwareAPI_.SetMotorPowerOn(); }

std::pair<bool, PowerInfo> ApiDevice::SetMotorPowerOff() { return hardwareAPI_.SetMotorPowerOff(); }

std::pair<bool, PowerInfo> ApiDevice::SetBMSPowerOffCancel() { return hardwareAPI_.SetBMSPowerOffCancel(); }

std::pair<bool, PowerInfo> ApiDevice::SetBMSPowerOff() { return hardwareAPI_.SetBMSPowerOff(); }

TimestampedData<bool> ApiDevice::GetEmergStop() { return hardwareAPI_.GetEmergStop(); }

std::array<TimestampedData<s16>, 4> ApiDevice::GetMidsizeBMSTemperature() { return hardwareAPI_.GetMidsizeBMSTemperature(); }

TimestampedData<s16> ApiDevice::GetMidsizePowerPcbTemperature() { return hardwareAPI_.GetMidsizePowerPcbTemperature(); }

std::array<TimestampedData<s16>, 3> ApiDevice::GetSmallBMSTemperature() { return hardwareAPI_.GetSmallBMSTemperature(); }

double ApiDevice::GetLegCurrent() { return ((double)hardwareAPI_.GetLegCurrent().data) / 1000; }

double ApiDevice::GetLegCurrent2() { return ((double)hardwareAPI_.GetLegCurrent2().data) / 1000; }

double ApiDevice::GetLegCurrent3() { return ((double)hardwareAPI_.GetLegCurrent3().data) / 1000; }

double ApiDevice::GetLegCurrent4()  // 右前腿电流
{
    return ((double)hardwareAPI_.GetLegCurrent4().data) / 1000;
}

double ApiDevice::GetLegCurrentMax()  // 两条前腿一段时间内最大电流
{
    return ((double)hardwareAPI_.GetLegCurrentMax().data) / 1000;
}

double ApiDevice::GetLegCurrentMax2() { return ((double)hardwareAPI_.GetLegCurrentMax2().data) / 1000; }

TimestampedData<s16> ApiDevice::GetSmallBMSTemperatureMax() { return hardwareAPI_.GetSmallBMSTemperatureMax(); }

double ApiDevice::GetSmallBMSVoltage() { return hardwareAPI_.GetSmallBMSVoltage().data; }

double ApiDevice::GetBC() { return ((double)hardwareAPI_.GetBC().data); }

double ApiDevice::GetBCM() { return ((double)hardwareAPI_.GetBCM().data); }

std::vector<double> ApiDevice::GetSmallBMSCellVoltage()
{
    std::vector<double> ret;
    auto temp = hardwareAPI_.GetSmallBMSCellVoltage();

    for (size_t i = 0; i < temp.size(); i++) {
        ret.push_back(temp[i].data);
    }

    return ret;
}

u16 ApiDevice::GetQuantity() { return hardwareAPI_.GetQuantity().data; }

PowerOffReason ApiDevice::GetLastPowerOutageReason() { return hardwareAPI_.GetLastPowerOutageReason(); }

PowerOffReason ApiDevice::GetPenultimatePowerOutageReason() { return hardwareAPI_.GetPenultimatePowerOutageReason(); }

PowerOffReason ApiDevice::GetAntepenultimatePowerOutageReason() { return hardwareAPI_.GetAntepenultimatePowerOutageReason(); }

bool ApiDevice::CanIctrl(int level)
{
    if (level <= level_) {
        return true;
    }
    return false;
}

bool ApiDevice::AccessCtrlPermission(int level)
{
    if (level <= level_) {
        level_ = level;
        return true;
    }
    return false;
}

bool ApiDevice::ReleaseCtrlPermission(int level)
{
    if (level == level_) {
        level_ = CTRL_LOWEST_LEVEL;
        return true;
    }
    return false;
}

RobotState ApiDevice::GetRobotState() const { return GetRobotCurState(); }

DeviceInfo ApiDevice::GetDeviceInfo()
{
    DeviceInfo info;

    info.version.software = GetDevParam().version.software;
    info.version.sdk = GetDevParam().version.sdk;

    auto cfgStr = GetRobotConfigDirect<string>("ctrModel");
    if (cfgStr) {
        info.model = cfgStr.value();
    }

    string noInit = "noInit/read fail";
    auto nameStr = GetDevCustomParam().ReadRobotName();
    if (nameStr.first == true) {
        info.name = nameStr.second;
    } else {
        info.name = noInit;
    }

    auto snStr = GetDevCustomParam().ReadSerialNo();
    if (snStr.first == true) {
        info.serialNo = snStr.second;
    } else {
        info.serialNo = noInit;
    }

    auto hwStr = GetDevCustomParam().ReadHardwareVer();
    if (hwStr.first == true) {
        info.version.hardware = hwStr.second;
    } else {
        info.version.hardware = noInit;
    }

    auto mcStr = GetDevCustomParam().ReadMechanicalVer();
    if (mcStr.first == true) {
        info.version.mechanical = mcStr.second;
    } else {
        info.version.mechanical = noInit;
    }

    return info;
}

void ApiDevice::Shutdown()
{
    LOG_CRITICAL("os will be shutdown in 3s...");
    AddMediaPackage(MediaTaskPackage("shutDown"));
    TimerTools::SleepForS(3);
    auto ret = system("shutdown -h now");  // 关机，-r参数是重启
    (void)ret;
}

void ApiDevice::SetRobotName(const std::string &name)
{
    if (name.size() > 0) {
        GetDevCustomParam().WriteRobotName(name);
    }
}

void ApiDevice::SetRobotSerialNo(const std::string &serialNo)
{
    if (serialNo.size() > 0) {
        GetDevCustomParam().WriteSerialNo(serialNo);
    }
}

void ApiDevice::SetRobotDescription(const std::string &description)
{
    if (description.size() > 0) {
        GetDevCustomParam().WriteRobotDescription(description);
    }
}

void ApiDevice::SetMotorInit(bool set)
{
    if (set) {
        GetDevCustomParam().WriteInitFlag(1);
    } else {
        GetDevCustomParam().WriteInitFlag(0);
    }
}

std::string ApiDevice::GetRobotDescription()
{
    auto desc = GetDevCustomParam().ReadRobotDescription();
    if (desc.first == true) {
        return desc.second;
    } else {
        return "";
    }
}