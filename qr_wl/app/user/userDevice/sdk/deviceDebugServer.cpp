#include "deviceDebugServer.hpp"

using namespace ::std;

DeviceDebugServer::DeviceDebugServer() {}

void DeviceDebugServer::SaveRpyCmd(const msg::imu_data& data)
{
    for (int i = 0; i < 3; i++) {
        device_debug_packet_.imuData.cmd.acc[i] = data.acc[i];
        device_debug_packet_.imuData.cmd.ang[i] = data.ang[i];
        device_debug_packet_.imuData.cmd.gyro[i] = data.gyro[i];
        device_debug_packet_.imuData.cmd.quat[i] = data.quat[i];
    }
    device_debug_packet_.imuData.cmd.quat[3] = data.quat[3];
}

void DeviceDebugServer::SaveImuData(const msg::imu_data& data)
{
    for (int i = 0; i < 3; i++) {
        device_debug_packet_.imuData.ret.acc[i] = data.acc[i];
        device_debug_packet_.imuData.ret.ang[i] = data.ang[i];
        device_debug_packet_.imuData.ret.gyro[i] = data.gyro[i];
        device_debug_packet_.imuData.ret.quat[i] = data.quat[i];
    }
    device_debug_packet_.imuData.ret.quat[3] = data.quat[3];
}

void DeviceDebugServer::SaveDebugData(const msg::watch_data& data) { device_debug_packet_.debugData = data; }

void DeviceDebugServer::SaveDebugDataA(const msg::watch_data_a& data)
{
    for (int i = 0; i < 3; i++) {
        device_debug_packet_.debugData.a[i] = data.a[i];
        device_debug_packet_.debugData.ra[i] = data.ra[i];
    }
}
void DeviceDebugServer::SaveDebugDataB(const msg::watch_data_b& data)
{
    for (int i = 0; i < 3; i++) {
        device_debug_packet_.debugData.b[i] = data.b[i];
        device_debug_packet_.debugData.rb[i] = data.rb[i];
    }
}
void DeviceDebugServer::SaveDebugDataC(const msg::watch_data_c& data)
{
    for (int i = 0; i < 3; i++) {
        device_debug_packet_.debugData.c[i] = data.c[i];
        device_debug_packet_.debugData.rc[i] = data.rc[i];
    }
}

std::pair<bool, const std::vector<uint8_t>&> DeviceDebugServer::GetDeviceDebugData()
{
    bool hasData = false;

    if (GetRobotCurState() == RobotState::jointCtrl) {
        auto ret = MsgTryRecv<msg::imu_data>("qr::imuData", this);
        if (ret) {
            SaveImuData(ret.value());
            hasData = true;
        }
    } else {
        {
            auto ret = MsgTryRecv<msg::imu_data>("qr::rpyFilter", this);
            if (ret) {
                SaveImuData(ret.value());
                hasData = true;
            }
        }

        {
            auto ret = MsgTryRecv<msg::imu_data>("qr::rpyCmd", this);
            if (ret) {
                SaveRpyCmd(ret.value());
                hasData = true;
            }
        }
    }

    {
        auto ret = MsgTryRecv<msg::watch_data>("qr::debugData", this);
        if (ret) {
            SaveDebugData(ret.value());
            hasData = true;
        }
    }

    {
        auto ret = MsgTryRecv<msg::watch_data_a>("qr::debugDataA", this);
        if (ret) {
            SaveDebugDataA(ret.value());
            hasData = true;
        }
    }
    {
        auto ret = MsgTryRecv<msg::watch_data_b>("qr::debugDataB", this);
        if (ret) {
            SaveDebugDataB(ret.value());
            hasData = true;
        }
    }
    {
        auto ret = MsgTryRecv<msg::watch_data_c>("qr::debugDataC", this);
        if (ret) {
            SaveDebugDataC(ret.value());
            hasData = true;
        }
    }

    if (hasData == true) {
        device_data_ = Struct2Vector(device_debug_packet_);
    }
    return {hasData, device_data_};
}
