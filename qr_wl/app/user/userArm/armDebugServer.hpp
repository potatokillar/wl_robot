
#pragma once

#include "deviceDebugServer.hpp"

constexpr int ARM_DEBUG_AXIS_SIZE = 7;  // 不管算法是几轴，都按7轴处理，多余填0
struct ArmDebugDataPacket
{
    BinPacketHeader armHead{"arm", sizeof(std::array<msg::mit_motor_cmd, ARM_DEBUG_AXIS_SIZE>) + sizeof(std::array<msg::mit_motor_ret, ARM_DEBUG_AXIS_SIZE>)};
    std::array<msg::mit_motor_cmd, ARM_DEBUG_AXIS_SIZE> armCmd;
    std::array<msg::mit_motor_ret, ARM_DEBUG_AXIS_SIZE> armRet;
    BinPacketHeader arm2Head{"armTeach", sizeof(msg::net::arm_teachs)};
    msg::net::arm_teachs armTeachs;
};

class ArmDebugServer : public DeviceDebugServer
{
public:
    void SaveArmData(const msg::arm_cmd &cmd);
    void SaveArmData(const msg::arm_data &ret);

    void SaveArmTeachCmd(const Vec6<double> &cmd);
    void SaveArmTeachRet(const msg::arm_teach &ret);

    std::pair<bool, const std::vector<uint8_t> &> GetData() override;

private:
    ArmDebugDataPacket arm_debug_packet_;
    std::vector<uint8_t> arm_data_;
};
