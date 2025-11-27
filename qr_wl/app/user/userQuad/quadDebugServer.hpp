
#pragma once

#include "deviceDebugServer.hpp"

struct QuadDebugDataPacket
{
    BinPacketHeader legHead{"leg", sizeof(msg::wheel::motor_cmd) + sizeof(msg::wheel::motor_ret)};
    msg::wheel::motor_cmd legCmd;
    msg::wheel::motor_ret legRet;
};

/**
 * @description: 四足业务中，除了公用的debug数据之外，还有一些四足特有的debug数据，在派生类的同名虚方法中，实现这部分数据的收集、转换为字符数组的功能
 * @return {}
 */
class QuadDebugServer : public DeviceDebugServer
{
public:
    void SaveMotorCmd(const msg::qr::motor_cmd &data);
    void SaveMotorRet(const msg::qr::motor_ret &data);
    void SaveMotorWheelCmd(const msg::wheel::motor_cmd &data);
    void SaveMotorWheelRet(const msg::wheel::motor_ret &data);

    std::pair<bool, const std::vector<uint8_t> &> GetData() override;

private:
    QuadDebugDataPacket quad_debug_packet_;
    std::vector<uint8_t> quad_data_;
};