
#include "quadDebugServer.hpp"

/**
 * @description: 普通四足，轮子不填充数据
 * @param data
 * @return {}
 */
void QuadDebugServer::SaveMotorCmd(const msg::qr::motor_cmd& data)
{
    for (int leg = 0; leg < 4; leg++) {
        quad_debug_packet_.legCmd.leg[leg][0].alpha = data.leg[leg][0].alpha;
        quad_debug_packet_.legCmd.leg[leg][1].alpha = data.leg[leg][1].alpha;
        quad_debug_packet_.legCmd.leg[leg][2].alpha = data.leg[leg][2].alpha;

        quad_debug_packet_.legCmd.leg[leg][0].torq = data.leg[leg][0].torq;
        quad_debug_packet_.legCmd.leg[leg][1].torq = data.leg[leg][1].torq;
        quad_debug_packet_.legCmd.leg[leg][2].torq = data.leg[leg][2].torq;

        quad_debug_packet_.legCmd.leg[leg][0].blta = data.leg[leg][0].blta;
        quad_debug_packet_.legCmd.leg[leg][1].blta = data.leg[leg][1].blta;
        quad_debug_packet_.legCmd.leg[leg][2].blta = data.leg[leg][2].blta;
    }
}

void QuadDebugServer::SaveMotorRet(const msg::qr::motor_ret& data)
{
    for (int leg = 0; leg < 4; leg++) {
        quad_debug_packet_.legRet.leg[leg][0].alpha = data.leg[leg][0].alpha;
        quad_debug_packet_.legRet.leg[leg][1].alpha = data.leg[leg][1].alpha;
        quad_debug_packet_.legRet.leg[leg][2].alpha = data.leg[leg][2].alpha;

        quad_debug_packet_.legRet.leg[leg][0].torq = data.leg[leg][0].torq;
        quad_debug_packet_.legRet.leg[leg][1].torq = data.leg[leg][1].torq;
        quad_debug_packet_.legRet.leg[leg][2].torq = data.leg[leg][2].torq;

        quad_debug_packet_.legRet.leg[leg][0].blta = data.leg[leg][0].blta;
        quad_debug_packet_.legRet.leg[leg][1].blta = data.leg[leg][1].blta;
        quad_debug_packet_.legRet.leg[leg][2].blta = data.leg[leg][2].blta;

        quad_debug_packet_.legRet.leg[leg][0].sta = data.leg[leg][0].sta;
        quad_debug_packet_.legRet.leg[leg][1].sta = data.leg[leg][1].sta;
        quad_debug_packet_.legRet.leg[leg][2].sta = data.leg[leg][2].sta;
    }
}

void QuadDebugServer::SaveMotorWheelCmd(const msg::wheel::motor_cmd& data) { quad_debug_packet_.legCmd = data; }
void QuadDebugServer::SaveMotorWheelRet(const msg::wheel::motor_ret& data) { quad_debug_packet_.legRet = data; }

std::pair<bool, const std::vector<uint8_t>&> QuadDebugServer::GetData()
{
    bool hasData = false;

    // 先生成基类的公用debug数据的字符数组，存储在基类的device_data_对象中
    auto [ret, vData] = GetDeviceDebugData();
    hasData = ret;

    {
        auto ret = MsgTryRecv<msg::qr::motor_cmd>("qr::motor_cmd", this);
        if (ret) {
            SaveMotorCmd(ret.value());
            hasData = true;
        }
    }

    {
        auto ret = MsgTryRecv<msg::qr::motor_ret>("qr::motor_ret", this);
        if (ret) {
            SaveMotorRet(ret.value());
            hasData = true;
        }
    }

    {
        auto ret = MsgTryRecv<msg::qr::motor_cmd>("qr::motor_cmd", this);
        if (ret) {
            SaveMotorCmd(ret.value());
            hasData = true;
        }
    }

    {
        auto ret = MsgTryRecv<msg::qr::motor_ret>("qr::motor_ret", this);
        if (ret) {
            SaveMotorRet(ret.value());
            hasData = true;
        }
    }

    {
        auto ret = MsgTryRecv<msg::wheel::motor_cmd>("wheel::motor_cmd", this);
        if (ret) {
            SaveMotorWheelCmd(ret.value());
            hasData = true;
        }
    }

    {
        auto ret = MsgTryRecv<msg::wheel::motor_ret>("wheel::motor_ret", this);
        if (ret) {
            SaveMotorWheelRet(ret.value());
            hasData = true;
        }
    }

    if (hasData == true) {
        quad_data_ = Struct2Vector(quad_debug_packet_);
        quad_data_.insert(quad_data_.end(), device_data_.begin(), device_data_.end()); /** @todo 2024.1115 - 这不分在3种业务中重复出现，可以在基类写一个函数来实现 */
    }
    return {hasData, quad_data_};
}