
#include "armDebugServer.hpp"

void ArmDebugServer::SaveArmData(const msg::arm_cmd &cmd)
{
    for (int i = 0; i < ARM_DEBUG_AXIS_SIZE; i++) {
        arm_debug_packet_.armCmd[i].alpha = cmd.motor[i].alpha;
        arm_debug_packet_.armCmd[i].torq = cmd.motor[i].torq;
        arm_debug_packet_.armCmd[i].blta = cmd.motor[i].blta;
        arm_debug_packet_.armCmd[i].k2 = cmd.motor[i].k2;
        arm_debug_packet_.armCmd[i].k1 = cmd.motor[i].k1;
    }
}

void ArmDebugServer::SaveArmData(const msg::arm_data &ret)
{
    for (int i = 0; i < ARM_DEBUG_AXIS_SIZE; i++) {
        arm_debug_packet_.armRet[i].alpha = ret.motor[i].alpha;
        arm_debug_packet_.armRet[i].torq = ret.motor[i].torq;
        arm_debug_packet_.armRet[i].blta = ret.motor[i].blta;
    }
}

void ArmDebugServer::SaveArmTeachCmd(const Vec6<double> &cmd)
{
    arm_debug_packet_.armTeachs.cmd.x = cmd(0);
    arm_debug_packet_.armTeachs.cmd.y = cmd(1);
    arm_debug_packet_.armTeachs.cmd.z = cmd(2);
    arm_debug_packet_.armTeachs.cmd.Rx = cmd(3);
    arm_debug_packet_.armTeachs.cmd.Ry = cmd(4);
    arm_debug_packet_.armTeachs.cmd.Rz = cmd(5);
}

void ArmDebugServer::SaveArmTeachRet(const msg::arm_teach &ret) { arm_debug_packet_.armTeachs.ret = ret; }

std::pair<bool, const std::vector<uint8_t> &> ArmDebugServer::GetData()
{
    bool hasData = false;
    // 先生成基类的公用debug数据的字符数组，存储在基类的device_data_对象中
    auto [ret, vData] = GetDeviceDebugData();
    hasData = ret;

    {
        auto ret = MsgTryRecv<msg::arm_cmd>("arm::arm_cmd", this);
        if (ret) {
            SaveArmData(ret.value());
            hasData = true;
        }
    }

    {
        auto ret = MsgTryRecv<msg::arm_data>("arm::arm_data", this);
        if (ret) {
            SaveArmData(ret.value());
            hasData = true;
        }
    }

    {
        auto ret = MsgTryRecv<Vec6<double>>("arm::arm_teach_cmd", this);
        if (ret) {
            SaveArmTeachCmd(ret.value());
            hasData = true;
        }
    }

    {
        auto ret = MsgTryRecv<msg::arm_teach>("arm::arm_teach_ret", this);
        if (ret) {
            SaveArmTeachRet(ret.value());
            hasData = true;
        }
    }

    if (hasData == true) {
        arm_data_ = Struct2Vector(arm_debug_packet_);
        arm_data_.insert(arm_data_.end(), device_data_.begin(), device_data_.end());
    }
    return {hasData, arm_data_};
}