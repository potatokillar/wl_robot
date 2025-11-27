
#include "armMotorGazebo.hpp"

#include "baseline.hpp"

using namespace std;

/**
 * @description:
 * @return {}
 */
ArmMotorGazebo::ArmMotorGazebo()
{
    MsgTryRecv<bool>("arm::enable", this);
    MsgTryRecv<std::pair<std::string, std::bitset<32>>>("arm::set_io", this);
    motorCmd_inited_ = false;
}

ArmMotorGazebo::~ArmMotorGazebo() {}

void ArmMotorGazebo::Enable()
{
    if (motorInfos_.state != msg::arm_motor_state::enable) {
        motorInfos_.state = msg::arm_motor_state::enable;
        cout << "enable" << endl;
        infoIsUpdate_ = true;
    }
}

void ArmMotorGazebo::Disable()
{
    if (motorInfos_.state != msg::arm_motor_state::disable) {
        motorInfos_.state = msg::arm_motor_state::disable;
        infoIsUpdate_ = true;
    }
}

void ArmMotorGazebo::Start()
{
    task_ = make_unique<PeriodicMemberFunction<ArmMotorGazebo>>("armMotor", 0.004, this, &ArmMotorGazebo::Run, false);
    task_->Start();
}

void ArmMotorGazebo::Stop() { task_->Stop(); }

void ArmMotorGazebo::Run()
{
    if (auto ret = MsgTryRecv<bool>("arm::enable", this)) {
        if (ret.value() == true) {
            Enable();
        } else {
            Disable();
        }
    }
    UpdateBasicData();
    RxArmCmd();
    RecvMotorRet();
    SendMotorCmd();
}

/**
 * @description: 发送电机命令
 * @return {}
 */
void ArmMotorGazebo::SendMotorCmd()
{
    if (motorInfos_.state != msg::arm_motor_state::enable) {
        return;
    }

    // 和真实电机不同，在gazebo电机中，该命令是同时发给qrChart和gazebo
    if (!motorCmd_inited_)
        // 在电机指令正确初始化之前，直接return不send电机指令，否者可能执行错误指令
        return;
    MsgTrySend("arm::arm_cmd", motorCmd_);
}

/**
 * @description: 接收电机返回
 * 和真实电机不同，这里的返回，来自于gazebo消息
 * @return {}
 */
void ArmMotorGazebo::RecvMotorRet()
{
    auto ret = MsgTryRecv<msg::arm_data>("arm::arm_data", this);
    if (ret) {
        motorRet_ = ret.value();
    }
}

/**
 * @description: 接收算法的IO控制指令
 * @param in
 * @return {}
 */
void ArmMotorGazebo::SetIoOut(const std::string& name, std::bitset<32> set)
{
    if (set != motorInfos_.ioOutSta[name]) {
        // cout << "set io out " << name << " " << set << endl;
        infoIsUpdate_ = true;
        motorInfos_.ioOutSta[name] = set;  // 仿真直接保存
    }
}

std::bitset<32> ArmMotorGazebo::GetIoOut(const std::string& name) { return motorInfos_.ioOutSta[name]; }

void ArmMotorGazebo::RxArmCmd()
{
    if (auto ret = MsgTryRecv<bool>("arm::enable", this)) {
        if (ret.value() == true) {
            Enable();
        } else {
            Disable();
        }
    }
    if (auto ret = MsgTryRecv<std::pair<std::string, std::bitset<32>>>("arm::set_io", this)) {
        SetIoOut(ret.value().first, ret.value().second);
    }
}
