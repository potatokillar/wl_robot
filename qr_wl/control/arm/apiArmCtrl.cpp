
#include "apiArmCtrl.hpp"

#include "armNode.hpp"
#include "armTask.hpp"
#include "baseline.hpp"
#include "innerType.hpp"

using namespace std;
using namespace arm;

ApiArmCtrl::ApiArmCtrl()
{
    info_ = make_unique<ArmUserInfo>();
    Reset();
    MsgTryRecv<ArmUserInfo>("arm::pub_info", this);
}

void ApiArmCtrl::Reset() {}

void ApiArmCtrl::Start(const BootArgs& args)
{
    if (GetArmParam().model != ArmModel::unused) {
        ArmBootArgs armArgs;
        armArgs.dt = 0.004;
        GetArmNode().Start(args, armArgs);
        isRun_ = true;
    }
}

void ApiArmCtrl::Stop()
{
    GetArmNode().Stop();
    isRun_ = false;
}

Result<std::function<std::optional<msg::arm_cmd>(msg::arm_data)>> ApiArmCtrl::GetTxRxCallback()
{
    Result<std::function<std::optional<msg::arm_cmd>(msg::arm_data)>> ret;

    if (RpcTrySend("arm::txrx_callback", &ret)) {
        return ret;
    } else {
        ret.first = RetState::timeout;
    }
    return ret;
}
Result<std::function<void(msg::arm_motor_info)>> ApiArmCtrl::GetInfoCallback()
{
    Result<std::function<void(msg::arm_motor_info)>> ret;
    if (RpcTrySend("arm::info_callback", &ret)) {
        return ret;
    } else {
        ret.first = RetState::timeout;
    }
    return ret;
}

void ApiArmCtrl::UpdateInfo()
{
    auto ret = MsgTryRecv<ArmUserInfo>("arm::pub_info", this);
    if (ret) {
        info_ = make_unique<ArmUserInfo>(ret.value());
    }
}

bool ApiArmCtrl::IsRun() { return isRun_; }

Result<u32> ApiArmCtrl::SetMoveJTask(const std::vector<std::vector<double>>& jointEnd, MoveMode moveMode)
{
    double spd = 1.0;  // todo 填写默认值
    double acc = 0.5;
    return SetMoveJTask(jointEnd, moveMode, spd, acc);
}
Result<u32> ApiArmCtrl::SetMoveJTask(const std::vector<std::vector<double>>& jointEnd, MoveMode moveMode, double robSpeed)
{
    double acc = 0.5;
    return SetMoveJTask(jointEnd, moveMode, robSpeed, acc);
}

Result<u32> ApiArmCtrl::SetMoveJTask(const std::vector<std::vector<double>>& jointEnd, MoveMode moveMode, double robSpeed, double acc)
{
    if (jointEnd.size() != 1) {
        return {RetState::noSupport, 0};
    }
    ArmMovePoint movepoint;
    Result<u32> ret;

    movepoint.flag = MoveFlag::movej;
    movepoint.Targetjoint = jointEnd.at(0);  // 前者已判断size，此时必有数据
    movepoint.speed = robSpeed;
    movepoint.acceleration = acc;
    movepoint.movemode = moveMode;
    // movepoint.speedmode = speedmode;
    // movepoint.rapidrate = rapidrate;

    if (RpcTrySend("arm::move", movepoint, &ret) == true) {
        if (ret.first == RetState::ok) {
            return {RetState::ok, ret.second};
        } else if (ret.first == RetState::outRange) {
            cout << "MoveJ out of range" << endl;
            return {RetState::outRange, 0};
        } else if (ret.first == RetState::noSupport) {
            cout << "out of Support" << endl;
            return {RetState::noSupport, 0};
        }
    }
    return {RetState::timeout, 0};
}

Result<u32> ApiArmCtrl::SetMoveLTask(const std::vector<Pose>& posEnd, MoveMode moveMode)
{
    double spd = 1.0;
    double acc = 0.5;
    return SetMoveLTask(posEnd, moveMode, spd, acc);
}
Result<u32> ApiArmCtrl::SetMoveLTask(const std::vector<Pose>& posEnd, MoveMode moveMode, double spd)
{
    double acc = 0.5;
    return SetMoveLTask(posEnd, moveMode, spd, acc);
}
Result<u32> ApiArmCtrl::SetMoveLTask(const vector<Pose>& posEnd, MoveMode moveMode, double robSpeed, double acc)
{
    if (posEnd.size() != 1) {
        return {RetState::noSupport, 0};
    }

    ArmMovePoint movepoint;
    Result<u32> ret;
    movepoint.flag = MoveFlag::movel;
    movepoint.Targetpose = posEnd.at(0);
    movepoint.speed = robSpeed;
    movepoint.acceleration = acc;
    movepoint.movemode = moveMode;

    if (RpcTrySend("arm::move", movepoint, &ret) == true) {
        if (ret.first == RetState::ok) {
            return {RetState::ok, ret.second};
        } else if (ret.first == RetState::outRange) {
            cout << "MoveL out of range" << endl;
            return {RetState::outRange, 0};
        } else if (ret.first == RetState::noSupport) {
            cout << "out of Support" << endl;
            return {RetState::noSupport, 0};
        }
    }
    return {RetState::timeout, 0};
}

RetState ApiArmCtrl::WaitMoveTaskComplete(u32 timeout)
{
    RetState ret;
    return RpcBlockSend("arm::wait_task_complete", timeout, &ret, timeout) ? ret : RetState::timeout;
}

RetState ApiArmCtrl::SetCtrlState(ArmCtrl set)
{
    RetState ret;
    // 这些控制指令统一在armCoreUser接收
    return RpcTrySend("arm::user_ctrl", set, &ret) ? ret : RetState::timeout;
}

ArmState ApiArmCtrl::GetRunState()
{
    if (isRun_ == false) {
        return ArmState::noRun;
    }
    UpdateInfo();
    return info_->state;
}

Result<Pose> ApiArmCtrl::GetToolValue(const string& toolName)
{
    Result<Pose> ret;

    Pose toolValue;
    if (RpcTrySend("arm::toolvalue", toolName, &toolValue)) {
        ret.second = toolValue;
        ret.first = RetState::ok;
    } else {
        ret.first = RetState::timeout;
    }
    return ret;
}

RetState ApiArmCtrl::SetToolValue(const std::string& toolName, const Pose& pose)
{
    RetState ret;
    ArmUserTool tool;
    tool.toolName = toolName;
    tool.toolPose = pose;
    if (RpcTrySend("arm::set_tool_value", tool, &ret)) {
        return RetState::ok;
    }
    return ret;
}

vector<string> ApiArmCtrl::GetToolNameList()
{
    vector<string> toolName;
    RpcTrySend("arm::tool_name_list", &toolName);
    return toolName;
}

RetState ApiArmCtrl::SetToolName(const std::string& toolName)
{
    RetState ret = RetState::timeout;
    RpcTrySend("arm::set_tool_name", toolName, &ret);
    return ret;
}

std::string ApiArmCtrl::GetToolName()
{
    string ret;
    RpcTrySend("arm::get_tool_name", &ret);
    return ret;
}

Pose ApiArmCtrl::GetFkine6(std::vector<double> robotjoint)
{
    Vec6<double> joint;
    ArmMathFun math_;
    auto toolName = GetToolName();
    auto toolValue = GetToolValue(toolName);

    for (int i = 0; i < 6; i++) {
        joint(i) = robotjoint[i];
    }
    Pose POSE = math_.Fkine(joint, toolValue.second);

    return POSE;
}

std::pair<Pose, double> ApiArmCtrl::GetFkine7(std::vector<double> robotjoint)
{
    Vec7<double> joint;
    ArmMathFun math_;
    auto toolName = GetToolName();
    auto toolValue = GetToolValue(toolName);

    for (int i = 0; i < 7; i++) {
        joint(i) = robotjoint[i];
    }
    std::pair<Pose, double> POSE = math_.Fkine_seven(joint, toolValue.second);
    return POSE;
}

std::pair<bool, Vec6<double>> ApiArmCtrl::GetIkine6(Pose FK, vector<double> NowAngle)
{
    auto toolName = GetToolName();
    auto toolValue = GetToolValue(toolName);
    ArmMathFun math_;
    Vec6<double> qRecv;
    for (int i = 0; i < 6; i++) {
        qRecv(i) = NowAngle[i];
    }
    std::pair<bool, Vec6<double>> IK = math_.Ikine(FK, qRecv, toolValue.second);
    return IK;
}

std::pair<bool, Vec7<double>> ApiArmCtrl::GetIkine7(Pose FK, double phi, vector<double> NowAngle)
{
    // auto toolName = GetToolName();
    // auto toolValue = GetToolValue(toolName);
    ArmMathFun math_;
    Vec7<double> qRecv;
    for (int i = 0; i < 7; i++) {
        qRecv(i) = NowAngle[i];
    }
    std::pair<bool, Vec7<double>> IK = math_.IKine_seven(FK, phi, qRecv);
    return IK;
}

vector<double> ApiArmCtrl::GetNowAngle()
{
    UpdateInfo();
    return info_->angle;
}

vector<double> ApiArmCtrl::GetNowVelocity()
{
    UpdateInfo();
    return info_->velocity;
}

Result<ArmUserParam> ApiArmCtrl::GetUserParam()
{
    Result<ArmUserParam> ret;
    ArmUserParam param;
    if (RpcTrySend("arm::get_arm_param", &param)) {
        ret.second = param;
        ret.first = RetState::ok;
    } else {
        ret.first = RetState::timeout;
    }
    return ret;
}

Pose ApiArmCtrl::GetNowCart()
{
    UpdateInfo();
    return info_->cart;
}

std::vector<double> ApiArmCtrl::GetNowCartSpeed()
{
    UpdateInfo();
    return {};
}

/**
 * @description: 设置输出IO的输出状态，该指令直接透传到电机侧，中间不再进行处理
 * @param set
 * @return {}
 */
RetState ApiArmCtrl::SetIoOutState(const std::string& name, std::bitset<32> set)
{
    RetState ret;
    std::pair<string, std::bitset<32>> setPair{name, set};
    return RpcTrySend("arm::user_set_io_out", setPair, &ret) ? ret : RetState::timeout;
}
std::bitset<32> ApiArmCtrl::GetIoInState(const std::string& name)
{
    if (info_->ioInSta.count(name) == 0) {
        return 0;
    }
    UpdateInfo();
    return info_->ioInSta.at(name);
}
std::bitset<32> ApiArmCtrl::GetIoOutState(const std::string& name)
{
    if (info_->ioOutSta.count(name) == 0) {
        return 0;
    }
    UpdateInfo();
    return info_->ioOutSta.at(name);
}

Result<ValueRange<vector<double>>> ApiArmCtrl::GetPositionLimit()
{
    Result<ValueRange<vector<double>>> ret;

    auto param = GetUserParam();
    if (param.first == RetState::ok) {
        ret.second.min = param.second.Smin;
        ret.second.max = param.second.Smax;
        ret.first = RetState::ok;
    } else {
        ret.first = RetState::timeout;
    }
    return ret;
}
RetState ApiArmCtrl::SetPositionLimit(vector<ValueRange<double>> data)
{
    RetState ret = RetState::timeout;
    if (data.size() != 6 && data.size() != 7) {
        ret = RetState::outRange;
        return ret;
    } else {
        ret = RpcTrySend("arm::set_position_limit", data, &ret) ? ret : RetState::timeout;
    }
    return ret;
}
// RetState ApiArmCtrl::SetUserTool(pair<string, Pose> data)
// {
//     RetState ret = RetState::timeout;
//     RpcTrySend("arm::set_user_tool", data, &ret);
//     return ret;
// }
RetState ApiArmCtrl::SetReset(bool Reset)
{
    RetState ret;
    return RpcTrySend("arm::set_reset", Reset, &ret) ? ret : RetState::timeout;
}
Result<bool> ApiArmCtrl::GetReset()
{
    Result<double> ret;
    if (isRun_ == false) {
        ret.first = RetState::noUpdate;
    } else {
        auto param = GetUserParam();
        if (param.first == RetState::ok) {
            ret.first = RetState::ok;
            ret.second = param.second.reset;
        } else {
            ret.first = param.first;
        }
    }
    return ret;
}
RetState ApiArmCtrl::SetRapidrate(double rate)
{
    RetState ret;
    return RpcTrySend("arm::set_rapidrate", rate, &ret) ? ret : RetState::timeout;
}

Result<double> ApiArmCtrl::GetRapidrate()
{
    Result<double> ret;
    if (isRun_ == false) {
        ret.first = RetState::noUpdate;
    } else {
        auto param = GetUserParam();
        if (param.first == RetState::ok) {
            ret.first = param.first;
            ret.second = param.second.rapidrate;
        } else {
            ret.first = param.first;
        }
    }
    return ret;
};

RetState ApiArmCtrl::SetJointSpeedLimit(vector<double> Vmax)
{
    RetState ret;
    if (Vmax.size() != 6 && Vmax.size() != 7) {
        ret = RetState::outRange;
        return ret;
    } else {
        RpcTrySend("arm::set_joint_speed_limit", Vmax, &ret);
        ret = RetState::ok;
    }
    return ret;
}
Result<vector<double>> ApiArmCtrl::GetJointSpeedLimit()
{
    Result<vector<double>> ret;
    if (isRun_ == false) {
        ret.first = RetState::noUpdate;
    } else {
        auto param = GetUserParam();
        if (param.first == RetState::ok) {
            ret.first = param.first;
            ret.second = param.second.Vmax;
            // for (int i = 0; i < param.second.Vmax.size(); i++) {
            //     ret.second.push_back(param.second.Vmax(i));
            // }

        } else {
            ret.first = param.first;
        }
    }
    return ret;
}
RetState ApiArmCtrl::SetJointAccLimit(vector<double> Amax)
{
    RetState ret;
    if (Amax.size() != 6 && Amax.size() != 7) {
        RetState ret = RetState::outRange;
        return ret;
    } else {
        RpcTrySend("arm::set_joint_acc_limit", Amax, &ret);
        ret = RetState::ok;
    }
    return ret;
}
Result<vector<double>> ApiArmCtrl::GetJointAccLimit()
{
    Result<vector<double>> ret;
    if (isRun_ == false) {
        ret.first = RetState::noUpdate;
    } else {
        auto param = GetUserParam();
        if (param.first == RetState::ok) {
            ret.first = param.first;
            ret.second = param.second.Amax;
            // for (int i = 0; i < param.second.Amax.size(); i++) {
            //     ret.second.push_back(param.second.Amax(i));
            // }
        } else {
            ret.first = param.first;
        }
    }
    return ret;
}

vector<double> ApiArmCtrl::GetAxisCurrent()
{
    UpdateInfo();
    return info_->motor.current;
}

vector<double> ApiArmCtrl::GetAxisVoltage()
{
    UpdateInfo();
    return info_->motor.voltage;
}

vector<double> ApiArmCtrl::GetAxisTemperature()
{
    UpdateInfo();
    return info_->motor.temperature;
}

vector<double> ApiArmCtrl::GetAxisTorque()
{
    UpdateInfo();
    return info_->motor.torque;
}

RetState ApiArmCtrl::SetCollisionLevel(u8 level)
{
    (void)level;
    return RetState::noSupport;
}
Result<u8> ApiArmCtrl::GetCollisionLevel() { return {RetState::noSupport, 0}; }

RetState ApiArmCtrl::SetDragMode(bool mode)
{
    (void)mode;
    return RetState::noSupport;
}
Result<bool> ApiArmCtrl::GetDragMode() { return {RetState::noSupport, false}; }

Result<Mat6<double>> ApiArmCtrl::GetJacobianTool6(vector<double> robotjoint)
{
    (void)robotjoint;
    ArmMathFun math_;
    Result<Mat6<double>> JacobianTool6;
    auto nowAngle = GetNowAngle();

    JacobianTool6.second = math_.Jacobian_tool6(nowAngle);
    JacobianTool6.first = RetState::ok;
    return JacobianTool6;
}

Result<Mat6<double>> ApiArmCtrl::GetJacobianBase6(vector<double> robotjoint)
{
    (void)robotjoint;
    ArmMathFun math_;
    Result<Mat6<double>> JacobianBase6;
    auto nowAngle = GetNowAngle();
    JacobianBase6.second = math_.Jacobian_base6(nowAngle);
    JacobianBase6.first = RetState::ok;
    return JacobianBase6;
}