
#include "apiArm.hpp"

#include "apiArmCtrl.hpp"
#include "deviceParam.hpp"
#include "driver.hpp"

using namespace arm;
using namespace std;

ApiArm::~ApiArm() { Stop(); }

void ApiArm::Start()
{
    if (bootArgs.isSim) {
        motor_ = make_shared<ArmMotorGazebo>();
        motor_->Start();
    } else {
        if (GetDevArmParam().ethercat.empty() == false) {
            motor_ = make_shared<ArmDevEcat>(GetDevArmParam().ethercat);
            motor_->SetParam(ArmMotorParamType::circleCnt, GetDevArmParam().motorCfg[0].circleCnt);  // 目前所有电机公用一个cnt
            motor_->Start();
        } else if (GetDevArmParam().motorCfg.size() > 0) {
            auto canDev = make_unique<MitCanInterface>(GetDevArmParam().spiName, GetDevArmParam().spiSpeed);
            motor_ = make_shared<ArmMotorMit>(GetDevArmParam().motorCfg, std::move(canDev));
            motor_->Start();
        } else {
            LOG_ERROR("no Arm motor config, can't init");
            return;
        }
    }

    api_.Start(bootArgs);  // 程序启动，算法同时启动
    {
        auto func = api_.GetTxRxCallback();
        if (func.first == RetState::ok) {
            motor_->SetTxRxCallback(func.second);
        }
    }
    {
        auto func = api_.GetInfoCallback();

        if (func.first == RetState::ok) {
            std::cout << "set info" << std::endl;
            motor_->SetInfoCallback(func.second);
        }
    }
}
void ApiArm::Stop() { api_.Stop(); }

bool ApiArm::IsRun() { return api_.IsRun(); }

RetState ApiArm::WaitMoveTaskComplete(u32 timeout) { return api_.WaitMoveTaskComplete(timeout); }

/**
 * @description: 设置任务状态
 * @param set
 * @return {}
 */
RetState ApiArm::SetCtrlState(arm::ArmCtrl set)
{
    RetState ret = RetState::noSupport;
    switch (set) {
        case ArmCtrl::start:
            ret = api_.SetCtrlState(ArmCtrl::start);
            break;
        case ArmCtrl::stop:
            ret = api_.SetCtrlState(ArmCtrl::stop);
            break;
        case ArmCtrl::pause:
            ret = api_.SetCtrlState(ArmCtrl::pause);
            break;
        case ArmCtrl::enable:
            ret = api_.SetCtrlState(ArmCtrl::enable);
            break;
        case ArmCtrl::disable:
            ret = api_.SetCtrlState(ArmCtrl::disable);
            break;
        default:
            break;
    }
    // std::cout << "set ctrl" << std::endl;
    return ret;
}

/**
 * @description: 根据工具名字获取具体的工具坐标系坐标
 * @param toolName
 * @return {}
 */
Result<Pose> ApiArm::GetToolValue(std::string toolName) { return api_.GetToolValue(toolName); }
RetState ApiArm::SetToolValue(const std::string& toolName, const Pose& pose) { return api_.SetToolValue(toolName, pose); }

/**
 * @description: 获取工具坐标系列表
 * @return {} 若机械臂算法异常，则无法获取到
 */
std::vector<std::string> ApiArm::GetToolNameList() { return api_.GetToolNameList(); }

RetState ApiArm::SetToolName(const std::string& name) { return api_.SetToolName(name); }

std::string ApiArm::GetToolName() { return api_.GetToolName(); }

/**
 * @brief 关节规划
 *
 * @param jointEnd 目标关节位置
 * @param robSpeed 运行速率：0-1；0为最慢；1为最快；
 */
Result<u32> ApiArm::SetMoveJTaskNoBlock(const std::vector<std::vector<double>>& jointEnd, MoveMode movemode) { return api_.SetMoveJTask(jointEnd, movemode); }
Result<u32> ApiArm::SetMoveJTaskNoBlock(const std::vector<std::vector<double>>& jointEnd, MoveMode movemode, double spd) { return api_.SetMoveJTask(jointEnd, movemode, spd); }
Result<u32> ApiArm::SetMoveJTaskNoBlock(const std::vector<std::vector<double>>& jointEnd, MoveMode movemode, double robSpeed, double acc)
{
    return api_.SetMoveJTask(jointEnd, movemode, robSpeed, acc);
}
/**
 * @brief 笛卡尔直线规划
 *
 * @param poseEnd 目标空间位置
 * @param robSpeed 运行速率：0-1；0为最慢；1为最快；
 */
Result<u32> ApiArm::SetMoveLTaskNoBlock(const std::vector<Pose>& posEnd, MoveMode movemode) { return api_.SetMoveLTask(posEnd, movemode); }
Result<u32> ApiArm::SetMoveLTaskNoBlock(const std::vector<Pose>& posEnd, MoveMode movemode, double spd) { return api_.SetMoveLTask(posEnd, movemode, spd); }
Result<u32> ApiArm::SetMoveLTaskNoBlock(const std::vector<Pose>& posEnd, MoveMode movemode, double robSpeed, double acc)
{
    return api_.SetMoveLTask(posEnd, movemode, robSpeed, acc);
}

RetState ApiArm::SetMoveJTaskBlock(const std::vector<double>& jointEnd, MoveMode movemode, double robSpeed, double acc)
{
    vector<std::vector<double>> jointEnds;
    jointEnds.push_back(jointEnd);
    auto timeout = SetMoveJTaskNoBlock(jointEnds, movemode, robSpeed, acc);

    SetCtrlState(ArmCtrl::start);
    if (timeout.first == RetState::ok) {
        return WaitMoveTaskComplete(timeout.second);
    }
    return RetState::error;
}
RetState ApiArm::SetMoveLTaskBlock(const Pose& posEnd, MoveMode movemode, double robSpeed, double acc)
{
    vector<Pose> posEnds;
    posEnds.push_back(posEnd);
    auto timeout = SetMoveLTaskNoBlock(posEnds, movemode, robSpeed, acc);
    SetCtrlState(ArmCtrl::start);
    if (timeout.first == RetState::ok) {
        return WaitMoveTaskComplete(timeout.second);
    }
    return RetState::error;
}

Pose ApiArm::GetFkine6(vector<double> robotjoint) { return api_.GetFkine6(robotjoint); }
std::pair<bool, Vec6<double>> ApiArm::GetIkine6(Pose FK, vector<double> NowAngle) { return api_.GetIkine6(FK, NowAngle); }

std::pair<Pose, double> ApiArm::GetFkine7(vector<double> robotjoint) { return api_.GetFkine7(robotjoint); }
std::pair<bool, Vec7<double>> ApiArm::GetIkine7(Pose FK, double phi, vector<double> NowAngle) { return api_.GetIkine7(FK, phi, NowAngle); }

Result<Mat6<double>> ApiArm::GetJacobianTool6(std::vector<double> robotjoint) { return api_.GetJacobianTool6(robotjoint); }
Result<Mat6<double>> ApiArm::GetJacobianBase6(std::vector<double> robotjoint) { return api_.GetJacobianBase6(robotjoint); }

/**
 * @brief 获取当前的关节角
 *
 * @return Result<Vec6<double>>
 */
vector<double> ApiArm::GetNowAngle() { return api_.GetNowAngle(); }

vector<double> ApiArm::GetNowVelocity() { return api_.GetNowVelocity(); }

Pose ApiArm::GetNowCart() { return api_.GetNowCart(); }

vector<double> ApiArm::GetNowCartSpeed() { return api_.GetNowCartSpeed(); }
arm::ArmState ApiArm::GetArmState() { return api_.GetRunState(); }

RetState ApiArm::SetIoOutState(const std::string& name, std::bitset<32> set) { return api_.SetIoOutState(name, set); }
std::bitset<32> ApiArm::GetIoInState(const std::string& name) { return api_.GetIoInState(name); }
std::bitset<32> ApiArm::GetIoOutState(const std::string& name) { return api_.GetIoOutState(name); }

Result<vector<ValueRange<double>>> ApiArm::GetPositionLimit()
{
    Result<vector<ValueRange<double>>> ret;

    auto apiRet = api_.GetPositionLimit();
    ret.second.resize(apiRet.second.min.size());
    if (apiRet.first == RetState::ok) {
        for (size_t i = 0; i < apiRet.second.max.size(); i++) {
            ret.second[i].min = apiRet.second.min[i];
            ret.second[i].max = apiRet.second.max[i];
        }
        ret.first = RetState::ok;
    } else {
        ret.first = apiRet.first;
    }

    return ret;
}

RetState ApiArm::SetPositionLimit(vector<ValueRange<double>> data) { return api_.SetPositionLimit(data); }

// RetState ApiArm::SetUserTool(pair<string, Pose> data) { return api_.SetUserTool(data); }
RetState ApiArm::SetReset(bool Reset) { return api_.SetReset(Reset); }

Result<bool> ApiArm::GetReset() { return api_.GetReset(); }
RetState ApiArm::SetRapidrate(double rate) { return api_.SetRapidrate(rate); }
Result<double> ApiArm::GetRapidrate() { return api_.GetRapidrate(); }

RetState ApiArm::SetJointSpeedLimit(vector<double> Vmax) { return api_.SetJointSpeedLimit(Vmax); }
Result<vector<double>> ApiArm::GetJointSpeedLimit() { return api_.GetJointSpeedLimit(); }

RetState ApiArm::SetJointAccLimit(std::vector<double> Amax) { return api_.SetJointAccLimit(Amax); }
Result<vector<double>> ApiArm::GetJointAccLimit() { return api_.GetJointAccLimit(); }

std::vector<double> ApiArm::GetAxisCurrent() { return api_.GetAxisCurrent(); }
std::vector<double> ApiArm::GetAxisVoltage() { return api_.GetAxisVoltage(); }
std::vector<double> ApiArm::GetAxisTemperature() { return api_.GetAxisTemperature(); }
std::vector<double> ApiArm::GetAxisTorque() { return api_.GetAxisTorque(); }

RetState ApiArm::SetCollisionLevel(u8 level) { return api_.SetCollisionLevel(level); }
Result<u8> ApiArm::GetCollisionLevel() { return api_.GetCollisionLevel(); }

RetState ApiArm::SetDragMode(bool mode) { return api_.SetDragMode(mode); }
Result<bool> ApiArm::GetDragMode() { return api_.GetDragMode(); }

RetState ApiArm::SetGripper(bool state, double speed)
{
    if (state == true) {
        // cout << "gripper on" << endl;
        gripperAPI_.AotuGetGripperOn(speed);
    } else if (state == false) {
        // cout << "gripper off" << endl;
        gripperAPI_.AotuGetGripperOff(speed);
    }
    return RetState::ok;
}

Result<Vec6<double>> ApiArm::GetJacobian()
{
    Result<Vec6<double>> ret;
    return ret;
}
