
#include "armCore.hpp"

#include "armParam.hpp"

using namespace std;
using namespace arm;
ArmCore::ArmCore(const ArmBootArgs& armArgs) : dt_(armArgs.dt)
{
    coData_ = make_shared<CoreData>();
    coData_->armSta = ArmState::init;
    ArmParam();
    // 均来自用户侧的指令，非阻塞
    rpcRxDeal_.emplace_back("arm::tool_name_list", [this](MsgType in) { return this->RxGetToolNameList(in); });
    rpcRxDeal_.emplace_back("arm::set_tool_name", [this](MsgType in) { return this->RxSetToolName(in); });
    rpcRxDeal_.emplace_back("arm::get_tool_name", [this](MsgType in) { return this->RxGetToolName(in); });
    rpcRxDeal_.emplace_back("arm::toolvalue", [this](MsgType in) { return this->RxGetToolValue(in); });
    rpcRxDeal_.emplace_back("arm::user_ctrl", [this](MsgType in) { return this->RxUserCtrl(in); });
    rpcRxDeal_.emplace_back("arm::get_arm_param", [this](MsgType in) { return this->RxGetArmParam(in); });
    rpcRxDeal_.emplace_back("arm::user_set_io_out", [this](MsgType in) { return this->RxSetIoOut(in); });
    // rpcRxDeal_.emplace_back("arm::move", [this](MsgType set) { return this->RxMoveSearch(set); });   // 6 7不同
    rpcRxDeal_.emplace_back("arm::set_reset", [this](MsgType in) { return this->RxSetReset(in); });
    // rpcRxDeal_.emplace_back("arm::set_position_limit", [this](MsgType set) { return this->RxSetPositionLimit(set); });
    // rpcRxDeal_.emplace_back("arm::get_position_limit", [this]() { return this->RxGetPositionLimit(); });

    // rpcRxDeal_.emplace_back("arm::set_user_tool", [this](MsgType set) { return this->RxSetUserTool(set); });
    // rpcRxDeal_.emplace_back("arm::txrx_callback", [this](MsgType set) { return this->RxTxRxCallback(set); });
    // rpcRxDeal_.emplace_back("arm::info_callback", [this](MsgType set) { return this->RxInfoCallback(set); });

    rpcRxDeal_.emplace_back("arm::set_tool_value", [this](MsgType set) { return this->RxSetToolValue(set); });

    rpcRxDeal_.emplace_back("arm::set_rapidrate", [this](MsgType set) { return this->RxSetRapidRate(set); });

    for (auto& rpc : rpcRxDeal_) {
        rpc.Connect();
    }
    coData_->armSta = ArmState::noReady;
}

void ArmCore::Run()
{
    for (auto& rpc : rpcRxDeal_) {
        rpc.Run();
    }
    ExecTaskCmd();
    UpdateArmState();
}

void ArmCore::ArmParam()
{
    user_.Smax = GetArmParam().axis.S_max_user;
    user_.Smin = GetArmParam().axis.S_min_user;
    user_.Vmax = GetArmParam().axis.V_max;
    user_.Amax = GetArmParam().axis.A_max;
    user_.userTool = GetArmParam().coor.tool;
    user_.rapidrate = rapidrate_;
    user_.reset = reset_;
}

/**
 * @description: core上报机械臂用户设置的参数
 */
MsgType ArmCore::RxGetArmParam(const MsgType& in)
{
    (void)in;
    return user_;
}

/**
 * @description: 接收来自用户侧的控制指令
 * @param &in
 * @return {} 必成功
 */
MsgType ArmCore::RxUserCtrl(const MsgType& in)
{
    auto cmd = in.GetType<ArmCtrl>();
    switch (cmd) {
        case ArmCtrl::enable:
            // LOG_INFO_ARM("enable motor");
            // motor_->Enable();
            MsgTrySend("arm::enable", true);
            break;
        case ArmCtrl::disable:
        case ArmCtrl::emgStop:
            // LOG_INFO_ARM("disable motor");
            // motor_->Disable();
            MsgTrySend("arm::enable", false);
            break;
        default:
            // 必须前一个命令执行完毕
            //  if (moveData_.cmd.has_value() == false) {
            moveData_.cmd = cmd;  // 无需上一个命令
            //  } else {
            //      LOG_WARN_ARM("last cmd no exec complete");
            //     return RetState::busy;
            //  }

            break;
    }
    // 需要删除之前的arm::task_exec上报，因为部分情况下，RxWaitTaskComplete不会调用。
    // todo msg接口临时方案，等待msg优化后再次修改
    MsgSend msg("arm::task_exec");
    msg.Clear();
    return RetState::ok;
}
/**
 * @description: 改变状态
 * @param sta
 * @return {}
 */
void ArmCore::SetArmState(arm::ArmState sta)
{
    // cout << "SetArmState:" << Enum2Num(sta) << endl;
    if (coData_->armSta != sta) {
        lastStateChange_ = TimerTools::GetNowTickMs();
        coData_->armSta = sta;
        LOG_DEBUG_ARM("SetArmState:{}", Enum2Num(sta));
    }
}

/**
 * @description: 获取状态
 * @return {}
 */
ArmState ArmCore::GetArmState() { return coData_->armSta; }

RetState ArmCore::RxSetReset(const MsgType& in)
{
    user_.reset = in.GetType<bool>();
    return RetState::ok;
}

/**
 * @description: 接收阻塞任务
 * @param in
 * @todo 关节规划根据数据长度判断，笛卡尔规划改为根据型号判断
 * @return {}
 */
// MsgType UserCore::RxMoveSearch(const MsgType& in) { return armcore_->MoveSearch(in); }

RetState ArmCore::RxSetRapidRate(const MsgType& in)
{
    RetState ret;
    auto set = in.GetType<double>();

    ret = math_.Checkrapid(set);
    if (ret == RetState::ok) {
        user_.rapidrate = set;
    } else {
        user_.rapidrate = 0.5;
    }
    return ret;
}

/**
 * @description: motor其他信息上报回调
 * @param info
 * @return {}
 */
void ArmCore::InfoCallback(const msg::arm_motor_info& info)
{
    // lock_guard lock(mtx_);
    coData_->motorInfo = info;
    // LOG_DEBUG_ARM("rx motor state: {}", Enum2Num(info.state));
}

/**
 * @description: 接收来自用户侧的请求
 * @param in
 * @return {}
 */
MsgType ArmCore::RxGetToolValue(const MsgType& in)
{
    string toolName = in.GetType<string>();
    Pose ToolValue;
    const auto& tool = user_.userTool;
    auto it = tool.find(toolName);
    if (it != tool.end()) {
        ToolValue = it->second;
    }
    return ToolValue;
}

MsgType ArmCore::RxGetToolNameList(const MsgType& in)
{
    (void)in;

    vector<string> UserToolName;
    for (const auto [num, tool] : user_.userTool) {
        UserToolName.push_back(num);
    }
    return UserToolName;
}

MsgType ArmCore::RxSetToolName(const MsgType& in)
{
    RetState ret;
    auto toolName = in.GetType<string>();

    if (user_.userTool.count(toolName) == 1) {
        coData_->toolName = toolName;
        ret = RetState::ok;
    } else {
        ret = RetState::noExist;
    }

    return ret;
}
MsgType ArmCore::RxGetToolName(const MsgType& in)
{
    (void)in;
    return coData_->toolName;
}

MsgType ArmCore::RxSetToolValue(const MsgType& in)
{
    map<std::string, Pose> userset;

    auto set = in.GetType<ArmUserTool>();
    // userset.insert(pair<std::string, Pose>(set.toolName, set.toolPose));
    // user_.userTool.emplace(set.toolName, set.toolPose);
    user_.userTool[set.toolName] = set.toolPose;
    return RetState::ok;
}

/**
 * @description: 上报任务执行结果
 * @param in
 * @return {}
 */
void ArmCore::ReportTaskExecResult(RetState ret)
{
    moveData_.cmd.reset();  // 发送task_exec之前，必须清除cmd
    MsgTrySend("arm::task_exec", ret);
}

/**
 * @description: 接收用户侧的设置IO指令
 * @param in
 * @return {}
 */
MsgType ArmCore::RxSetIoOut(const MsgType& in)
{
    auto ioCtl = in.GetType<std::pair<string, std::bitset<32>>>();
    // motor_->SetIoOut(ioCtl.first, ioCtl.second);
    MsgTrySend("arm::set_io", ioCtl);
    return RetState::ok;  // 必定成功
}

void ArmCore::ExecTaskCmd()
{
    if (moveData_.cmd.has_value()) {
        auto cmd = moveData_.cmd.value();
        if (GetArmState() == ArmState::noReady) {
            LOG_WARN_ARM("Task noReady, can't exec {}", Enum2Num(cmd));
            ReportTaskExecResult(RetState::noSupport);
        } else if (GetArmState() == ArmState::error) {
            LOG_WARN_ARM("Task error, can't exec {}", Enum2Num(cmd));
            ReportTaskExecResult(RetState::error);
        } else {
            switch (cmd) {
                case ArmCtrl::pause: {
                    SetArmState(ArmState::pause);
                    LOG_INFO_ARM("Task pause");
                    ReportTaskExecResult(RetState::interrupt);
                } break;
                case ArmCtrl::stop: {
                    if (moveData_.idx < GetFirstMoveData().size()) {
                        LOG_INFO_ARM("Task interrupt");
                        SetArmState(ArmState::interrupt);
                    } else {
                        // 发送stop的时候刚好任务完成，概率太小，一般不会出现
                        LOG_INFO_ARM("Task complete[stop]");
                        SetArmState(ArmState::complete);
                    }
                    ClearAllMoveData();
                    ReportTaskExecResult(RetState::interrupt);
                } break;

                case ArmCtrl::start:
                case ArmCtrl::resume: {
                    SetArmState(ArmState::running);
                    if (moveData_.idx >= GetFirstMoveData().size()) {
                        ClearFirstMoveData();
                    }
                    if (moveData_.datas.empty()) {
                        SetArmState(ArmState::complete);
                        LOG_INFO_ARM("Task complete");
                        ReportTaskExecResult(RetState::ok);
                    }
                    // moveData_.cmd.reset();  // 执行完成但不上报状态，todo任务状态是否需要独立的状态上报
                } break;

                default: {
                    LOG_WARN_ARM("exec cmd no support");
                    ReportTaskExecResult(RetState::noSupport);
                } break;
            }
        }
    } else {
        // cout << "ExecTaskCmd: 5" << endl;
    }
}

const std::vector<msg::arm_cmd>& ArmCore::GetFirstMoveData() { return moveData_.datas.front(); }

/**
 * @description: 清楚数据
 * @return {}
 */
void ArmCore::ClearFirstMoveData()
{
    if (!moveData_.datas.empty()) {
        moveData_.datas.pop_front();
        moveData_.idx = 0;
    }
}
void ArmCore::ClearLastMoveData()
{
    if (!moveData_.datas.empty()) {
        moveData_.datas.pop_back();  // 这个不会影响idx}
    }
}
void ArmCore::ClearAllMoveData()
{
    moveData_.datas.clear();
    moveData_.idx = 0;
}

/**
 * @description: 更新机械臂状态
 * @return {}
 */
void ArmCore::UpdateArmState()
{
    msg::arm_motor_state state = coData_->motorInfo.state;

    // 根据电机状态，修改机械臂的状态
    if (state == msg::arm_motor_state::disable) {
        ClearAllMoveData();
        SetArmState(ArmState::noReady);
    } else if (state == msg::arm_motor_state::error) {
        ClearAllMoveData();
        SetArmState(ArmState::error);
    } else if (state == msg::arm_motor_state::enable) {
        if ((GetArmState() == ArmState::noReady) || (GetArmState() == ArmState::error)) {
            SetArmState(ArmState::ready);
        }
    }
    if (GetArmState() == ArmState::complete) {
        if (TimerTools::GetNowTickMs() - lastStateChange_ > 100) {
            SetArmState(ArmState::ready);
        }
    }
}

/**
 * @description: 发送给qrChart
 * @return {}
 */
void ArmCore::Send2Server()
{
    Vec6<double> qData, ArmTeach;
    Pose pos;

    Pose ToolValue = user_.userTool[coData_->toolName];
    pos = math_.Fkine(coData_->nowAngle, ToolValue);

    ArmTeach << pos.tran_.x, pos.tran_.y, pos.tran_.z, pos.rpy_.rx, pos.rpy_.ry, pos.rpy_.rz;
    MsgTrySend("arm::arm_teach_cmd", ArmTeach);
}
