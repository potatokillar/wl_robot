
#pragma once
#include <list>

#include "armCore.hpp"
#include "armCore6.hpp"
#include "armCore7.hpp"
#include "armMathTools.hpp"
#include "coreData.hpp"

class UserCore
{
public:
    UserCore(const BootArgs &args, const ArmBootArgs &armArgs);
    void Run();

private:
    std::unique_ptr<ArmCore> armcore_;

    std::thread thread_;
    // core的Run线程，motor的Run线程互斥
    std::mutex mtx_;
    ArmMathFun math_;
    std::vector<RpcRecv> rpcRxDeal_;  // rpc接收处理
    BootArgs boot_;
    double dt_;
    std::shared_ptr<arm::CoreData> coData_;
    u64 lastStateChange_;
    MoveTask moveData_;
    Vec6<double> qRecv_;
    Vec6<double> qOld_;

    bool Reset_ = false;
    double PY_ = 0.0;
    double QY_ = 0.0;
    Vec6<double> S_max_user_;
    Vec6<double> S_min_user_;
    Vec6<double> V_max_;
    double rapidrate_ = 1.0;
    std::vector<msg::arm_cmd> planlist_;

private:
    MsgType RxGetToolNameList(const MsgType &in);
    MsgType RxSetToolName(const MsgType &in);
    MsgType RxGetToolName(const MsgType &in);
    MsgType RxGetToolValue(const MsgType &in);
    MsgType RxUserCtrl(const MsgType &in);
    void ArmParam();
    MsgType RxGetArmParam(const MsgType &in);
    MsgType RxSetIoOut(const MsgType &in);
    MsgType RxMoveSearch(const MsgType &in);
    MsgType RxInfoCallback(const MsgType &in);

    void PubInfo();
    void ExecTaskCmd();
    void ReportTaskExecResult(RetState ret);
    void Send2Server();
    void UpdateArmState();
    void SetArmState(arm::ArmState sta);
    arm::ArmState GetArmState();

    RetState RxSetReset(const MsgType &in);
    RetState RxSetRapidRate(const MsgType &in);
    RetState RxSetPositionLimit(const MsgType &in);
    Result<ValueRange<std::vector<double>>> RxGetPositionLimit();
    RetState RxSetJointSpeedLimit(const MsgType &in);
    RetState RxSetJointAccLimit(const MsgType &in);
    RetState RxSetToolValue(const MsgType &in);
    Result<Pose> RxGetUserTool(const MsgType &in);

    void ClearFirstMoveData();
    void ClearLastMoveData();
    void ClearAllMoveData();
    const std::vector<msg::arm_cmd> &GetFirstMoveData();

    // 由电机侧回调的指令
    std::optional<msg::arm_cmd> TxRxCallback(const msg::arm_data &ret);
    void InfoCallback(const msg::arm_motor_info &info);

private:
    CppSqlite sql_;
    // ArmUserParam user_;
};
