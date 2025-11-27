
#pragma once
#include <list>

#include "armMathTools.hpp"
#include "baseline.hpp"
#include "coreData.hpp"
// #include "cppSqlite.hpp"
#include "innerType.hpp"

struct armPoint
{
    Vec6<double> qSend_;
    Vec6<double> qdSend_;
    Vec6<double> tauSend_;
    Vec6<double> kpSend_;
    Vec6<double> kdSend_;
};

// Arm启动参数，注意和全局启动参数的区别
struct ArmBootArgs
{
    double dt;  // 核心模块的调度周期
};

struct MoveTask
{
    std::optional<arm::ArmCtrl> cmd;
    bool isExec;
    std::list<std::vector<msg::arm_cmd>> datas;  // 所有数据的集合
    std::size_t idx;                             // 指向第一个数据的索引
};

class ArmCore
{
public:
    // 谁都能访问
    ArmCore(const ArmBootArgs& armArgs);
    virtual ~ArmCore() {};
    virtual void Run();

protected:
    ArmMathFun math_;
    ArmUserParam user_;
    std::vector<RpcRecv> rpcRxDeal_;  // rpc接收处理
    std::shared_ptr<arm::CoreData> coData_;
    MoveTask moveData_;
    // 子类可以访问
    double dt_;
    u64 lastStateChange_;
    double rapidrate_ = 0.5;
    bool reset_ = false;
    void SetArmState(arm::ArmState sta);

    const std::vector<msg::arm_cmd>& GetFirstMoveData();
    void ClearFirstMoveData();
    void ClearLastMoveData();
    void ClearAllMoveData();
    arm::ArmState GetArmState();
    CppSqlite sql_;

private:
    void ArmParam();
    MsgType RxUserCtrl(const MsgType& in);
    MsgType RxGetArmParam(const MsgType& in);
    RetState RxSetReset(const MsgType& in);
    RetState RxSetRapidRate(const MsgType& in);
    void InfoCallback(const msg::arm_motor_info& info);
    MsgType RxGetToolValue(const MsgType& in);
    MsgType RxGetToolNameList(const MsgType& in);
    MsgType RxSetToolName(const MsgType& in);
    MsgType RxGetToolName(const MsgType& in);
    MsgType RxSetIoOut(const MsgType& in);
    MsgType RxSetToolValue(const MsgType& in);
    void ReportTaskExecResult(RetState ret);
    void ExecTaskCmd();

    void UpdateArmState();

    void Send2Server();

    RetState RxSetPositionLimit(const MsgType& in);
    // Result<ValueRange<std::vector<double>>> RxGetPositionLimit();
    RetState RxSetJointSpeedLimit(const MsgType& in);
    RetState RxSetJointAccLimit(const MsgType& in);
    // Result<Pose> RxGetUserTool(const MsgType& in);
};
