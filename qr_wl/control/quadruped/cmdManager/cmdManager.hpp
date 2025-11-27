
#pragma once
#include <list>

#include "baseline.hpp"
#include "innerType.hpp"
#include "mathTools.hpp"
#include "quadrupedParam.hpp"

struct UserSetRange
{
    ValueRange<Vec3<double>> v;
    ValueRange<Vec3<double>> w;
    ValueRange<Vec3<double>> p;
    ValueRange<double> h;
    ValueRange<double> loadMass;
};

class CmdManager
{
public:
    CmdManager();
    ~CmdManager();
    void Run();

    Vec3<double> GetLinear() const;
    Vec3<double> GetAngular() const;
    Vec3<double> GetPose() const;
    bool GetHoldflag() const;
    double GetHeight() const;
    WalkMode GetWalkMode() const;
    GaitType GetGaitType() const;
    RunState GetRunState() const;
    double GetLoadMass() const;
    const std::shared_ptr<std::vector<CmdVal2>> GetMotorCmdQueue() const;
    void ClearMotorCmdQueue();

private:
    void SetWalkModeRange(WalkMode mode);
    const QuadrupedParam& qrParam{GetQrParam()};
    void PublishParamRange();
    MathFun mathFun_;
    UserSetRange userSetRange_;
    ControllerCmd cmd_;
    std::list<std::string> message_;
    std::shared_ptr<std::vector<CmdVal2>> motorCmdQueue_;

    /// 新消息框架
    std::vector<RpcRecv> rpcRxDeal_;  // rpc接收处理

    MsgType RxSetLinearVelocity(const MsgType& set);
    MsgType RxSetAngularVelocity(const MsgType& set);
    MsgType RxSetPose(const MsgType& set);
    MsgType RxSetHeight(const MsgType& heightOffset);
    MsgType RxSetWalkMode(const MsgType& mode);
    MsgType RxSetGaitType(const MsgType& gait);
    MsgType RxSetHoldFlag(const MsgType& set);
    MsgType RxSetMessage(const MsgType& msg);
    MsgType RxSetLoadMass(const MsgType& mass);
    MsgType RxSetMotorCmdQ(const MsgType& cmd);

    u64 moveCmdLastTime_ = 0;  // 上一次运动指令的时间
};
