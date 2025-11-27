
#pragma once
#include <future>

#include "ArmType.hpp"
#include "armMathTools.hpp"
#include "baseline.hpp"
#include "userCore.hpp"

class ArmTask
{
public:
    ArmTask(const BootArgs& args, const ArmBootArgs& armArgs);
    ~ArmTask();
    void Run();

private:
    std::unique_ptr<UserCore> core_;
    MsgType RxWaitTaskComplete(const MsgType& in);
    std::vector<RpcRecv> rpcRxDeal_;  // rpc接收处理
};