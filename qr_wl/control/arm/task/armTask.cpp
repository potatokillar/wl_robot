
#include "armTask.hpp"

#include "baseline.hpp"

using namespace arm;
using namespace std;

ArmTask::ArmTask(const BootArgs& args, const ArmBootArgs& armArgs)
{
    MsgTryRecv<RetState>("arm::task_exec", this);  // msg的特性，第一次接收时会注册到队列中，不注册可能导致第一次数据收不到
    rpcRxDeal_.emplace_back("arm::wait_task_complete", [this](MsgType set) { return this->RxWaitTaskComplete(set); });
    for (auto& rpc : rpcRxDeal_) {
        rpc.Connect();
    }
    core_ = make_unique<UserCore>(args, armArgs);
}

ArmTask::~ArmTask()
{
    for (auto& rpc : rpcRxDeal_) {
        rpc.DisConnect();
    }
}

void ArmTask::Run()
{
    for (auto& rpc : rpcRxDeal_) {
        rpc.Run();
    }
}

/**
 * @description: 等待任务完成
 * @param in
 * @return {}
 */
MsgType ArmTask::RxWaitTaskComplete(const MsgType& in)
{
    u32 ms = in.GetType<u32>();
    RetState taskRet = RetState::timeout;

    auto ret = MsgBlockRecv<RetState>("arm::task_exec", this, ms);
    if (ret) {
        taskRet = ret.value();
    } else {
        LOG_WARN_ARM("Task timeout");
        taskRet = RetState::timeout;
    }
    return taskRet;
}
