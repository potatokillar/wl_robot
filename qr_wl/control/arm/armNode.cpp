
#include "armNode.hpp"

#include <fstream>

#include "armMathTools.hpp"
#include "armParam.hpp"
#include "baseline.hpp"

using namespace std;

void ArmNode::Start(const BootArgs& args, const ArmBootArgs& armArgs)
{
    // 若之前已经启动了算法，则关闭旧算法，析构由语法管理
    if ((armTask_) && (task_)) {
        task_->Stop();
    }

    armTask_ = make_unique<ArmTask>(args, armArgs);
    task_ = make_unique<PeriodicMemberFunction<ArmNode>>("ctrl-arm-task", armArgs.dt * 2, this, &ArmNode::Run, false);
    task_->Start();
}
void ArmNode::Stop()
{
    if (task_) {
        task_->Stop();
    }
}

void ArmNode::Run() { armTask_->Run(); }
