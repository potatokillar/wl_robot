
#include "ctrlNode.hpp"

#include "quadrupedParam.hpp"

using namespace ::std;

/**
 * @description: 启动
 * @param fromSim
 * @return {}
 */
void CtrlNode::Start(const BootArgs& args, const QrBootArgs& qrArgs)
{
    // 写这里
    // 构造状态机模块
    staManager_ = make_unique<StateManager>(args, qrArgs);
    string onnxPathClimb = GetRobotConfigDirect<string>("onnxPath", "climb").value();
    string onnxPathNormal = GetRobotConfigDirect<string>("onnxPath", "normal").value();
    ifstream onnxFile(onnxPathClimb), onnxFile2(onnxPathNormal);
    if (!onnxFile.is_open() || !onnxFile2.is_open()) {
        LOG_CRITICAL("can not find onnx file of AI!");
        return;
    }
    staManager_->Start();
}

void CtrlNode::Stop()
{
    if (staManager_) {
        staManager_->Stop();
        staManager_.reset();  // 还是得主动析构，因为内部有很多类，并没有提供stop方法，只能通过析构stop
    }
}
