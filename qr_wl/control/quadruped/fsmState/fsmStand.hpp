
#pragma once
#include "fsmState.hpp"

class FsmStand : public FsmState
{
public:
    FsmStand(ContrContainer contr, std::shared_ptr<JpData> jpData);
    void OnEnter() override;
    void Run() override;
    void OnExit() override;

private:
    double kp_ = 0.007;
    Mat34<double> pFootOld;
    Mat34<double> qdCmdFilter;
    double smoothValue = 0.0;
    size_t cmdQueueIdx_ = 0;  // 命令队列的序号
    Vec4<double> fzFilter;
    u64 timeForMode;
};