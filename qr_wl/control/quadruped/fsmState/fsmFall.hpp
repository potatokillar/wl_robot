
#pragma once
#include "fsmState.hpp"
class FsmFall : public FsmState
{
public:
    FsmFall(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::fall, contr, jpData) { qFallCmd.setZero(); }

    void OnEnter() override;
    void Run() override;
    void OnExit() override;

private:
    Mat34<double> qFallCmd;
    double fallDownCoutTime = 0.0;
    Mat34<double> kp, kd, kff;  // 暂时给中型四足使用
};