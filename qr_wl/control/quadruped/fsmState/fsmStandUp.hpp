
#pragma once
#include "fsmState.hpp"

class FsmStand2 : public FsmState
{
public:
    FsmStand2(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::stand2, contr, jpData) { qCmdOld.setZero(); }

    void OnEnter() override;
    void Run() override;
    void Small();
    void Middle();
    void SmallWheel();
    void OnExit() override;

private:
    StateShiftTime upTime_{GetQrParam().standUpTime};

    Mat34<double> qInitBias;
    Mat34<double> qCmdOld;
};