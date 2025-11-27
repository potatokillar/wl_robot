
#pragma once
#include "fsmState.hpp"

class FsmRecover : public FsmState
{
public:
    FsmRecover(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::recover, contr, jpData)
    {
        qFallDown.setZero();
        qCmdOld.setZero();
    }

    void OnEnter() override;
    void Run() override;
    void OnExit() override;
    void Small();
    void Middle();

private:
    StateShiftTime recoverTime{1.0};
    Mat34<double> qFallDown;
    Mat34<double> qCmdRecover1, qCmdRecover2, qCmdRecover3, qCmdRecover4;
    Mat34<double> qCmdOld;
    Vec6<double> kt;
};