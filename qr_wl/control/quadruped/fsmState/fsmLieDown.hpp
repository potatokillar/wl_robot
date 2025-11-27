
#pragma once
#include "fsmState.hpp"

class FsmLie2 : public FsmState
{
public:
    FsmLie2(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::lie2, contr, jpData)
    {
        qCmdOld = mathFun_.InverseKinematics(jpData_->pFootForStand, qrParam.pRoll);
    }

    void OnEnter() override;
    void Run() override;
    void Small();
    void SmallWheel();
    void Middle();
    void OnExit() override;

private:
    StateShiftTime downTime_{GetQrParam().lieDownTime};
    Mat34<double> qCmdOld;
};