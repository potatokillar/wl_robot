
#include "fsmWalk.hpp"

#include <iostream>

#include "baseline.hpp"
#include "debugParam.hpp"
#include "orientation_tools.h"

using namespace std;
using namespace ori;

FsmWalk::FsmWalk(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::walk, contr, jpData) {}

void FsmWalk::OnEnter()
{
    LOG_INFO("walk enter");
    done_ = false;
    jpData_->sPhi.fill(0);
    jpData_->phi.fill(0.0);

    if ((cmd_->GetWalkMode() == WalkMode::aiNormal) || (cmd_->GetWalkMode() == WalkMode::aiClimb)) {
        done_ = true;
        Mat34<double> tempVecFoot = jpData_->pFootForStand;
        for (int i = 0; i < 4; i++) {
            tempVecFoot(2, i) = -jpData_->desire.h;
        }
        Mat34<double> qTemp = mathFun_.InverseKinematics_new(tempVecFoot, qrParam.pRoll);
        aiWalk_.SetTsrV3(rxData_->GetImuData());
        aiWalk_.SetStandQ(qTemp);
        aiWalk_.Start(cmd_->GetWalkMode());
        jpData_->desire.mode = cmd_->GetWalkMode();
    }
}
void FsmWalk::Run()
{
    auto& desMode = jpData_->desire.mode;

    if (desMode != cmd_->GetWalkMode()) {
        aiWalk_.Stop();
        if ((cmd_->GetWalkMode() == WalkMode::aiNormal) || (cmd_->GetWalkMode() == WalkMode::aiClimb)) {
            done_ = true;
            aiWalk_.SetStandQ(rxData_->GetData3().alpha);  // 获取此时机器人关节角
            aiWalk_.Start(cmd_->GetWalkMode());
        }
        desMode = cmd_->GetWalkMode();
    }

    if ((desMode == WalkMode::aiNormal) || (desMode == WalkMode::aiClimb)) {
        aiWalk_.SetTsrV3(rxData_->GetImuData());
        aiWalk_.SetCxdV3(cmd_->GetLinear()(0), cmd_->GetLinear()(1), cmd_->GetAngular()(2));

        if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
            aiWalk_.SetCxdV2(rxData_->GetMotorDataWheelMit());
            txData_->SendMotorCmdFixed(aiWalk_.t2flagY());
        } else {
            aiWalk_.SetCxdV2(rxData_->GetMotorDataMit());
            txData_->SendMotorCmd2(aiWalk_.ret2flagX());
        }
    }
}
void FsmWalk::OnExit()
{
    LOG_INFO("walk exit");
    aiWalk_.Stop();
}
