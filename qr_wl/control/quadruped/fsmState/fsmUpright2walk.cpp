
#include "fsmUpright2walk.hpp"

#include "baseline.hpp"
#include "eventMsg.hpp"
#include "fsmStand.hpp"
#include "orientation_tools.h"

using namespace std;
using namespace ori;

// 该类继承于FsmState，构造函数写成这样
FsmUpright2Walk::FsmUpright2Walk(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::upright2walk, contr, jpData) {}

void FsmUpright2Walk::OnEnter()
{
    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        LOG_WARN("Enter Upright2Walk. Implementation for linkV2_3_w.");
    } else if (qrParam.model.contains(QrModel::linkV2_3) || qrParam.model.contains(QrModel::gazebo)) {
        LOG_INFO("Enter Upright2Walk.");
    } else if (qrParam.model.contains(QrModel::middleV3)) {
        LOG_WARN("Enter Upright2Walk. Implementation for middleV3.");
    } else {  // if (qrParam.model.contains(QrModel::middleV3_w)}
        LOG_ERROR("ERROR robot type!");
    }
    done_ = false;
}

void FsmUpright2Walk::Run()
{
    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        SmallWheel();
    } else if (qrParam.model.contains(QrModel::linkV2_3) || qrParam.model.contains(QrModel::gazebo)) {
        Small();
    } else if (qrParam.model.contains(QrModel::middleV3)) {
        Middle();
    } else {  // if (qrParam.model.contains(QrModel::middleV3_w)}
        MiddleWheel();
    }
}

void FsmUpright2Walk::Small()
{
    interval_++;
    double s = interval_ * 1.0 / duration_;
    CmdVal2 cmd;
    cmd.k2.fill(50.0);
    cmd.k1.fill(3.0);
    cmd.alpha = rxData_->GetData3().alpha;
    cmd.alpha(1, 2) += 0.3 * s;
    cmd.alpha(1, 3) += 0.3 * s;
    cmd.torq.setZero();
    cmd.blta.setZero();
    txData_->SendMotorCmd(cmd);
    if (interval_ == duration_) {
        done_ = true;
    }
}

void FsmUpright2Walk::SmallWheel()
{
    done_ = true;
    return;
}

void FsmUpright2Walk::Middle()
{
    done_ = true;
    return;
}

void FsmUpright2Walk::MiddleWheel()
{
    done_ = true;
    return;
}

void FsmUpright2Walk::OnExit()
{
    LOG_INFO("FsmUpright2Walk exit");
    interval_ = 0;
}