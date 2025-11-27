
#include "fsmWalk2upright.hpp"

#include "baseline.hpp"
#include "eventMsg.hpp"
#include "fsmStand.hpp"
#include "orientation_tools.h"

using namespace std;
using namespace ori;

FsmWalk2Upright::FsmWalk2Upright(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::walk5, contr, jpData) {}

void FsmWalk2Upright::OnEnter()
{
    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        LOG_WARN("Enter Walk2Upright. Implementation for linkV2_3_w.");
    } else if (qrParam.model.contains(QrModel::linkV2_3) || qrParam.model.contains(QrModel::gazebo)) {
        LOG_INFO("Enter Walk2Upright.");
        done_ = false;
        deltaTi << 100, 200, 300, 400, 400, 100;  // 最后一个没用到
        Ti(0) = deltaTi(0);
        Ti(1) = deltaTi(0) + deltaTi(1);
        Ti(2) = deltaTi(0) + deltaTi(1) + deltaTi(2);
        Ti(3) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3);
        Ti(4) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4);
        Ti(5) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4) + deltaTi(5);
        Mat34<double> tempVecFoot = qrParam.pFoot;
        tmpCmd0 = mathFun_.InverseKinematics_new(tempVecFoot, qrParam.pRoll);
    } else if (qrParam.model.contains(QrModel::middleV3)) {
        LOG_WARN("Enter Walk2Upright. Implementation for middleV3.");
    } else {  // if (qrParam.model.contains(QrModel::middleV3_w)}
        LOG_ERROR("ERROR robot type.");
    }
}
void FsmWalk2Upright::Run()
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

void FsmWalk2Upright::Small()
{
    ti++;
    CmdVal2 cmd;
    cmd.k2.fill(50.0);
    cmd.k1.fill(3.0);
    if (ti < Ti(0)) {
        double s = ti * 1.0 / Ti(0);
        cmd.alpha = tmpCmd0;
        cmd.alpha(2, 0) += 0.7 * s;
        cmd.alpha(2, 1) += 0.7 * s;
        cmd.alpha(2, 2) += 0.7 * s;
        cmd.alpha(2, 3) += 0.7 * s;
        cmd.torq.setZero();
        cmd.blta.setZero();
        tmpCmd1 = cmd.alpha;
    } else if (ti >= Ti(0) && ti < Ti(1)) {
        double s = (ti - Ti(0)) * 1.0 / (Ti(1) - Ti(0));
        cmd.alpha = tmpCmd1;
        cmd.alpha(1, 0) += 0.8 * s;
        cmd.alpha(1, 1) += 0.8 * s;
        cmd.alpha(2, 0) -= 0.8 * s;
        cmd.alpha(2, 1) -= 0.8 * s;
        cmd.alpha(1, 2) += 0.8 * s;
        cmd.alpha(1, 3) += 0.8 * s;
        cmd.alpha(2, 2) -= 0.8 * s;
        cmd.alpha(2, 3) -= 0.8 * s;
        cmd.torq.setZero();
        cmd.blta.setZero();
        tmpCmd2 = cmd.alpha;
    } else if (ti >= Ti(1) && ti < Ti(2)) {
        double s = (ti - Ti(1)) * 1.0 / (Ti(2) - Ti(1));
        cmd.alpha = tmpCmd2;
        cmd.alpha(1, 0) -= 1.3 * s;
        cmd.alpha(1, 1) -= 1.3 * s;
        cmd.alpha(2, 0) -= 0.3 * s;
        cmd.alpha(2, 1) -= 0.3 * s;
        cmd.alpha(1, 2) -= 0.7 * s;
        cmd.alpha(1, 3) -= 0.7 * s;
        // cmd.alpha(2, 2) += 0.3 * s;
        // cmd.alpha(2, 3) += 0.3 * s;
        cmd.torq.setZero();
        cmd.blta.setZero();
        tmpCmd3 = cmd.alpha;
    } else if (ti >= Ti(2) && ti < Ti(3)) {
        double s = (ti - Ti(2)) * 1.0 / (Ti(3) - Ti(2));
        cmd.alpha = tmpCmd3;
        // cmd.alpha(0, 0) += 0.3 * s;
        // cmd.alpha(0, 1) -= 0.3 * s;
        // cmd.alpha(1, 0) += 0.5 * s;
        // cmd.alpha(1, 1) += 0.5 * s;
        cmd.alpha(1, 2) -= 1.1 * s;
        cmd.alpha(1, 3) -= 1.1 * s;
        cmd.alpha(2, 2) += 0.6 * s;
        cmd.alpha(2, 3) += 0.6 * s;
        cmd.torq.setZero();
        cmd.blta.setZero();
        tmpCmd4 = cmd.alpha;
    } else if (ti >= Ti(3) && ti < Ti(4)) {
        // double s = (ti - Ti(3)) * 1.0 / (Ti(4) - Ti(3));
        cmd.alpha = tmpCmd4;
        // cmd.alpha(0, 0) += 0.3 * s;
        // cmd.alpha(0, 1) -= 0.3 * s;
        // cmd.alpha(1, 0) += 0.5 * s;
        // cmd.alpha(1, 1) += 0.5 * s;
        // cmd.alpha(2, 0) += 1.0 * s;
        // cmd.alpha(2, 1) += 1.0 * s;
        cmd.torq.setZero();
        cmd.blta.setZero();
        tmpCmd5 = cmd.alpha;
    } else if (ti >= Ti(4) && ti < Ti(5)) {
        // double s = (ti - Ti(4)) * 1.0 / (Ti(5) - Ti(4));
        cmd.alpha = tmpCmd5;
        // cmd.alpha(0, 0) += 0.3 * s;
        // cmd.alpha(0, 1) -= 0.3 * s;
        // cmd.alpha(1, 0) += 0.5 * s;
        // cmd.alpha(1, 1) += 0.5 * s;
        // cmd.alpha(2, 2) += 0.5 * s;
        // cmd.alpha(2, 3) += 0.5 * s;
        cmd.torq.setZero();
        cmd.blta.setZero();
        tmpCmd6 = cmd.alpha;
    } else if (ti == Ti(5)) {
        done_ = true;
        cmd.alpha = tmpCmd6;
    }
    txData_->SendMotorCmd(cmd);
}

void FsmWalk2Upright::SmallWheel()
{
    done_ = true;
    return;
}

void FsmWalk2Upright::Middle()
{
    done_ = true;
    return;
}

void FsmWalk2Upright::MiddleWheel()
{
    done_ = true;
    return;
}

void FsmWalk2Upright::OnExit()
{
    LOG_INFO("FsmWalk2Upright exit");
    ti = 0;
}