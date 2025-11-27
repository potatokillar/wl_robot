
#include "fsmWalk3.hpp"

#include "baseline.hpp"
#include "eventMsg.hpp"
#include "orientation_tools.h"

using namespace std;
using namespace ori;

// 该类继承于FsmState，构造函数写成这样
FsmWalk3::FsmWalk3(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::walk3, contr, jpData)
{
    motorCmd_.k2.fill(50.0);
    motorCmd_.k1.fill(3.0);
}

void FsmWalk3::OnEnter()
{
    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        LOG_WARN("Enter Walk2handStand Implementation for linkV2_3_w.");
    } else if (qrParam.model.contains(QrModel::linkV2_3) || qrParam.model.contains(QrModel::gazebo)) {
        LOG_INFO("Enter Walk2handStand");
    } else if (qrParam.model.contains(QrModel::middleV3)) {
        LOG_WARN("Enter Walk2handStand Implementation for middleV3.");
    } else {  // if (qrParam.model.contains(QrModel::middleV3_w)}
        LOG_ERROR("ERROR robot type!");
    }
    done_ = false;
}

void FsmWalk3::Run()
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

void FsmWalk3::Small()
{
    interval_++;

    Mat34<double> tempVecFoot = qrParam.pFoot;
    double ti = (double(interval_) / durations_);
    // double temp_pitch = (double(iRun) / 15 ) * 1.0;
    // double temp_h = (double(iRun) / 15 ) * 0.12;
    // cout << temp_pitch << endl;
    // for (int i = 0; i < 4; i++) {

    //     tempVecFoot.col(i) = mathFun_.Ry(tempVecFoot.col(i), -temp_pitch);

    // }
    // tempVecFoot(2, 0) += temp_h * 1.5;
    // tempVecFoot(2, 1) += temp_h * 1.5;
    // tempVecFoot(2, 2) -= temp_h;
    // tempVecFoot(2, 3) -= temp_h;
    motorCmd_.alpha = mathFun_.InverseKinematics_new(tempVecFoot, qrParam.pRoll);  //
    motorCmd_.alpha(1, 0) -= 0.5 * ti;
    motorCmd_.alpha(1, 1) -= 0.5 * ti;
    motorCmd_.alpha(0, 0) -= 0.25 * ti;
    motorCmd_.alpha(0, 1) += 0.25 * ti;
    motorCmd_.alpha(1, 2) += 0.6 * ti;
    motorCmd_.alpha(1, 3) += 0.6 * ti;
    motorCmd_.torq.setZero();
    // 计算tau
    Mat34<double> tmpForceFoot;
    tmpForceFoot.setZero();
    // 考虑重心相对于几何中心偏移
    double k = 0.6;  // 偏向于后腿
    tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k);
    tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k;

    tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
    tmpForceFoot(2, 3) = tmpForceFoot(2, 2);
    motorCmd_.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd_.alpha);

    txData_->SendMotorCmd(motorCmd_);
    if (interval_ == durations_) {
        done_ = true;
    }
}

void FsmWalk3::SmallWheel()
{
    done_ = true;
    return;
}

void FsmWalk3::Middle()
{
    done_ = true;
    return;
}

void FsmWalk3::MiddleWheel()
{
    done_ = true;
    return;
}

void FsmWalk3::OnExit()
{
    LOG_INFO("Walk2handStand exit");
    interval_ = 0;
}