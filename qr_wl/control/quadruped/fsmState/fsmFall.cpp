
#include "fsmFall.hpp"

#include "baseline.hpp"

void FsmFall::OnEnter()
{
    LOG_INFO("fall enter");
    jpData_->fallFlag = true;
    done_ = true;
    qFallCmd = rxData_->GetData3().alpha;
    txData_->SetCxdV3(qFallCmd);
    fallDownCoutTime = 0.0;
    kp.fill(0.0);
    kd.fill(15.0);
    kff.fill(0.0);
}

void FsmFall::Run()
{
    CmdVal2 motorCmd;
    motorCmd.alpha = qFallCmd;
    motorCmd.torq.setZero();
    motorCmd.blta.setZero();
    motorCmd.k2 = kp;
    motorCmd.k1 = kd;
    motorCmd.k3 = kff;

    if (qrParam.model.back() == QrModel::middleV4) {
        if (fallDownCoutTime < 8.0) {
            fallDownCoutTime += qrParam.dt;
            txData_->SendMotorCmd(motorCmd);
        } else {
            motorCmd.k1.fill(1.0);
            txData_->SendMotorCmd(motorCmd);
        }
    } else if (qrParam.model.back() == QrModel::middleV3) {
        if (fallDownCoutTime < 2.0) {
            fallDownCoutTime += qrParam.dt;
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 3; ++j) {
                    motorCmd.blta(j, i) = 0.0 * (0.0 - rxData_->GetData3().torq(j, i));
                }
            }
            motorCmd.k2.fill(0.0);
            motorCmd.k1.fill(5.0);
            motorCmd.k3.fill(0.0);
            txData_->SendMotorCmd(motorCmd);
        } else {
            motorCmd.k2.fill(0.0);
            motorCmd.k1.fill(0.0);
            motorCmd.k3.fill(0.0);
            txData_->SendMotorCmd(motorCmd);
        }
    } else if (qrParam.model.back() == QrModel::linkV2_3) {
        if (fallDownCoutTime < 5.0) {
            fallDownCoutTime += qrParam.dt;
            motorCmd.k1.fill(4.5);
            txData_->SendMotorCmd(motorCmd);
        } else {
            motorCmd.k1.fill(1.0);
            txData_->SendMotorCmd(motorCmd);
        }
    } else if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        DataValue2 newCmd;
        newCmd.alpha.block(0, 0, 3, 4) = motorCmd.alpha;
        newCmd.torq.block(0, 0, 3, 4) = motorCmd.torq;
        newCmd.blta.block(0, 0, 3, 4) = motorCmd.blta;
        newCmd.k2.fill(0);
        newCmd.k1.fill(3);
        newCmd.k3.fill(0);
        newCmd.torq.row(3).fill(0);
        newCmd.k2.row(3).fill(0);
        newCmd.k1.row(3).fill(2);
        newCmd.k3.row(3).fill(0);
        txData_->SendMotorCmd(newCmd);
    } else {
        txData_->SendMotorCmd(motorCmd);
    }
}

void FsmFall::OnExit()
{
    fallDownCoutTime = 0.0;
    LOG_INFO("fall exit");
}