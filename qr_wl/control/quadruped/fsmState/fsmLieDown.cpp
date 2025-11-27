
#include "fsmLieDown.hpp"

#include "baseline.hpp"

void FsmLie2::OnEnter()
{
    LOG_INFO("lieDown enter");
    done_ = false;
    qCmdOld = mathFun_.InverseKinematics(jpData_->pFootForStand, qrParam.pRoll);
    downTime_.t = 0;
}

void FsmLie2::Run()
{
    // TODO: MiddleWheel is needed
    if (qrParam.model.contains(QrModel::middleV3)) {
        Middle();
    } else if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        SmallWheel();
    } else {
        Small();
    }
}

void FsmLie2::Small()
{
    DataCmd motorCmd;
    Mat34<double> qCmdLieDown1, qCmdLieDown2;
    qCmdLieDown1 = qrParam.qCmdLieDownStep1;
    qCmdLieDown2 = qrParam.qInit;                                                                                                   // under my marker  rad
    qCmdLieDown2.row(0) += Vec4<double>(-MathFun::deg2rad(30), MathFun::deg2rad(30), -MathFun::deg2rad(30), MathFun::deg2rad(30));  // rad

    downTime_.t += qrParam.dt;
    double st = 0.0;
    if (downTime_.t <= downTime_.T) {
        st = MathFun::Poly5(downTime_.T, downTime_.t);
        Mat34<double> tmp_pFoot;
        tmp_pFoot = jpData_->pFootForStand;
        tmp_pFoot.row(2).fill(-jpData_->desire.h - st * (-1 * qrParam.pFootzMin - jpData_->desire.h));
        motorCmd.alpha = mathFun_.InverseKinematics(tmp_pFoot, qrParam.pRoll);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        // 力矩计算
        Mat34<double> tmpForceFoot;
        tmpForceFoot.setZero();
        double k = 0.6;
        tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.6 * (1 - k) * (1.0 - st);
        tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.7 * k * (1.0 - st);

        tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
        tmpForceFoot(2, 3) = tmpForceFoot(2, 2);

        motorCmd.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd.alpha);
    } else if (downTime_.t > downTime_.T && downTime_.t <= 2 * downTime_.T) {
        Mat34<double> pFootReady, qCmdReady;
        pFootReady = jpData_->pFootForStand;
        pFootReady.row(2).fill(qrParam.pFootzMin);
        qCmdReady = mathFun_.InverseKinematics(pFootReady, qrParam.pRoll);
        st = MathFun::Poly5(downTime_.T, downTime_.t - downTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (MathFun::deg2rad(qCmdLieDown1(0, i)) - qCmdReady(0, i)) * st + qCmdReady(0, i);
            motorCmd.alpha(1, i) = (MathFun::deg2rad(qCmdLieDown1(1, i)) - qCmdReady(1, i)) * st + qCmdReady(1, i);
            motorCmd.alpha(2, i) = (MathFun::deg2rad(qCmdLieDown1(2, i)) - qCmdReady(2, i)) * st + qCmdReady(2, i);
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (downTime_.t > 2 * downTime_.T && downTime_.t <= 2.6 * downTime_.T) {
        st = MathFun::Poly5(0.6 * downTime_.T, downTime_.t - 2 * downTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (qCmdLieDown2(0, i) - MathFun::deg2rad(qCmdLieDown1(0, i))) * st + MathFun::deg2rad(qCmdLieDown1(0, i));
            motorCmd.alpha(1, i) = (qCmdLieDown2(1, i) - MathFun::deg2rad(qCmdLieDown1(1, i))) * st + MathFun::deg2rad(qCmdLieDown1(1, i));
            motorCmd.alpha(2, i) = (qCmdLieDown2(2, i) - MathFun::deg2rad(qCmdLieDown1(2, i))) * st + MathFun::deg2rad(qCmdLieDown1(2, i));
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (downTime_.t > 2.6 * downTime_.T && downTime_.t <= 3.6 * downTime_.T) {
        st = MathFun::Poly5(downTime_.T, downTime_.t - 2.6 * downTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (qrParam.qInit(0, i) - qCmdLieDown2(0, i)) * st + qCmdLieDown2(0, i);
            motorCmd.alpha(1, i) = (qrParam.qInit(1, i) - qCmdLieDown2(1, i)) * st + qCmdLieDown2(1, i);
            motorCmd.alpha(2, i) = (qrParam.qInit(2, i) - qCmdLieDown2(2, i)) * st + qCmdLieDown2(2, i);
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else {
        motorCmd.alpha = qrParam.qInit;
        motorCmd.torq.setZero();
        motorCmd.blta.setZero();
        done_ = true;
    }

    txData_->SendMotorCmd(motorCmd);
}

void FsmLie2::SmallWheel()
{
    DataCmd motorCmd;
    Mat34<double> qCmdLieDown1, qCmdLieDown2;
    qCmdLieDown1 = qrParam.qCmdLieDownStep1;
    qCmdLieDown2 = qrParam.qInit;                                                                                                   // under my marker  rad
    qCmdLieDown2.row(0) += Vec4<double>(-MathFun::deg2rad(30), MathFun::deg2rad(30), -MathFun::deg2rad(30), MathFun::deg2rad(30));  // rad

    downTime_.t += qrParam.dt;
    double st = 0.0;
    if (downTime_.t <= downTime_.T) {  // 蹲下
        st = MathFun::Poly5(downTime_.T, downTime_.t);
        Mat34<double> tmp_pFoot;
        tmp_pFoot = jpData_->pFootForStand;
        tmp_pFoot.row(2).fill(-jpData_->desire.h - st * (-1 * qrParam.pFootzMin - jpData_->desire.h));
        motorCmd.alpha = mathFun_.InverseKinematics(tmp_pFoot, qrParam.pRoll);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        // 力矩计算
        Mat34<double> tmpForceFoot;
        tmpForceFoot.setZero();
        double k = 0.6;
        tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k) * (1.0 - st);
        tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k * (1.0 - st);

        tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
        tmpForceFoot(2, 3) = tmpForceFoot(2, 2);

        motorCmd.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd.alpha);
    } else if (downTime_.t > downTime_.T && downTime_.t <= 2 * downTime_.T) {
        Mat34<double> pFootReady, qCmdReady;
        pFootReady = jpData_->pFootForStand;
        pFootReady.row(2).fill(qrParam.pFootzMin);
        qCmdReady = mathFun_.InverseKinematics(pFootReady, qrParam.pRoll);
        st = MathFun::Poly5(downTime_.T, downTime_.t - downTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (MathFun::deg2rad(qCmdLieDown1(0, i)) - qCmdReady(0, i)) * st + qCmdReady(0, i);
            motorCmd.alpha(1, i) = (MathFun::deg2rad(qCmdLieDown1(1, i)) - qCmdReady(1, i)) * st + qCmdReady(1, i);
            motorCmd.alpha(2, i) = (MathFun::deg2rad(qCmdLieDown1(2, i)) - qCmdReady(2, i)) * st + qCmdReady(2, i);
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (downTime_.t > 2 * downTime_.T && downTime_.t <= 2.6 * downTime_.T) {
        st = MathFun::Poly5(0.6 * downTime_.T, downTime_.t - 2 * downTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (qCmdLieDown2(0, i) - MathFun::deg2rad(qCmdLieDown1(0, i))) * st + MathFun::deg2rad(qCmdLieDown1(0, i));
            motorCmd.alpha(1, i) = (qCmdLieDown2(1, i) - MathFun::deg2rad(qCmdLieDown1(1, i))) * st + MathFun::deg2rad(qCmdLieDown1(1, i));
            motorCmd.alpha(2, i) = (qCmdLieDown2(2, i) - MathFun::deg2rad(qCmdLieDown1(2, i))) * st + MathFun::deg2rad(qCmdLieDown1(2, i));
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (downTime_.t > 2.6 * downTime_.T && downTime_.t <= 3.6 * downTime_.T) {
        st = MathFun::Poly5(downTime_.T, downTime_.t - 2.6 * downTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (qrParam.qInit(0, i) - qCmdLieDown2(0, i)) * st + qCmdLieDown2(0, i);
            motorCmd.alpha(1, i) = (qrParam.qInit(1, i) - qCmdLieDown2(1, i)) * st + qCmdLieDown2(1, i);
            motorCmd.alpha(2, i) = (qrParam.qInit(2, i) - qCmdLieDown2(2, i)) * st + qCmdLieDown2(2, i);
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else {
        motorCmd.alpha = qrParam.qInit;
        motorCmd.torq.setZero();
        motorCmd.blta.setZero();
        done_ = true;
    }

    DataValue2 newCmd;
    newCmd.alpha.block(0, 0, 3, 4) = motorCmd.alpha;
    newCmd.torq.block(0, 0, 3, 4) = motorCmd.torq;
    newCmd.blta.block(0, 0, 3, 4) = motorCmd.blta;
    newCmd.k2.fill(50);
    newCmd.k1.fill(3);
    newCmd.k3.fill(1.0);
    newCmd.k2.row(3).fill(0);
    newCmd.k1.row(3).fill(2);
    newCmd.k3.row(3).fill(0);
    txData_->SendMotorCmd(newCmd);
}
void FsmLie2::Middle()
{
    DataCmd motorCmd;
    downTime_.t += qrParam.dt;
    double st = 0.0;
    if (downTime_.t <= 1.0 * downTime_.T) {  // 蹲下
        st = MathFun::Poly5(1.0 * downTime_.T, downTime_.t);
        Mat34<double> pFootReady, qCmdReady;
        pFootReady = jpData_->pFootForStand;
        pFootReady.row(2).fill(-jpData_->desire.h - st * (-1 * qrParam.pFootzMin - jpData_->desire.h));
        motorCmd.alpha = mathFun_.InverseKinematics(pFootReady, qrParam.pRoll);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        // 力矩计算
        Mat34<double> tmpForceFoot;
        tmpForceFoot.setZero();
        double k = 0.6;
        tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k) * (1.0 - st);
        tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k * (1.0 - st);

        tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
        tmpForceFoot(2, 3) = tmpForceFoot(2, 2);

        motorCmd.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd.alpha);
    } else if (downTime_.t > 1.0 * downTime_.T && downTime_.t <= 2.0 * downTime_.T) {
        Mat34<double> pFootReady, qCmdReady;
        pFootReady = jpData_->pFootForStand;
        pFootReady.row(2).fill(qrParam.pFootzMin);
        qCmdReady = mathFun_.InverseKinematics(pFootReady, qrParam.pRoll);
        st = MathFun::Poly5(downTime_.T, downTime_.t - 1.0 * downTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (qrParam.qInit(0, i) - qCmdReady(0, i)) * st + qCmdReady(0, i);
            motorCmd.alpha(1, i) = (qrParam.qInit(1, i) - qCmdReady(1, i)) * st + qCmdReady(1, i);
            motorCmd.alpha(2, i) = (qrParam.qInit(2, i) - qCmdReady(2, i)) * st + qCmdReady(2, i);
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else {
        motorCmd.alpha = qrParam.qInit;
        motorCmd.torq.setZero();
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
        done_ = true;
        // std::cout << "middle ok" << std::endl;
    }

    txData_->SendMotorCmd(motorCmd);
}

void FsmLie2::OnExit() {}