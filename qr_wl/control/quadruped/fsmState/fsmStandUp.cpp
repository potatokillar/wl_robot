
#include "fsmStandUp.hpp"

#include "baseline.hpp"

void FsmStand2::OnEnter()
{
    LOG_INFO("standUp enter");
    done_ = false;
    upTime_.t = 0;
    qInitBias = rxData_->GetData3().alpha;
    qCmdOld = qInitBias;
}

void FsmStand2::Run()
{
    if (qrParam.model.contains(QrModel::middleV3)) {
        Middle();
    } else if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        SmallWheel();
    } else {
        Small();
    }
}

void FsmStand2::Small()
{
    DataCmd motorCmd;
    Mat34<double> qCmdStandUp1, qCmdStandUp2;                                                                                       // 注意，这是角度deg
    qCmdStandUp1 = qInitBias;                                                                                                       // under my marker
    qCmdStandUp1.row(0) += Vec4<double>(-MathFun::deg2rad(30), MathFun::deg2rad(30), -MathFun::deg2rad(30), MathFun::deg2rad(30));  // rad
    qCmdStandUp2 = qrParam.qCmdStandUpStep2;                                                                                        // deg

    upTime_.t += qrParam.dt;
    double st = 0.0;
    if (upTime_.t <= 1.0 * upTime_.T) {
        st = MathFun::Poly5(1.0 * upTime_.T, upTime_.t);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (qCmdStandUp1(0, i) - qInitBias(0, i)) * st + qInitBias(0, i);  // 1默认是弧度了
        }
        motorCmd.alpha.row(1) = qInitBias.row(1);
        motorCmd.alpha.row(2) = qInitBias.row(2);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (upTime_.t > 1.0 * upTime_.T && upTime_.t <= 2.0 * upTime_.T) {
        st = MathFun::Poly5(1.0 * upTime_.T, upTime_.t - 1.0 * upTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (MathFun::deg2rad(qCmdStandUp2(0, i)) - qCmdStandUp1(0, i)) * st + qCmdStandUp1(0, i);
            motorCmd.alpha(1, i) = (MathFun::deg2rad(qCmdStandUp2(1, i)) - qCmdStandUp1(1, i)) * st + qCmdStandUp1(1, i);
            motorCmd.alpha(2, i) = (MathFun::deg2rad(qCmdStandUp2(2, i)) - qCmdStandUp1(2, i)) * st + qCmdStandUp1(2, i);
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (upTime_.t > 2.0 * upTime_.T && upTime_.t <= 3.0 * upTime_.T) {
        Mat34<double> pFootReady, qCmdReady;
        pFootReady = jpData_->pFootForStand;

        pFootReady.row(2) << qrParam.pFootzMin, qrParam.pFootzMin, qrParam.pFootzMin, qrParam.pFootzMin;

        qCmdReady = mathFun_.InverseKinematics(pFootReady, qrParam.pRoll);
        st = MathFun::Poly5(1.0 * upTime_.T, upTime_.t - 2.0 * upTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (qCmdReady(0, i) - MathFun::deg2rad(qCmdStandUp2(0, i))) * st + MathFun::deg2rad(qCmdStandUp2(0, i));
            motorCmd.alpha(1, i) = (qCmdReady(1, i) - MathFun::deg2rad(qCmdStandUp2(1, i))) * st + MathFun::deg2rad(qCmdStandUp2(1, i));
            motorCmd.alpha(2, i) = (qCmdReady(2, i) - MathFun::deg2rad(qCmdStandUp2(2, i))) * st + MathFun::deg2rad(qCmdStandUp2(2, i));
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (upTime_.t > 3.0 * upTime_.T && upTime_.t <= 3.5 * upTime_.T) {
        Mat34<double> pFootUp;
        pFootUp = jpData_->pFootForStand;
        pFootUp.row(2).fill(qrParam.pFootzMin);
        motorCmd.alpha = mathFun_.InverseKinematics(pFootUp, qrParam.pRoll);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (upTime_.t > 3.5 * upTime_.T && upTime_.t <= 4.1 * upTime_.T) {
        st = MathFun::Poly5(0.6 * upTime_.T, upTime_.t - 3.5 * upTime_.T);
        Mat34<double> pFootUp;
        pFootUp = jpData_->pFootForStand;
        pFootUp.row(2).fill(qrParam.pFootzMin - st * (jpData_->desire.h - qrParam.pFootzMin * (-1)));
        motorCmd.alpha = mathFun_.InverseKinematics(pFootUp, qrParam.pRoll);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        // 力矩计算
        Mat34<double> tmpForceFoot;
        tmpForceFoot.setZero();
        double k = 0.6;                                                   // 偏向于后腿
        tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k) * st;  // 乘以st由0-1多项式过渡
        tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k * st;

        tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
        tmpForceFoot(2, 3) = tmpForceFoot(2, 2);

        motorCmd.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd.alpha);
    } else  // 注意，命令也要给,否则造成突变
    {
        Mat34<double> tmp_pFoot;
        tmp_pFoot = jpData_->pFootForStand;
        tmp_pFoot.row(2).fill(-jpData_->desire.h);
        motorCmd.alpha = mathFun_.InverseKinematics(tmp_pFoot, qrParam.pRoll);
        // 力矩计算
        Mat34<double> tmpForceFoot;
        tmpForceFoot.setZero();
        double k = 0.6;  // 偏向于后腿
        tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k);
        tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k;

        tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
        tmpForceFoot(2, 3) = tmpForceFoot(2, 2);
        motorCmd.torq.setZero();
        motorCmd.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd.alpha);
        jpData_->qCmdRecord = motorCmd.alpha;
        jpData_->tauCmdRecord = motorCmd.blta;

        done_ = true;
    }

    // std::cout << "motorCmd.alpha.row(2): " << std::endl;
    // std::cout << motorCmd.alpha.row(2) << std::endl;

    // todo 起来动作检查
    txData_->SendMotorCmd(motorCmd);

}
void FsmStand2::Middle()
{
    DataCmd motorCmd;
    upTime_.t += qrParam.dt;
    double st = 0.0;
    if (upTime_.t <= 1.0 * upTime_.T) {
        // 计算起立准备点位
        Mat34<double> pFootRead, qCmdReady;
        pFootRead = jpData_->pFootForStand;
        pFootRead.row(2).fill(qrParam.pFootzMin);
        qCmdReady = mathFun_.InverseKinematics(pFootRead, qrParam.pRoll);

        st = MathFun::Poly5(1.0 * upTime_.T, upTime_.t);
        for (int j = 0; j < 4; j++) {
            for (int i = 0; i < 3; i++) {
                motorCmd.alpha(i, j) = (qCmdReady(i, j) - qInitBias(i, j)) * st + qInitBias(i, j);  // 1默认是弧度了
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (upTime_.t > 1.0 * upTime_.T && upTime_.t <= 2.0 * upTime_.T) {  // 起立阶段，保留
        st = MathFun::Poly5(1.0 * upTime_.T, upTime_.t - 1.0 * upTime_.T);
        Mat34<double> pFootRead;
        pFootRead = jpData_->pFootForStand;
        pFootRead.row(2).fill(qrParam.pFootzMin - st * (jpData_->desire.h - qrParam.pFootzMin * (-1)));
        motorCmd.alpha = mathFun_.InverseKinematics(pFootRead, qrParam.pRoll);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        // 力矩计算
        Mat34<double> tmpForceFoot;
        tmpForceFoot.setZero();
        double k = 0.6;                                                   // 偏向于后腿
        tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k) * st;  // 乘以st由0-1多项式过渡
        tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k * st;

        tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
        tmpForceFoot(2, 3) = tmpForceFoot(2, 2);

        motorCmd.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd.alpha);
        // std::cout << "2" << std::endl;
    } else  // 注意，命令也要给,否则造成突变
    {
        Mat34<double> tmp_pFoot;
        tmp_pFoot = jpData_->pFootForStand;
        tmp_pFoot.row(2).fill(-jpData_->desire.h);
        motorCmd.alpha = mathFun_.InverseKinematics(tmp_pFoot, qrParam.pRoll);
        motorCmd.torq.setZero();
        qCmdOld = motorCmd.alpha;
        // 力矩计算
        Mat34<double> tmpForceFoot;
        tmpForceFoot.setZero();
        double k = 0.6;  // 偏向于后腿
        tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k);
        tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k;

        tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
        tmpForceFoot(2, 3) = tmpForceFoot(2, 2);

        motorCmd.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd.alpha);
        // std::cout << "3" << std::endl;
        done_ = true;
    }

    txData_->SendMotorCmd(motorCmd);
}

void FsmStand2::SmallWheel()
{
    DataCmd motorCmd;
    Mat34<double> qCmdStandUp1, qCmdStandUp2;  // 注意，这是角度deg
    qCmdStandUp1 = qInitBias;
    // under my marker
    qCmdStandUp1.row(0) += Vec4<double>(-MathFun::deg2rad(30), MathFun::deg2rad(30), -MathFun::deg2rad(30), MathFun::deg2rad(30));  // rad
    // qCmdStandUp2 = qrParam.qCmdStandUpStep2;
    qCmdStandUp2 << 5, -5, 5, -5, -80, -80, -80, -80, 150, 150, 150, 150;  // deg

    upTime_.t += qrParam.dt;
    double st = 0.0;
    if (upTime_.t <= 1.0 * upTime_.T) {
        st = MathFun::Poly5(1.0 * upTime_.T, upTime_.t);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (qCmdStandUp1(0, i) - qInitBias(0, i)) * st + qInitBias(0, i);  // 1默认是弧度了
        }
        motorCmd.alpha.row(1) = qInitBias.row(1);
        motorCmd.alpha.row(2) = qInitBias.row(2);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (upTime_.t > 1.0 * upTime_.T && upTime_.t <= 2.0 * upTime_.T) {
        st = MathFun::Poly5(1.0 * upTime_.T, upTime_.t - 1.0 * upTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (MathFun::deg2rad(qCmdStandUp2(0, i)) - qCmdStandUp1(0, i)) * st + qCmdStandUp1(0, i);
            motorCmd.alpha(1, i) = (MathFun::deg2rad(qCmdStandUp2(1, i)) - qCmdStandUp1(1, i)) * st + qCmdStandUp1(1, i);
            motorCmd.alpha(2, i) = (MathFun::deg2rad(qCmdStandUp2(2, i)) - qCmdStandUp1(2, i)) * st + qCmdStandUp1(2, i);
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (upTime_.t > 2.0 * upTime_.T && upTime_.t <= 3.0 * upTime_.T) {
        Mat34<double> pFootReady, qCmdReady;
        pFootReady = jpData_->pFootForStand;

        pFootReady.row(2) << qrParam.pFootzMin, qrParam.pFootzMin, qrParam.pFootzMin, qrParam.pFootzMin;

        qCmdReady = mathFun_.InverseKinematics(pFootReady, qrParam.pRoll);
        st = MathFun::Poly5(1.0 * upTime_.T, upTime_.t - 2.0 * upTime_.T);
        for (int i = 0; i < 4; i++) {
            motorCmd.alpha(0, i) = (qCmdReady(0, i) - MathFun::deg2rad(qCmdStandUp2(0, i))) * st + MathFun::deg2rad(qCmdStandUp2(0, i));
            motorCmd.alpha(1, i) = (qCmdReady(1, i) - MathFun::deg2rad(qCmdStandUp2(1, i))) * st + MathFun::deg2rad(qCmdStandUp2(1, i));
            motorCmd.alpha(2, i) = (qCmdReady(2, i) - MathFun::deg2rad(qCmdStandUp2(2, i))) * st + MathFun::deg2rad(qCmdStandUp2(2, i));
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (upTime_.t > 3.0 * upTime_.T && upTime_.t <= 3.5 * upTime_.T) {
        Mat34<double> pFootUp;
        pFootUp = jpData_->pFootForStand;
        pFootUp.row(2).fill(qrParam.pFootzMin);
        motorCmd.alpha = mathFun_.InverseKinematics(pFootUp, qrParam.pRoll);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (upTime_.t > 3.5 * upTime_.T && upTime_.t <= 4.1 * upTime_.T) {
        st = MathFun::Poly5(0.6 * upTime_.T, upTime_.t - 3.5 * upTime_.T);
        Mat34<double> pFootUp;
        pFootUp = jpData_->pFootForStand;
        pFootUp.row(2).fill(qrParam.pFootzMin - st * (jpData_->desire.h - qrParam.pFootzMin * (-1)));
        motorCmd.alpha = mathFun_.InverseKinematics(pFootUp, qrParam.pRoll);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        // 力矩计算
        Mat34<double> tmpForceFoot;
        tmpForceFoot.setZero();
        double k = 0.6;                                                   // 偏向于后腿
        tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k) * st;  // 乘以st由0-1多项式过渡
        tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k * st;

        tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
        tmpForceFoot(2, 3) = tmpForceFoot(2, 2);

        motorCmd.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd.alpha);
    } else  // 注意，命令也要给,否则造成突变
    {
        Mat34<double> tmp_pFoot;
        tmp_pFoot = jpData_->pFootForStand;
        tmp_pFoot.row(2).fill(-jpData_->desire.h);
        motorCmd.alpha = mathFun_.InverseKinematics(tmp_pFoot, qrParam.pRoll);
        // 力矩计算
        Mat34<double> tmpForceFoot;
        tmpForceFoot.setZero();
        double k = 0.6;  // 偏向于后腿
        tmpForceFoot(2, 0) = -qrParam.DOG_M * G(2) * 0.5 * (1 - k);
        tmpForceFoot(2, 2) = -qrParam.DOG_M * G(2) * 0.5 * k;

        tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
        tmpForceFoot(2, 3) = tmpForceFoot(2, 2);
        motorCmd.torq.setZero();
        motorCmd.blta = mathFun_.JacobianT(tmpForceFoot, motorCmd.alpha);
        jpData_->qCmdRecord = motorCmd.alpha;
        jpData_->tauCmdRecord = motorCmd.blta;

        done_ = true;
    }

    // std::cout << "motorCmd.alpha.row(2): " << std::endl;
    // std::cout << motorCmd.alpha.row(2) << std::endl;

    // todo 起来动作检查

    DataValue2 newCmd;
    newCmd.alpha.block(0, 0, 3, 4) = motorCmd.alpha;
    newCmd.torq.block(0, 0, 3, 4) = motorCmd.torq;
    newCmd.blta.block(0, 0, 3, 4) = motorCmd.blta;
    // std::cout << "newCmd.alpha.block(0, 0, 3, 4): " << std::endl;
    // std::cout << newCmd.alpha.block(0, 0, 3, 4) << std::endl;
    newCmd.k2.fill(50);
    newCmd.k1.fill(3);
    newCmd.k3.fill(1.0);
    // newCmd.kp.row(3).fill(0); // 黑色轮足参数，轮部驱动特殊
    // newCmd.kd.row(3).fill(2);
    newCmd.k2.row(3).fill(0.0); // 白色新组装轮足参数
    newCmd.k1.row(3).fill(0.5);
    newCmd.k3.row(3).fill(0);
    txData_->SendMotorCmd(newCmd);
}

void FsmStand2::OnExit() { LOG_INFO("standUp exit"); }