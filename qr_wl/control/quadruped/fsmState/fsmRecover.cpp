
#include "baseline.hpp"
#include "fsmRecover.hpp"

void FsmRecover::OnEnter()
{
    LOG_INFO("recover OnEnter");
    done_ = false;
    recoverTime.t = 0;
    if (staEstimator_->GetResult().rpy(0) > 0) {
        qCmdRecover1 << 0, 0, 0, 0, 0, 0, 0, 0, 150, 150, 150, 150;  // under my marker

        qCmdRecover2 << -40, 20, -40, 20, -40, -80, -40, -80, 150, 150, 150, 150;
        if (qrParam.model.contains(QrModel::middleV3)) {
            qCmdRecover3 << 55, 15, 55, 15, -80, 45, -80, 45, 150, 90, 150, 90;

            qCmdRecover4 << 55, -35, 55, -35, -60, -60, -60, -60, 150, 150, 150, 150;
        } else if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
            qCmdRecover2 << -80, 20, -80, 20, -40, -90, -40, -90, 150, 150, 150, 150;
            qCmdRecover3 << -30, -25, -30, -25, -90, -30, -90, -30, 150, 150, 150, 150;

            qCmdRecover4 << 75, -45, 75, -45, -60, -60, -60, -60, 150, 150, 150, 150;
        } else {
            qCmdRecover3 << -30, -25, -30, -25, -80, -80, -80, -80, 150, 150, 150, 150;

            qCmdRecover4 << 35, -35, 35, -35, -60, -60, -60, -60, 150, 150, 150, 150;
        }
    } else {
        qCmdRecover1 << 0, 0, 0, 0, 0, 0, 0, 0, 150, 150, 150, 150;  // under my marker

        qCmdRecover2 << -20, 40, -20, 40, -80, -40, -80, -40, 150, 150, 150, 150;
        if (qrParam.model.contains(QrModel::middleV3)) {
            qCmdRecover3 << -15, -55, -15, -55, 45, -80, 45, -80, 90, 150, 90, 150;

            qCmdRecover4 << 35, -55, 35, -55, -60, -60, -60, -60, 150, 150, 150, 150;
        } else if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
            qCmdRecover2 << -20, 80, -20, 80, -90, -40, -90, -40, 150, 150, 150, 150;
            qCmdRecover3 << 25, 30, 25, 30, -30, -90, -30, -90, 150, 150, 150, 150;

            qCmdRecover4 << 45, -75, 45, -75, -60, -60, -60, -60, 150, 150, 150, 150;
        } else {
            qCmdRecover3 << 25, 30, 25, 30, -80, -80, -80, -80, 150, 150, 150, 150;  // 腿塞进

            qCmdRecover4 << 35, -35, 35, -35, -60, -60, -60, -60, 150, 150, 150, 150;  // 翻身
        }
    }

    // 不需要执行翻身动作，直接起立,如果是中型，速度慢一点
    if (qrParam.model.contains(QrModel::middleV3)) {
        kt << 1.5, 3.0, 5.0, 8.0, 9.0, 10;
    } else {
        kt << 1.0, 2.0, 3.0, 3.5, 4.0, 5.0;
    }
    if (fabs(staEstimator_->GetResult().rpy(0)) < 0.1) {
        recoverTime.t = kt(3) * recoverTime.T;  // 注意，这里小型和中型kT值不一样
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                qCmdRecover4(i, j) = MathFun::rad2deg(rxData_->GetData3().alpha(i, j));
            }
        }
    }

    qFallDown = rxData_->GetData3().alpha;
    qCmdOld = qFallDown;  // 每次进来初始化为摔倒时的关节角
    txData_->SetCxdV3(qFallDown);
}

void FsmRecover::Run()
{
    if (qrParam.model.contains(QrModel::middleV3)) {
        Middle();
    } else {
        Small();
    }
}

void FsmRecover::Small()
{
    DataCmd motorCmd;
    recoverTime.t += qrParam.dt;
    double st = 0.0;
    if (recoverTime.t <= recoverTime.T * kt(0)) {                   // 1
        st = mathFun_.Poly5(recoverTime.T * kt(0), recoverTime.t);  // 改用五次多项式
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.alpha(i, j) = qFallDown(i, j) + st * (MathFun::deg2rad(qCmdRecover1(i, j)) - qFallDown(i, j));
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();

    } else if (recoverTime.t > recoverTime.T * kt(0) && recoverTime.t <= recoverTime.T * kt(1)) {  // 2
        st = (recoverTime.t - recoverTime.T * kt(0)) / (recoverTime.T * (kt(1) - kt(0)));
        // st = mathFun_.Poly5(recoverTime.T, recoverTime.t - recoverTime.T);  // 改用五次多项式
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.alpha(i, j) = MathFun::deg2rad(qCmdRecover1(i, j)) + st * (MathFun::deg2rad(qCmdRecover2(i, j)) - MathFun::deg2rad(qCmdRecover1(i, j)));
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();

    } else if (recoverTime.t > recoverTime.T * kt(1) && recoverTime.t <= recoverTime.T * kt(2)) {  // 3
        st = (recoverTime.t - recoverTime.T * kt(1)) / (recoverTime.T * (kt(2) - kt(1)));
        // st = mathFun_.Poly5(2 * recoverTime.T, recoverTime.t - 2 * recoverTime.T);  // 改用五次多项式
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.alpha(i, j) = MathFun::deg2rad(qCmdRecover2(i, j)) + st * (MathFun::deg2rad(qCmdRecover3(i, j)) - MathFun::deg2rad(qCmdRecover2(i, j)));
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();

    } else if (recoverTime.t > recoverTime.T * kt(2) && recoverTime.t <= recoverTime.T * kt(3)) {  // 4
        // st = (recoverTime.t - 4 * recoverTime.T) / (0.5 * recoverTime.T);
        st = mathFun_.Poly5(recoverTime.T * (kt(3) - kt(2)), recoverTime.t - recoverTime.T * kt(2));  // 改用五次多项式翻身
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.alpha(i, j) = MathFun::deg2rad(qCmdRecover3(i, j)) + st * (MathFun::deg2rad(qCmdRecover4(i, j)) - MathFun::deg2rad(qCmdRecover3(i, j)));
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();

    } else if (recoverTime.t > recoverTime.T * kt(3) && recoverTime.t <= recoverTime.T * kt(4)) {  // 6到达准备起立位置
        Mat34<double> pFootReady, qCmdReady;
        pFootReady = qrParam.pFoot;
        pFootReady.row(2) << -0.15, -0.15, -0.15, -0.15;
        qCmdReady = mathFun_.InverseKinematics(pFootReady, qrParam.pRoll);
        // st = MathFun::Poly5(recoverTime.T, recoverTime.t - 4.5 * recoverTime.T);
        st = (recoverTime.t - recoverTime.T * kt(3)) / (recoverTime.T * (kt(4) - kt(3)));
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.alpha(i, j) = MathFun::deg2rad(qCmdRecover4(i, j)) + st * ((qCmdReady(i, j)) - MathFun::deg2rad(qCmdRecover4(i, j)));
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
    } else if (recoverTime.t > recoverTime.T * kt(4) && recoverTime.t <= recoverTime.T * kt(5)) {  // 7站起来
        Mat34<double> pFootUp, qCmdUp;
        pFootUp = qrParam.pFoot;
        st = MathFun::Poly5(recoverTime.T * (kt(5) - kt(4)), recoverTime.t - recoverTime.T * kt(4));
        pFootUp.row(2).fill(-0.15 - st * (cmd_->GetHeight() - 0.15));
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
    } else {
        done_ = true;

        Mat34<double> pFootTmp = qrParam.pFoot;
        pFootTmp.row(2).fill(-cmd_->GetHeight());
        motorCmd.alpha = mathFun_.InverseKinematics(pFootTmp, qrParam.pRoll);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
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

        // 注意，摔倒后起立用的是Recover，不是standup
        jpData_->qCmdRecord = motorCmd.alpha;
        jpData_->tauCmdRecord = motorCmd.blta;

        jpData_->pFoot = pFootTmp;
        jpData_->pFootForStand = pFootTmp;
        jpData_->desire = {};  // 重新全部初始化,否则摔倒以后有残余值
        jpData_->desire.h = cmd_->GetHeight();
        jpData_->current = {};  // 保险起见，这个也初始化
        jpData_->current.h = cmd_->GetHeight();
    }

    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        DataValue2 newCmd;
        newCmd.alpha.block(0, 0, 3, 4) = motorCmd.alpha;
        newCmd.torq.block(0, 0, 3, 4) = motorCmd.torq;
        newCmd.blta.block(0, 0, 3, 4) = motorCmd.blta;

        newCmd.k2.fill(60);
        newCmd.k1.fill(3);
        newCmd.k3.fill(1.0);
        newCmd.k2.row(3).fill(0);
        newCmd.k1.row(3).fill(2);
        newCmd.k3.row(3).fill(0);
        txData_->SendMotorCmd(newCmd);
    } else {
        txData_->SendMotorCmd(motorCmd);
    }
}

void FsmRecover::Middle()
{
    DataCmd motorCmd;
    recoverTime.t += qrParam.dt;
    double st = 0.0;
    if (recoverTime.t <= recoverTime.T * kt(0)) {  // 1
        // st = recoverTime.t / recoverTime.T;
        st = mathFun_.Poly5(recoverTime.T * kt(0), recoverTime.t);  // 改用五次多项式
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.alpha(i, j) = qFallDown(i, j) + st * (MathFun::deg2rad(qCmdRecover1(i, j)) - qFallDown(i, j));
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
        // std::cout << "1 = " << st << std::endl;
    } else if (recoverTime.t > recoverTime.T * kt(0) && recoverTime.t <= recoverTime.T * kt(1)) {  // 2
        st = (recoverTime.t - recoverTime.T * kt(0)) / (recoverTime.T * (kt(1) - kt(0)));
        // st = mathFun_.Poly5(recoverTime.T, recoverTime.t - recoverTime.T);  // 改用五次多项式
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.alpha(i, j) = MathFun::deg2rad(qCmdRecover1(i, j)) + st * (MathFun::deg2rad(qCmdRecover2(i, j)) - MathFun::deg2rad(qCmdRecover1(i, j)));
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
        // std::cout << "2 = " << st << std::endl;
    } else if (recoverTime.t > recoverTime.T * kt(1) && recoverTime.t <= recoverTime.T * kt(2)) {  // 3
        st = (recoverTime.t - recoverTime.T * kt(1)) / (recoverTime.T * (kt(2) - kt(1)));
        // st = mathFun_.Poly5(2 * recoverTime.T, recoverTime.t - 2 * recoverTime.T);  // 改用五次多项式
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.alpha(i, j) = MathFun::deg2rad(qCmdRecover2(i, j)) + st * (MathFun::deg2rad(qCmdRecover3(i, j)) - MathFun::deg2rad(qCmdRecover2(i, j)));
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
        // std::cout << "3 = " << st << std::endl;
    } else if (recoverTime.t > recoverTime.T * kt(2) && recoverTime.t <= recoverTime.T * kt(3)) {  // 4
        // st = (recoverTime.t - 4 * recoverTime.T) / (0.5 * recoverTime.T);
        st = mathFun_.Poly5(recoverTime.T * (kt(3) - kt(2)), recoverTime.t - recoverTime.T * kt(2));  // 改用五次多项式翻身
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.alpha(i, j) = MathFun::deg2rad(qCmdRecover3(i, j)) + st * (MathFun::deg2rad(qCmdRecover4(i, j)) - MathFun::deg2rad(qCmdRecover3(i, j)));
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
        // std::cout << "4 = " << st << std::endl;
    } else if (recoverTime.t > recoverTime.T * kt(3) && recoverTime.t <= recoverTime.T * kt(4)) {  // 6到达准备起立位置
        Mat34<double> pFootReady, qCmdReady;
        pFootReady = qrParam.pFoot;
        pFootReady.row(2) << -0.15, -0.15, -0.15, -0.15;
        qCmdReady = mathFun_.InverseKinematics(pFootReady, qrParam.pRoll);
        // st = MathFun::Poly5(recoverTime.T, recoverTime.t - 4.5 * recoverTime.T);
        st = (recoverTime.t - recoverTime.T * kt(3)) / (recoverTime.T * (kt(4) - kt(3)));
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.alpha(i, j) = MathFun::deg2rad(qCmdRecover4(i, j)) + st * ((qCmdReady(i, j)) - MathFun::deg2rad(qCmdRecover4(i, j)));
            }
        }
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
        motorCmd.blta.setZero();
        // std::cout << "5 = " << st << std::endl;
    } else if (recoverTime.t > recoverTime.T * kt(4) && recoverTime.t <= recoverTime.T * kt(5)) {  // 7站起来
        Mat34<double> pFootUp, qCmdUp;
        pFootUp = qrParam.pFoot;
        st = MathFun::Poly5(recoverTime.T * (kt(5) - kt(4)), recoverTime.t - recoverTime.T * kt(4));
        pFootUp.row(2).fill(-0.15 - st * (cmd_->GetHeight() - 0.15));
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
        // std::cout << "6 = " << st << std::endl;
    } else {
        done_ = true;

        Mat34<double> pFootTmp = qrParam.pFoot;
        pFootTmp.row(2).fill(-cmd_->GetHeight());
        motorCmd.alpha = mathFun_.InverseKinematics(pFootTmp, qrParam.pRoll);
        motorCmd.torq = (motorCmd.alpha - qCmdOld) / qrParam.dt;
        qCmdOld = motorCmd.alpha;
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

        // 注意，摔倒后起立用的是Recover，不是standup
        jpData_->qCmdRecord = motorCmd.alpha;
        jpData_->tauCmdRecord = motorCmd.blta;

        jpData_->pFoot = pFootTmp;
        jpData_->pFootForStand = pFootTmp;
        jpData_->desire = {};  // 重新全部初始化,否则摔倒以后有残余值
        jpData_->desire.h = cmd_->GetHeight();
        jpData_->current = {};  // 保险起见，这个也初始化
        jpData_->current.h = cmd_->GetHeight();
    }

    // if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
    //     DataValue2 newCmd;
    //     newCmd.alpha.block(0, 0, 3, 4) = motorCmd.alpha;
    //     newCmd.torq.block(0, 0, 3, 4) = motorCmd.torq;
    //     newCmd.blta.block(0, 0, 3, 4) = motorCmd.blta;

    //     newCmd.k2.fill(60);
    //     newCmd.k1.fill(3);
    //     newCmd.k3.fill(1.0);
    //     newCmd.k2.row(3).fill(0);
    //     newCmd.k1.row(3).fill(2);
    //     newCmd.k3.row(3).fill(0);
    //     txData_->SendMotorCmd(newCmd);
    // } else {
    txData_->SendMotorCmd(motorCmd);
    // }
}

void FsmRecover::OnExit() { LOG_INFO("recover exit"); }