
#include "qrTask.hpp"

#include "baseline.hpp"
#include "quadrupedParam.hpp"

using namespace std;

QrTask::QrTask()
{
    // 为了生成pFoot
    pCom.setZero();
    pRoll(0, 0) = pRoll(0, 1) = pCom(0, 0) + BODY_Lx * 0.5;
    pRoll(0, 2) = pRoll(0, 3) = pCom(0, 0) - BODY_Lx * 0.5;
    // pRolly
    pRoll(1, 0) = pRoll(1, 2) = pCom(1, 0) - BODY_Ly * 0.5;
    pRoll(1, 1) = pRoll(1, 3) = pCom(1, 0) + BODY_Ly * 0.5;
    // pRollz
    pRoll.row(2).fill(0.0);
    // pHip
    pHip = pRoll;
    pHip(1, 0) = pRoll(1, 0) - LEG_L0;
    pHip(1, 1) = pRoll(1, 1) + LEG_L0;
    pHip(1, 2) = pRoll(1, 2) - LEG_L0;
    pHip(1, 3) = pRoll(1, 3) + LEG_L0;
    // pFoot
    pFoot = pHip;
    pFoot.row(2) << -bodyHeight, -bodyHeight, -bodyHeight, -bodyHeight;

    // 前馈力矩计算
    tmpForceFoot.setZero();
    double k = 0.6;  // 偏向于后腿
    tmpForceFoot(2, 0) = -DOG_M * G * 0.5 * (1 - k);
    tmpForceFoot(2, 2) = -DOG_M * G * 0.5 * k;

    tmpForceFoot(2, 1) = tmpForceFoot(2, 0);
    tmpForceFoot(2, 3) = tmpForceFoot(2, 2);
    cmdQ_ = std::make_shared<std::vector<CmdVal2>>();
}

RetState QrTask::mixedDance1()
{
    auto len = rpyDance();
    return SendCmd(len);
}

RetState QrTask::mixedDance0()
{
    pushUp(3);
    int period = 4;
    double zDown = 0.1;  // max - 0.15
    double zUp = 0.17;   // max -0.25
    // double kBack = 0.0;  //+-0.3
    Mat34<double> pFootDeltaA, pFootDeltaB;

    pFootDeltaA << 0.15, 0.15, -0.15, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    pFootDeltaB << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    moonWalk2(pFootDeltaA, pFootDeltaB, period, zDown, zUp, 0.0, 0.0);
    pFootDeltaA << 0.0, 0.0, 0.0, 0.0, -0.12, -0.12, 0.12, 0.12, 0.0, 0.0, 0.0, 0.0;
    pFootDeltaB << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    moonWalk2(pFootDeltaA, pFootDeltaB, period, zDown, zUp, 0.2, 0.0);
    pFootDeltaA << 0.0, 0.0, 0.0, 0.0, 0.12, 0.12, -0.12, -0.12, 0.0, 0.0, 0.0, 0.0;
    pFootDeltaB << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    moonWalk2(pFootDeltaA, pFootDeltaB, period, zDown, zUp, 0.2, 0.0);
    pFootDeltaA << 0.0, 0.0, 0.0, 0.0, -0.15, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    pFootDeltaB << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.15, 0.15, 0.0, 0.0, 0.0, 0.0;
    moonWalk2(pFootDeltaA, pFootDeltaB, period, zDown, zUp, 0.4, 0.0);
    pFootDeltaA << 0.0, 0.0, 0.0, 0.0, -0.15, 0.15, -0.15, 0.15, 0.0, 0.0, 0.0, 0.0;
    pFootDeltaB << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    auto len = moonWalk2(pFootDeltaA, pFootDeltaB, period, zDown, zUp, 0.5, 0.0);
    return SendCmd(len);
}
RetState QrTask::singleDance0()
{
    // Vec3<double> angle;
    // angle << 0, 0.05, 0.15;
    // double period = 2;
    // double showTime = 2;
    auto len = handShake2();
    return SendCmd(len);
}
RetState QrTask::singleDance1()
{
    auto len = takeBow2();
    return SendCmd(len);
}
RetState QrTask::singleDance2()
{
    int period = 4;
    DVec<double> angle = DVec<double>::Zero(period, 1);  // 次数是该矢量长度
    DVec<double> x = DVec<double>::Zero(period, 1);
    DVec<double> y = DVec<double>::Zero(period, 1);
    DVec<double> zDown = DVec<double>::Zero(period, 1);
    DVec<double> zUp = DVec<double>::Zero(period, 1);
    DVec<double> delayT = DVec<double>::Zero(period, 1);
    double kBack = 0.1;             // 偏向与后退的比例
    angle << 1.0, -1.0, -1.0, 1.0;  // 每次跳的角度和方向
    x << 0.0, 0.0, 0.0, 0.0;
    y << 0.0, 0.0, 0.0, 0.0;
    zDown << 0.1, 0.1, 0.1, 0.1;    // max - 0.2
    zUp << 0.17, 0.17, 0.17, 0.17;  // max - 0.31
    delayT << 0.1, 0.1, 0.1, 0.1;
    auto len = happyJump(angle, x, y, zDown, zUp, delayT, kBack);
    return SendCmd(len);
}
RetState QrTask::singleDance3()
{
    int period = 6;
    // double stepLx = 0.1;
    double zDown = 0.1;  // max - 0.15
    double zUp = 0.17;   // max -0.25
    double delayT = 0.0;
    double kBack = 0.0;  //+-0.3
    Mat34<double> pFootDeltaA, pFootDeltaB;
    pFootDeltaA << 0.1, -0.1, -0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    pFootDeltaB << -0.1, 0.1, 0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    auto len = moonWalk2(pFootDeltaA, pFootDeltaB, period, zDown, zUp, delayT, kBack);
    return SendCmd(len);
}
RetState QrTask::singleDance4()
{
    double kBack = -0.0;  // 偏向与后退的比例
    double angle = 0;     // 每次跳的角度和方向
    double x = 0.25;
    double y = 0;
    double zDown = 0.1;              // max - 0.2
    double zUp = 0.2;                // max - 0.31
    double delayT = 0.4;             // 设置一些延迟提供恢复
    Vec4<Mat34<double>> kp, kd, kf;  // 4 stage kp kd kf
    kp(0).fill(50);
    kd(0).fill(3);
    kf(0).fill(1);

    kp(1).fill(100);
    // kp(1).row(0).fill(200);
    // kp(1).row(1).fill(50);
    // kp(1).row(2).fill(50);
    kd(1).fill(3);
    kf(1).fill(1);

    kp(2).fill(50);
    kd(2).fill(3);
    kf(2).fill(1);

    kp(3).fill(50);
    kd(3).fill(3);
    kf(3).fill(1);

    auto len = longJump2(kp, kd, kf, angle, x, y, zDown, zUp, delayT, kBack);
    return SendCmd(len);
}

/**
 * @description: 实现参数化跳跃动作
 * @param kpIn 设置12个电机kp参数，决定跳跃强度
 * @param kdIn 设置12个电机kd参数，决定跳跃强度
 * @param kfIn 设置12个电机kf参数，决定跳跃强度
 * @param angleIn 设置跳跃角度
 * @param xIn 设置跳跃x方向位移
 * @param yIn 设置跳跃y方向位移
 * @param zDownIn 设置跳跃z上方向位移
 * @param zUpIn 设置跳跃z下方向位移
 * @param delayIn 设置跳跃延时
 * @param kBackIn 设置跳跃后恢复系数
 * @return {}
 */
int QrTask::longJump2(Vec4<Mat34<double>> kpIn, Vec4<Mat34<double>> kdIn, Vec4<Mat34<double>> kfIn, double angleIn, double xIn, double yIn, double zDownIn, double zUpIn,
                      double delayIn, double kBackIn)
{
    double angleValue = mathFun_.SetValueRange(angleIn, -2.0, 2.0, "dance angle", false);
    double x = mathFun_.SetValueRange(xIn, -0.1, 1.0, "dance x", false);
    double y = mathFun_.SetValueRange(yIn, -0.05, 0.05, "dance y", false);
    double downZ = mathFun_.SetValueRange(zDownIn, 0, 0.2, "down limit", false);
    double upZ = mathFun_.SetValueRange(zUpIn, 0, 0.39 - (bodyHeight - downZ), "up limit", false);
    double delay = mathFun_.SetValueRange(delayIn, 0, 10, "delay t limit", false);
    double kBack = mathFun_.SetValueRange(kBackIn, -1.0, 1.0, "kBack limit", false);

    Vec4<Mat34<double>> kp, kd, kf;

    for (int k = 0; k < 4; k++) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                kp(k)(i, j) = mathFun_.SetValueRange(kpIn(k)(i, j), 0, 500, "kp limit", false);
                kd(k)(i, j) = mathFun_.SetValueRange(kdIn(k)(i, j), 0, 5, "kd limit", false);
                kf(k)(i, j) = mathFun_.SetValueRange(kfIn(k)(i, j), 0, 2, "kf limit", false);
            }
        }
    }

    kp = kpIn;
    kd = kdIn;
    kf = kfIn;
    double dx = 0.08;
    double dz = 0.1;
    CmdVal2 cmd;
    Vec4<double> TiJump;
    TiJump << 250, 350, 450, 800;
    TiJump(3) = TiJump(2) + delay * 500;
    for (int ti = 0; ti < TiJump(3); ti++) {
        if (ti >= 0 && ti <= TiJump(0)) {
            // 动作下蹲2
            double s = ti * 1.0 / TiJump(0);
            double deltaH = downZ * s;
            Mat34<double> tempVecFoot = pFoot;
            // tempVecFoot(1, 0) -= 0.05 * s;
            // tempVecFoot(1, 1) += 0.05 * s;
            // tempVecFoot(1, 2) -= 0.05 * s;
            // tempVecFoot(1, 3) += 0.05 * s;

            tempVecFoot.row(2).fill(-bodyHeight + deltaH);  // 0.18
            tempVecFoot(0, 0) -= dx * s;
            tempVecFoot(0, 1) -= dx * s;
            tempVecFoot(0, 2) -= dx * s;
            tempVecFoot(0, 3) -= dx * s;

            cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            cmd.k2 = kp(0);
            cmd.k1 = kd(0);
            cmd.k3 = kf(0);

        } else if (ti > TiJump(0) && ti <= TiJump(1)) {
            // 动作起跳2
            Mat34<double> tempVecFoot = pFoot;
            // tempVecFoot(1, 0) -= 0.05;
            // tempVecFoot(1, 1) += 0.05;
            // tempVecFoot(1, 2) -= 0.05;
            // tempVecFoot(1, 3) += 0.05;

            tempVecFoot(0, 0) -= dx;
            tempVecFoot(0, 1) -= dx;
            tempVecFoot(0, 2) -= dx;
            tempVecFoot(0, 3) -= dx;

            // tempVecFoot.row(2).fill(-bodyHeight + downZ);

            double s = ((ti - TiJump(0)) * 1.0 / (TiJump(1) - TiJump(0)));
            double deltaH = upZ * s;  // 135度
            // tempVecFoot.row(2).fill(-bodyHeight + downZ - deltaH);  // 0.18 ~ 0.35
            tempVecFoot(2, 0) = (-bodyHeight + downZ - deltaH * (1 - kBack));
            tempVecFoot(2, 1) = (-bodyHeight + downZ - deltaH * (1 - kBack));

            tempVecFoot(2, 2) = (-bodyHeight + downZ - deltaH * (1 + kBack));
            tempVecFoot(2, 3) = (-bodyHeight + downZ - deltaH * (1 + kBack));

            // tempVecFoot(0, 0) -= 1.0 * s;
            // tempVecFoot(0, 1) -= 1.0 * s;
            // tempVecFoot(0, 2) -= 1.0 * s;
            // tempVecFoot(0, 3) -= 1.0 * s;
            //  转弯 和 平移
            double deltaRz = angleValue * s;
            double deltaX = x * s;
            double deltaY = y * s;
            for (int i = 0; i < 4; i++) {
                tempVecFoot.col(i) = mathFun_.Rz(tempVecFoot.col(i), -deltaRz);
                tempVecFoot(0, i) -= deltaX;
                tempVecFoot(1, i) -= deltaY;
            }

            cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            cmd.k2 = kp(1);
            cmd.k1 = kd(1);
            cmd.k3 = kf(1);
        } else if (ti > TiJump(1) && ti <= TiJump(2)) {
            // 恢复2
            Mat34<double> tempVecFoot = pFoot;
            // tempVecFoot(1, 0) -= 0.05;
            // tempVecFoot(1, 1) += 0.05;
            // tempVecFoot(1, 2) -= 0.05;
            // tempVecFoot(1, 3) += 0.05;

            // double deltaH = downZ * ((ti - TiJump(1)) * 1.0 / (TiJump(2) - TiJump(1)));  // 135度
            tempVecFoot.row(2).fill(-bodyHeight + dz);  // 0.18-0.28
            cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.torq.setZero();
            // cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            cmd.blta.setZero();
            cmd.k2 = kp(2);
            cmd.k1 = kd(2);
            cmd.k3 = kf(2);
        } else {
            // delay
            Mat34<double> tempVecFoot = pFoot;
            // tempVecFoot(1, 0) -= 0.05;
            // tempVecFoot(1, 1) += 0.05;
            // tempVecFoot(1, 2) -= 0.05;
            // tempVecFoot(1, 3) += 0.05;

            double s = ((ti - TiJump(2)) * 1.0 / (TiJump(3) - TiJump(2)));
            tempVecFoot.row(2).fill(-bodyHeight + dz - dz * s);  // 0.18-0.28
            cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot * s, cmd.alpha);
            cmd.k2 = kp(3);
            cmd.k1 = kd(3);
            cmd.k3 = kf(3);
        }

        cmdQ_->push_back(cmd);
    }
    // 发送给对应线程
    return cmdQ_->size();
}
/**
 * @description:
 * @return {}
 */
int QrTask::longJump(Vec4<Mat34<double>> kpIn, Vec4<Mat34<double>> kdIn, Vec4<Mat34<double>> kfIn, double angleIn, double xIn, double yIn, double zDownIn, double zUpIn,
                     double delayIn, double kBackIn)
{
    double angleValue = mathFun_.SetValueRange(angleIn, -2.0, 2.0, "dance angle", false);
    double x = mathFun_.SetValueRange(xIn, -0.1, 1.0, "dance x", false);
    double y = mathFun_.SetValueRange(yIn, -0.05, 0.05, "dance y", false);
    double downZ = mathFun_.SetValueRange(zDownIn, 0, 0.2, "down limit", false);
    double upZ = mathFun_.SetValueRange(zUpIn, 0, 0.39 - (bodyHeight - downZ), "up limit", false);
    double delay = mathFun_.SetValueRange(delayIn, 0, 10, "delay t limit", false);
    double kBack = mathFun_.SetValueRange(kBackIn, -1.0, 1.0, "kBack limit", false);

    Vec4<Mat34<double>> kp, kd, kf;

    for (int k = 0; k < 4; k++) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                kp(k)(i, j) = mathFun_.SetValueRange(kpIn(k)(i, j), 0, 500, "kp limit", false);
                kd(k)(i, j) = mathFun_.SetValueRange(kdIn(k)(i, j), 0, 5, "kd limit", false);
                kf(k)(i, j) = mathFun_.SetValueRange(kfIn(k)(i, j), 0, 2, "kf limit", false);
            }
        }
    }

    kp = kpIn;
    kd = kdIn;
    kf = kfIn;
    CmdVal2 cmd;
    Vec4<double> TiJump;
    TiJump << 250, 350, 600, 800;
    TiJump(3) = TiJump(2) + delay * 500;
    for (int ti = 0; ti < TiJump(3); ti++) {
        if (ti > 0 && ti <= TiJump(0)) {
            // 动作下蹲2
            Mat34<double> tempVecFoot = pFoot;
            tempVecFoot(1, 0) -= 0.05;
            tempVecFoot(1, 1) += 0.05;
            tempVecFoot(1, 2) -= 0.05;
            tempVecFoot(1, 3) += 0.05;
            double s = ti * 1.0 / TiJump(0);
            double deltaH = downZ * s;
            tempVecFoot.row(2).fill(-bodyHeight + deltaH);  // 0.18

            cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            cmd.k2 = kp(0);
            cmd.k1 = kd(0);
            cmd.k3 = kf(0);

        } else if (ti > TiJump(0) && ti <= TiJump(1)) {
            // 动作起跳2
            Mat34<double> tempVecFoot = pFoot;
            tempVecFoot(1, 0) -= 0.05;
            tempVecFoot(1, 1) += 0.05;
            tempVecFoot(1, 2) -= 0.05;
            tempVecFoot(1, 3) += 0.05;
            double s = ((ti - TiJump(0)) * 1.0 / (TiJump(1) - TiJump(0)));
            double deltaH = upZ * s;  // 135度
            // tempVecFoot.row(2).fill(-bodyHeight + downZ - deltaH);  // 0.18 ~ 0.35
            tempVecFoot(2, 0) = (-bodyHeight + downZ - deltaH * (1 - kBack));
            tempVecFoot(2, 1) = (-bodyHeight + downZ - deltaH * (1 - kBack));

            tempVecFoot(2, 2) = (-bodyHeight + downZ - deltaH * (1 + kBack));
            tempVecFoot(2, 3) = (-bodyHeight + downZ - deltaH * (1 + kBack));
            // 转弯 和 平移
            double deltaRz = angleValue * s;
            double deltaX = x * s;
            double deltaY = y * s;
            for (int i = 0; i < 4; i++) {
                tempVecFoot.col(i) = mathFun_.Rz(tempVecFoot.col(i), -deltaRz);
                tempVecFoot(0, i) -= deltaX;
                tempVecFoot(1, i) -= deltaY;
            }

            cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            cmd.k2 = kp(1);
            cmd.k1 = kd(1);
            cmd.k3 = kf(1);
        } else if (ti > TiJump(1) && ti <= TiJump(2)) {
            // 恢复2
            Mat34<double> tempVecFoot = pFoot;
            tempVecFoot(1, 0) -= 0.05;
            tempVecFoot(1, 1) += 0.05;
            tempVecFoot(1, 2) -= 0.05;
            tempVecFoot(1, 3) += 0.05;
            double deltaH = downZ * ((ti - TiJump(1)) * 1.0 / (TiJump(2) - TiJump(1)));  // 135度
            tempVecFoot.row(2).fill(-bodyHeight + downZ - deltaH);                       // 0.18-0.28
            cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            cmd.k2 = kp(2);
            cmd.k1 = kd(2);
            cmd.k3 = kf(2);
        } else if (ti > TiJump(2) && ti <= TiJump(3)) {
            // delay
            Mat34<double> tempVecFoot = pFoot;
            tempVecFoot(1, 0) -= 0.05;
            tempVecFoot(1, 1) += 0.05;
            tempVecFoot(1, 2) -= 0.05;
            tempVecFoot(1, 3) += 0.05;
            tempVecFoot.row(2).fill(-bodyHeight);  // 0.18-0.28
            cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            cmd.k2 = kp(3);
            cmd.k1 = kd(3);
            cmd.k3 = kf(3);
        }

        cmdQ_->push_back(cmd);
    }
    // 发送给对应线程
    return cmdQ_->size();
}

/**
 * @description: 实现参数化欢乐跳
 * @param angleVec 设置跳跃
 * @param xVec 设置跳跃X轴
 * @param yVec 设置跳跃Y轴
 * @param zVecDown 设置向下跳幅度
 * @param zVecUp 设置向上跳幅度
 * @param delayVec 设置动作中间延迟时间
 * @param kBackIn 设置跳动后身体回弹幅度
 * @return {}
 */
int QrTask::happyJump(DVec<double> angleVec, DVec<double> xVec, DVec<double> yVec, DVec<double> zVecDown, DVec<double> zVecUp, DVec<double> delayVec, double kBackIn)
{
    int N = angleVec.size();
    DVec<double> angleValue = DVec<double>::Zero(N, 1);  // 次数是该矢量长度
    DVec<double> x = DVec<double>::Zero(N, 1);
    DVec<double> y = DVec<double>::Zero(N, 1);
    DVec<double> downZ = DVec<double>::Zero(N, 1);
    DVec<double> upZ = DVec<double>::Zero(N, 1);
    DVec<double> delay = DVec<double>::Zero(N, 1);
    double kBack = mathFun_.SetValueRange(kBackIn, -1.0, 1.0, "kBack limit", false);

    for (int i = 0; i < N; i++) {
        angleValue(i) = mathFun_.SetValueRange(angleVec(i), -2.0, 2.0, "dance angle", false);
        x(i) = mathFun_.SetValueRange(xVec(i), -0.1, 1.0, "dance x", false);
        y(i) = mathFun_.SetValueRange(yVec(i), -0.05, 0.05, "dance y", false);
        downZ(i) = mathFun_.SetValueRange(zVecDown(i), 0, 0.2, "down limit", false);
        upZ(i) = mathFun_.SetValueRange(zVecUp(i), 0, 0.39 - (bodyHeight - downZ(i)), "up limit", false);
        delay(i) = mathFun_.SetValueRange(delayVec(i), 0, 10, "delay t limit", false);

        CmdVal2 cmd;
        Vec4<double> TiJump;
        TiJump << 250, 350, 600, 800;
        TiJump(3) = TiJump(2) + delay(i) * 500;
        for (int ti = 0; ti < TiJump(3); ti++) {
            if (ti >= 0 && ti <= TiJump(0)) {
                // 动作下蹲2
                double s = ti * 1.0 / TiJump(0);
                double deltaH = downZ(i) * s;
                Mat34<double> tempVecFoot = pFoot;
                // tempVecFoot(1, 0) -= 0.05 * s;
                // tempVecFoot(1, 1) += 0.05 * s;
                // tempVecFoot(1, 2) -= 0.05 * s;
                // tempVecFoot(1, 3) += 0.05 * s;

                tempVecFoot.row(2).fill(-bodyHeight + deltaH);  // 0.18
                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);

            } else if (ti > TiJump(0) && ti <= TiJump(1)) {
                // 动作起跳2
                Mat34<double> tempVecFoot = pFoot;
                // tempVecFoot(1, 0) -= 0.05;
                // tempVecFoot(1, 1) += 0.05;
                // tempVecFoot(1, 2) -= 0.05;
                // tempVecFoot(1, 3) += 0.05;
                double s = ((ti - TiJump(0)) * 1.0 / (TiJump(1) - TiJump(0)));
                double deltaH = upZ(i) * s;  // 135度
                // tempVecFoot.row(2).fill(-bodyHeight + downZ - deltaH);  // 0.18 ~ 0.35
                tempVecFoot(2, 0) = (-bodyHeight + downZ(i) - deltaH * (1 - kBack));
                tempVecFoot(2, 1) = (-bodyHeight + downZ(i) - deltaH * (1 - kBack));

                tempVecFoot(2, 2) = (-bodyHeight + downZ(i) - deltaH * (1 + kBack));
                tempVecFoot(2, 3) = (-bodyHeight + downZ(i) - deltaH * (1 + kBack));
                // 转弯 和 平移
                double deltaRz = angleValue(i) * s;
                double deltaX = x(i) * s;
                double deltaY = y(i) * s;
                for (int i = 0; i < 4; i++) {
                    tempVecFoot.col(i) = mathFun_.Rz(tempVecFoot.col(i), -deltaRz);
                    tempVecFoot(0, i) -= deltaX;
                    tempVecFoot(1, i) -= deltaY;
                }

                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            } else if (ti > TiJump(1) && ti <= TiJump(2)) {
                // 恢复2
                Mat34<double> tempVecFoot = pFoot;
                // tempVecFoot(1, 0) -= 0.05;
                // tempVecFoot(1, 1) += 0.05;
                // tempVecFoot(1, 2) -= 0.05;
                // tempVecFoot(1, 3) += 0.05;
                double deltaH = downZ(i) * ((ti - TiJump(1)) * 1.0 / (TiJump(2) - TiJump(1)));  // 135度
                tempVecFoot.row(2).fill(-bodyHeight + downZ(i) - deltaH);                       // 0.18-0.28
                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            } else {
                // delay
                Mat34<double> tempVecFoot = pFoot;
                // tempVecFoot(1, 0) -= 0.05 * (1 - s);
                // tempVecFoot(1, 1) += 0.05 * (1 - s);
                // tempVecFoot(1, 2) -= 0.05 * (1 - s);
                // tempVecFoot(1, 3) += 0.05 * (1 - s);
                tempVecFoot.row(2).fill(-bodyHeight);  // 0.18-0.28

                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            }
            cmd.k2.fill(50.0);
            cmd.k1.fill(3.0);
            cmd.k3.fill(1.0);
            cmdQ_->push_back(cmd);
        }
    }
    // 发送给对应线程
    return cmdQ_->size();
}
/**
 * @description: 实现参数化太空步动作
 * @param timesIn 设置步数
 * @param stepLxIn 设置步距
 * @param zDownIn 设置步高上限
 * @param zUpIn 设置步高下限
 * @param delayIn 设置延时
 * @param kBackIn 设置步高回弹系数
 * @return {}
 */
int QrTask::moonWalk(int timesIn, double stepLxIn, double zDownIn, double zUpIn, double delayIn, double kBackIn)
{
    double stepLx = mathFun_.SetValueRange(stepLxIn, -0.15, 0.15, "walk stepLx", false);
    int N = mathFun_.SetValueRange(timesIn, 2, 20, "walk times", false) - 1;
    double delay = mathFun_.SetValueRange(delayIn, 0, 5, "delay limit", false);
    double downZ = mathFun_.SetValueRange(zDownIn, 0, 0.2, "down limit", false);
    double upZ = mathFun_.SetValueRange(zUpIn, 0, 0.39 - (bodyHeight - downZ), "up limit", false);
    double kBack = mathFun_.SetValueRange(kBackIn, -0.3, 0.3, "kBack limit", false);
    CmdVal2 cmd;
    Vec4<double> TiJump;
    TiJump << 100, 200, 300, 400;
    TiJump(3) = TiJump(2) + delay * 500;
    Mat34<double> tempVecFoot1 = pFoot;
    tempVecFoot1(0, 0) += stepLx;
    tempVecFoot1(0, 1) -= stepLx;
    tempVecFoot1(0, 2) -= stepLx;
    tempVecFoot1(0, 3) += stepLx;
    Mat34<double> tempVecFoot2 = pFoot;
    tempVecFoot2(0, 0) -= stepLx;
    tempVecFoot2(0, 1) += stepLx;
    tempVecFoot2(0, 2) += stepLx;
    tempVecFoot2(0, 3) -= stepLx;

    for (int Ni = 0; Ni <= N; Ni++) {
        for (int ti = 0; ti <= TiJump(3); ti++) {
            if (ti >= 0 && ti <= TiJump(0)) {
                // 动作下蹲
                Mat34<double> tempVecFoot;
                if (Ni == 0) {
                    tempVecFoot = pFoot;
                } else if ((Ni % 2) == 1) {
                    tempVecFoot = tempVecFoot1;
                } else {
                    tempVecFoot = tempVecFoot2;
                }

                double deltaH = downZ * (ti * 1.0 / TiJump(0));
                tempVecFoot.row(2).fill(-bodyHeight + deltaH);  // 0.18
                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);

            } else if (ti > TiJump(0) && ti <= TiJump(1)) {
                // 动作起跳
                Mat34<double> tempVecFoot;
                if (Ni == 0) {
                    tempVecFoot = pFoot;
                } else if ((Ni % 2) == 1) {
                    tempVecFoot = tempVecFoot1;
                } else {
                    tempVecFoot = tempVecFoot2;
                }
                double deltaH = upZ * ((ti - TiJump(0)) * 1.0 / (TiJump(1) - TiJump(0)));  // 135度
                // tempVecFoot.row(2).fill(-bodyHeight + downZ - deltaH);                     // 0.18 ~ 0.35
                tempVecFoot(2, 0) = (-bodyHeight + downZ - deltaH * (1 - kBack));
                tempVecFoot(2, 1) = (-bodyHeight + downZ - deltaH * (1 - kBack));

                tempVecFoot(2, 2) = (-bodyHeight + downZ - deltaH * (1 + kBack));
                tempVecFoot(2, 3) = (-bodyHeight + downZ - deltaH * (1 + kBack));
                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            } else if (ti > TiJump(1) && ti <= TiJump(2)) {
                // 恢复
                Mat34<double> tempVecFoot;
                if (Ni == 0) {
                    tempVecFoot = tempVecFoot1;
                } else if ((Ni % 2) == 1) {
                    if (Ni == N) {
                        tempVecFoot = pFoot;
                    } else {
                        tempVecFoot = tempVecFoot2;
                    }
                } else {
                    if (Ni == N) {
                        tempVecFoot = pFoot;
                    } else {
                        tempVecFoot = tempVecFoot1;
                    }
                }
                double deltaH = downZ * ((ti - TiJump(1)) * 1.0 / (TiJump(2) - TiJump(1)));  // 135度
                tempVecFoot.row(2).fill(-bodyHeight + downZ - deltaH);                       // 0.18-0.28
                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            } else if (ti > TiJump(2) && ti <= TiJump(3)) {
                // delay
                Mat34<double> tempVecFoot;
                if (Ni == 0) {
                    tempVecFoot = tempVecFoot1;
                } else if ((Ni % 2) == 1) {
                    if (Ni == N) {
                        tempVecFoot = pFoot;
                    } else {
                        tempVecFoot = tempVecFoot2;
                    }
                } else {
                    if (Ni == N) {
                        tempVecFoot = pFoot;
                    } else {
                        tempVecFoot = tempVecFoot1;
                    }
                }
                tempVecFoot.row(2).fill(-bodyHeight);  // 0.18-0.28
                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            }
            cmd.k2.fill(50.0);
            cmd.k1.fill(3.0);
            cmd.k3.fill(1.0);
            cmdQ_->push_back(cmd);
        }
    }
    return cmdQ_->size();
}
int QrTask::moonWalk2(Mat34<double> pFootInA, Mat34<double> pFootInB, int timesIn, double zDownIn, double zUpIn, double delayIn, double kBackIn)
{
    int N = mathFun_.SetValueRange(timesIn, 2, 20, "walk times", false) - 1;
    double delay = mathFun_.SetValueRange(delayIn, 0, 5, "delay limit", false);
    double downZ = mathFun_.SetValueRange(zDownIn, 0, 0.2, "down limit", false);
    double upZ = mathFun_.SetValueRange(zUpIn, 0, 0.39 - (bodyHeight - downZ), "up limit", false);
    double kBack = mathFun_.SetValueRange(kBackIn, -0.3, 0.3, "kBack limit", false);
    CmdVal2 cmd;
    Vec4<double> TiJump;
    TiJump << 100, 200, 300, 400;
    TiJump(3) = TiJump(2) + delay * 500;
    Mat34<double> tempVecFoot1 = pFoot + pFootInA;
    Mat34<double> tempVecFoot2 = pFoot + pFootInB;

    for (int Ni = 0; Ni <= N; Ni++) {
        for (int ti = 0; ti <= TiJump(3); ti++) {
            if (ti >= 0 && ti <= TiJump(0)) {
                // 动作下蹲
                Mat34<double> tempVecFoot;
                if (Ni == 0) {
                    tempVecFoot = pFoot;
                } else if ((Ni % 2) == 1) {
                    tempVecFoot = tempVecFoot1;
                } else {
                    tempVecFoot = tempVecFoot2;
                }

                double deltaH = downZ * (ti * 1.0 / TiJump(0));
                tempVecFoot.row(2).fill(-bodyHeight + deltaH);  // 0.18
                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);

            } else if (ti > TiJump(0) && ti <= TiJump(1)) {
                // 动作起跳
                Mat34<double> tempVecFoot;
                if (Ni == 0) {
                    tempVecFoot = pFoot;
                } else if ((Ni % 2) == 1) {
                    tempVecFoot = tempVecFoot1;
                } else {
                    tempVecFoot = tempVecFoot2;
                }
                double deltaH = upZ * ((ti - TiJump(0)) * 1.0 / (TiJump(1) - TiJump(0)));  // 135度
                // tempVecFoot.row(2).fill(-bodyHeight + downZ - deltaH);                     // 0.18 ~ 0.35
                tempVecFoot(2, 0) = (-bodyHeight + downZ - deltaH * (1 - kBack));
                tempVecFoot(2, 1) = (-bodyHeight + downZ - deltaH * (1 - kBack));

                tempVecFoot(2, 2) = (-bodyHeight + downZ - deltaH * (1 + kBack));
                tempVecFoot(2, 3) = (-bodyHeight + downZ - deltaH * (1 + kBack));
                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            } else if (ti > TiJump(1) && ti <= TiJump(2)) {
                // 恢复
                Mat34<double> tempVecFoot;
                if (Ni == 0) {
                    tempVecFoot = tempVecFoot1;
                } else if ((Ni % 2) == 1) {
                    if (Ni == N) {
                        tempVecFoot = pFoot;
                    } else {
                        tempVecFoot = tempVecFoot2;
                    }
                } else {
                    if (Ni == N) {
                        tempVecFoot = pFoot;
                    } else {
                        tempVecFoot = tempVecFoot1;
                    }
                }
                double deltaH = downZ * ((ti - TiJump(1)) * 1.0 / (TiJump(2) - TiJump(1)));  // 135度
                tempVecFoot.row(2).fill(-bodyHeight + downZ - deltaH);                       // 0.18-0.28
                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            } else if (ti > TiJump(2) && ti <= TiJump(3)) {
                // delay
                Mat34<double> tempVecFoot;
                if (Ni == 0) {
                    tempVecFoot = tempVecFoot1;
                } else if ((Ni % 2) == 1) {
                    if (Ni == N) {
                        tempVecFoot = pFoot;
                    } else {
                        tempVecFoot = tempVecFoot2;
                    }
                } else {
                    if (Ni == N) {
                        tempVecFoot = pFoot;
                    } else {
                        tempVecFoot = tempVecFoot1;
                    }
                }
                tempVecFoot.row(2).fill(-bodyHeight);  // 0.18-0.28
                cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
                cmd.torq.setZero();
                cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
            }
            cmd.k2.fill(50.0);
            cmd.k1.fill(3.0);
            cmd.k3.fill(1.0);
            cmdQ_->push_back(cmd);
        }
    }
    return cmdQ_->size();
}

/**
 * @description: 新版拜年动作，稳定性提高
 * @param angleIn
 * @param periodIn
 * @param showTimeIn
 * @return {}
 */
int QrTask::takeBow2()
{
    CmdVal2 cmd;
    Vec7<double> Ti, deltaTi;
    deltaTi << 200, 200, 600, 300, 2000, 100, 1000;  // 最后一个没用到
    Ti(0) = deltaTi(0);
    Ti(1) = deltaTi(0) + deltaTi(1);
    Ti(2) = deltaTi(0) + deltaTi(1) + deltaTi(2);
    Ti(3) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3);
    Ti(4) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4);
    Ti(5) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4) + deltaTi(5);
    Ti(6) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4) + deltaTi(5) + deltaTi(6);

    // 开始计算
    Mat34<double> tmpCmd0, tmpCmd1, tmpCmd2, tmpCmd3, tmpCmd4, tmpCmd5, tmpCmd6;
    tmpCmd0 = mathFun_.InverseKinematics_new(pFoot, pRoll);
    for (int ti = 0; ti < Ti(6); ti++) {
        if (ti > 0 && ti <= Ti(0)) {
            double s = ti * 1.0 / Ti(0);
            cmd.alpha = tmpCmd0;
            cmd.alpha(2, 0) += 0.5 * s;
            cmd.alpha(2, 1) += 0.5 * s;
            cmd.alpha(2, 2) += 0.5 * s;
            cmd.alpha(2, 3) += 0.5 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd1 = cmd.alpha;
        } else if (ti > Ti(0) && ti <= Ti(1)) {
            double s = (ti - Ti(0)) * 1.0 / (Ti(1) - Ti(0));
            cmd.alpha = tmpCmd1;
            cmd.alpha(1, 0) += 0.8 * s;
            cmd.alpha(1, 1) += 0.8 * s;
            cmd.alpha(2, 0) -= 0.8 * s;
            cmd.alpha(2, 1) -= 0.8 * s;
            cmd.alpha(1, 2) += 0.9 * s;
            cmd.alpha(1, 3) += 0.9 * s;
            cmd.alpha(2, 2) -= 0.9 * s;
            cmd.alpha(2, 3) -= 0.9 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd2 = cmd.alpha;
        } else if (ti > Ti(1) && ti <= Ti(2)) {
            double s = (ti - Ti(1)) * 1.0 / (Ti(2) - Ti(1));
            cmd.alpha = tmpCmd2;
            cmd.alpha(1, 0) -= 1.3 * s;
            cmd.alpha(1, 1) -= 1.3 * s;
            cmd.alpha(2, 0) -= 0.3 * s;
            cmd.alpha(2, 1) -= 0.3 * s;
            cmd.alpha(1, 2) -= 0.7 * s;
            cmd.alpha(1, 3) -= 0.7 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd3 = cmd.alpha;
        } else if (ti > Ti(2) && ti <= Ti(3)) {
            double s = (ti - Ti(2)) * 1.0 / (Ti(3) - Ti(2));
            cmd.alpha = tmpCmd3;
            cmd.alpha(0, 0) += 0.3 * s;
            cmd.alpha(0, 1) -= 0.3 * s;
            cmd.alpha(1, 0) += 0.5 * s;
            cmd.alpha(1, 1) += 0.5 * s;
            cmd.alpha(2, 0) += 1.0 * s;
            cmd.alpha(2, 1) += 1.0 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd4 = cmd.alpha;
        } else if (ti > Ti(3) && ti <= Ti(4)) {
            double s = (ti - Ti(3)) * 1.0 / (Ti(4) - Ti(3));
            cmd.alpha = tmpCmd4;
            cmd.alpha(2, 0) += 0.7 * sin(6.28 * 10 * s);
            cmd.alpha(2, 1) += 0.7 * sin(6.28 * 10 * s);
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd5 = cmd.alpha;
        } else if (ti > Ti(4) && ti <= Ti(5)) {
            double s = (ti - Ti(4)) * 1.0 / (Ti(5) - Ti(4));
            cmd.alpha = tmpCmd5 + (tmpCmd0 - tmpCmd5) * s;
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha) * s;
        }
        cmd.k2.fill(50.0);
        cmd.k1.fill(3.0);
        cmd.k3.fill(1.0);
        cmdQ_->push_back(cmd);
    }
    return cmdQ_->size();
}
/**
 * @description: 实现参数化拜年动作
 * @param angleIn 动作幅度和角度
 * @param periodIn 动作周期
 * @param showTimeIn 动作持续时间
 * @return {}
 */
int QrTask::takeBow(Vec6<double> angleIn, Vec6<double> periodIn, double showTimeIn)
{
    // double angle = mathFun_.SetValueRange(angleIn, -1.0, 1.0, "angle limit", false);
    Vec6<double> angle = angleIn;
    angle(0) = mathFun_.SetValueRange(angleIn(0), -1.0, 1.0, "angle roll limit", false);
    angle(1) = mathFun_.SetValueRange(angleIn(1), -1.0, 1.0, "angle roll limit", false);

    angle(2) = mathFun_.SetValueRange(angleIn(2), -1.0, 1.0, "angle hip limit", false);
    angle(3) = mathFun_.SetValueRange(angleIn(3), -1.0, 1.0, "angle hip limit", false);

    angle(4) = mathFun_.SetValueRange(angleIn(4), -1.0, 1.0, "angle knee limit", false);
    angle(5) = mathFun_.SetValueRange(angleIn(5), -1.0, 1.0, "angle knee limit", false);

    Vec6<double> period = periodIn;
    period(0) = mathFun_.SetValueRange(periodIn(0), 0, 20, "frequence limit", false);
    period(1) = mathFun_.SetValueRange(periodIn(1), 0, 20, "frequence limit", false);

    period(2) = mathFun_.SetValueRange(periodIn(2), 0, 20, "frequence limit", false);
    period(3) = mathFun_.SetValueRange(periodIn(3), 0, 20, "frequence limit", false);

    period(4) = mathFun_.SetValueRange(periodIn(4), 0, 20, "frequence limit", false);
    period(5) = mathFun_.SetValueRange(periodIn(5), 0, 20, "frequence limit", false);

    double showTime = mathFun_.SetValueRange(showTimeIn, 2, 10, "show time limie", false);

    CmdVal2 cmd;
    Vec6<double> Ti;
    Ti << 500, 530, 700, 1200, 1200 + showTime * 500, 3500;
    Ti(5) = Ti(4) + 500;
    pFoot.row(2).fill(-0.13);
    // 开始计算
    Mat34<double> tmpCmd1, tmpCmd2, tmpCmd3, tmpCmd4, tmpCmd5;
    tmpCmd1 = mathFun_.InverseKinematics_new(pFoot, pRoll);

    tmpCmd1(1, 2) -= 0.257;  // 越大起跳更猛
    tmpCmd1(1, 3) -= 0.257;

    tmpCmd1(0, 2) -= 0.3;
    tmpCmd1(0, 3) += 0.3;
    // 动作1
    tmpCmd2 = tmpCmd1;
    tmpCmd2(2, 0) -= 1.8;
    tmpCmd2(2, 1) -= 1.8;

    tmpCmd2(1, 0) += 0.5;
    tmpCmd2(1, 1) += 0.5;

    tmpCmd3 = tmpCmd2;

    tmpCmd3(1, 2) -= 0.95;  // 改变后仰角度，越大越后仰
    tmpCmd3(1, 3) -= 0.95;

    tmpCmd3(1, 0) -= 0.8;
    tmpCmd3(1, 1) -= 0.8;

    if (GetQrParam().model.back() == QrModel::gazebo) {
        tmpCmd3(2, 2) -= 0.35;
    } else {
        tmpCmd3(2, 2) -= 0.375;  // 这里非对称是因为实物电池包在右,不正
    }

    tmpCmd3(2, 3) -= 0.35;

    tmpCmd4 = tmpCmd3;

    tmpCmd4(1, 0) += 0.2;
    tmpCmd4(1, 1) += 0.2;

    tmpCmd4(2, 0) += 1.3;
    tmpCmd4(2, 1) += 1.3;

    tmpCmd4(0, 0) += 0.4;
    tmpCmd4(0, 1) -= 0.4;

    tmpCmd5 = tmpCmd4;

    for (int ti = 0; ti < Ti(5); ti++) {
        if (ti >= 0 && ti <= Ti(0)) {
            // 动作1
            Mat34<double> tempVecFoot = pFoot;
            double desireH = 0.13;
            double deltaH = (bodyHeight - desireH) * (ti * 1.0 / Ti(0));
            tempVecFoot.row(2).fill(-bodyHeight + deltaH);
            cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(0) && ti <= Ti(1)) {
            // 动作2
            cmd.alpha = tmpCmd2;
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(1) && ti <= Ti(2)) {
            // 动作3
            cmd.alpha = tmpCmd2 + (tmpCmd3 - tmpCmd2) * ((ti - Ti(1)) * 1.0 / (Ti(2) - Ti(1)));
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(2) && ti <= Ti(3)) {
            // 动作4
            cmd.alpha = tmpCmd3 + (tmpCmd4 - tmpCmd3) * ((ti - Ti(2)) * 1.0 / (Ti(3) - Ti(2)));
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(3) && ti <= Ti(4)) {
            // 动作5
            tmpCmd5 = tmpCmd4;
            Vec6<double> s;
            s(0) = period(0) * 6.28 * (ti - Ti(3)) / (Ti(4) - Ti(3));
            s(1) = period(1) * 6.28 * (ti - Ti(3)) / (Ti(4) - Ti(3));

            s(2) = period(2) * 6.28 * (ti - Ti(3)) / (Ti(4) - Ti(3));
            s(3) = period(3) * 6.28 * (ti - Ti(3)) / (Ti(4) - Ti(3));

            s(4) = period(4) * 6.28 * (ti - Ti(3)) / (Ti(4) - Ti(3));
            s(5) = period(5) * 6.28 * (ti - Ti(3)) / (Ti(4) - Ti(3));

            tmpCmd5(0, 0) += sin(s(0)) * angle(0);
            tmpCmd5(0, 1) -= sin(s(1)) * angle(1);

            tmpCmd5(1, 0) += sin(s(2)) * angle(2);
            tmpCmd5(1, 1) += sin(s(3)) * angle(3);

            tmpCmd5(2, 0) += sin(s(4)) * angle(4);
            tmpCmd5(2, 1) += sin(s(5)) * angle(5);

            cmd.alpha = tmpCmd5;
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(4) && ti <= Ti(5)) {
            Mat34<double> tempVecFoot = pFoot;
            tempVecFoot.row(2).fill(-bodyHeight);
            Mat34<double> qTemp = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.alpha = tmpCmd4 + (qTemp - tmpCmd4) * ((ti - Ti(4)) * 1.0 / (Ti(5) - Ti(4)));  // 恢复
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else {
            Mat34<double> tempVecFoot = pFoot;
            tempVecFoot.row(2).fill(-bodyHeight);
            cmd.alpha = mathFun_.InverseKinematics_new(tempVecFoot, pRoll);
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
        }
        cmd.k2.fill(50.0);
        cmd.k1.fill(3.0);
        cmd.k3.fill(1.0);
        cmdQ_->push_back(cmd);
    }
    return cmdQ_->size();
}
/**
 * @description: 实现参数化握手动作
 * @param angleIn 手摆动的关节角度
 * @param periodIn 摆动周期
 * @param showTimeIn 动作持续时间
 * @return {}
 */
int QrTask::handShake(Vec3<double> angleIn, double periodIn, double showTimeIn)
{
    Vec3<double> angle;
    angle(0) = mathFun_.SetValueRange(angleIn(0), 0, 30, "roll limit", false);
    angle(1) = mathFun_.SetValueRange(angleIn(1), 0, 30, "hip limit", false);
    angle(2) = mathFun_.SetValueRange(angleIn(2), 0, 30, "knee limit", false);

    double period = mathFun_.SetValueRange(periodIn, 1, 10, "period limit", false);

    double showTime = mathFun_.SetValueRange(showTimeIn, 1, 10, "show time limit", false);

    CmdVal2 cmd;
    Vec6<double> Ti;
    Ti << 500, 800, showTime * 500 + 800, 2800, 3000, 3200;
    Ti(3) = Ti(2) + 1000;
    Ti(4) = Ti(3) + 200;
    Ti(5) = Ti(4) + 200;
    // 开始计算
    Mat34<double> qStand = mathFun_.InverseKinematics_new(pFoot, pRoll);

    Mat34<double> tmpCmd1, tmpCmd2, tmpCmd3;
    // 动作1
    tmpCmd1 = qStand;

    tmpCmd1(0, 0) += 0.31;
    tmpCmd1(0, 1) -= 0.31;

    tmpCmd1(1, 2) -= 0.5;
    tmpCmd1(1, 3) -= 0.5;

    tmpCmd1(2, 0) -= 0.9;
    tmpCmd1(2, 1) -= 0.9;
    tmpCmd1(2, 2) += 0.8;
    tmpCmd1(2, 3) += 0.8;

    // 动作2
    tmpCmd2 = tmpCmd1;
    tmpCmd2(2, 0) += 1.4;
    tmpCmd2(1, 0) += 0.8;

    for (int ti = 0; ti < Ti(5); ti++) {
        if (ti >= 0 && ti <= Ti(0)) {
            cmd.alpha = qStand + (tmpCmd1 - qStand) * (ti * 1.0 / Ti(0));
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(0) && ti <= Ti(1)) {
            cmd.alpha = tmpCmd1 + (tmpCmd2 - tmpCmd1) * ((ti - Ti(0)) * 1.0 / (Ti(1) - Ti(0)));
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(1) && ti <= Ti(2)) {
            //   动作3
            tmpCmd3 = tmpCmd2;
            double s = period * 6.28 * (ti - Ti(1)) / (Ti(2) - Ti(1));

            tmpCmd3(0, 0) += sin(s) * angle(0);
            tmpCmd3(1, 0) += sin(s) * angle(1);
            tmpCmd3(2, 0) += sin(s) * angle(2);

            cmd.alpha = tmpCmd3;
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(2) && ti <= Ti(3)) {
            cmd.alpha = tmpCmd2;  // 保持不动；
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(3) && ti <= Ti(4)) {
            cmd.alpha = tmpCmd2 + (tmpCmd1 - tmpCmd2) * ((ti - Ti(3)) * 1.0 / (Ti(4) - Ti(3)));  // 落腿
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(4) && ti <= Ti(5)) {
            cmd.alpha = tmpCmd1 + (qStand - tmpCmd1) * ((ti - Ti(4)) * 1.0 / (Ti(5) - Ti(4)));  // 恢复
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else {
            cmd.alpha = qStand;
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha);
        }
        cmd.k2.fill(50.0);
        cmd.k1.fill(3.0);
        cmd.k3.fill(1.0);
        cmdQ_->push_back(cmd);
    }
    return cmdQ_->size();
}
int QrTask::handShake2()
{
    CmdVal2 cmd;
    Vec7<double> Ti, deltaTi;
    deltaTi << 200, 200, 600, 300, 2000, 100, 1000;  // 最后一个没用到
    Ti(0) = deltaTi(0);
    Ti(1) = deltaTi(0) + deltaTi(1);
    Ti(2) = deltaTi(0) + deltaTi(1) + deltaTi(2);
    Ti(3) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3);
    Ti(4) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4);
    Ti(5) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4) + deltaTi(5);
    Ti(6) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4) + deltaTi(5) + deltaTi(6);

    // 开始计算
    Mat34<double> tmpCmd0, tmpCmd1, tmpCmd2, tmpCmd3, tmpCmd4, tmpCmd5, tmpCmd6;
    tmpCmd0 = mathFun_.InverseKinematics_new(pFoot, pRoll);
    for (int ti = 0; ti < Ti(6); ti++) {
        if (ti > 0 && ti <= Ti(0)) {
            double s = ti * 1.0 / Ti(0);
            cmd.alpha = tmpCmd0;
            cmd.alpha(2, 0) += 0.5 * s;
            cmd.alpha(2, 1) += 0.5 * s;
            cmd.alpha(2, 2) += 0.5 * s;
            cmd.alpha(2, 3) += 0.5 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd1 = cmd.alpha;
        } else if (ti > Ti(0) && ti <= Ti(1)) {
            double s = (ti - Ti(0)) * 1.0 / (Ti(1) - Ti(0));
            cmd.alpha = tmpCmd1;
            cmd.alpha(1, 0) += 0.8 * s;
            cmd.alpha(1, 1) += 0.8 * s;
            cmd.alpha(2, 0) -= 0.8 * s;
            cmd.alpha(2, 1) -= 0.8 * s;
            cmd.alpha(1, 2) += 1.0 * s;
            cmd.alpha(1, 3) += 1.0 * s;
            cmd.alpha(2, 2) -= 1.0 * s;
            cmd.alpha(2, 3) -= 1.0 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd2 = cmd.alpha;
        } else if (ti > Ti(1) && ti <= Ti(2)) {
            double s = (ti - Ti(1)) * 1.0 / (Ti(2) - Ti(1));
            cmd.alpha = tmpCmd2;
            cmd.alpha(1, 0) -= 0.9 * s;
            cmd.alpha(1, 1) -= 0.9 * s;
            // cmd.alpha(2, 0) -= 1.0 * s;
            // cmd.alpha(2, 1) -= 1.0 * s;
            cmd.alpha(1, 2) -= 0.6 * s;
            cmd.alpha(1, 3) -= 0.6 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd3 = cmd.alpha;
        } else if (ti > Ti(2) && ti <= Ti(3)) {
            double s = (ti - Ti(2)) * 1.0 / (Ti(3) - Ti(2));
            cmd.alpha = tmpCmd3;
            cmd.alpha(1, 0) += 1.5 * s;
            cmd.alpha(2, 0) += 0.5 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd4 = cmd.alpha;
        } else if (ti > Ti(3) && ti <= Ti(4)) {
            double s = (ti - Ti(3)) * 1.0 / (Ti(4) - Ti(3));
            cmd.alpha = tmpCmd4;
            cmd.alpha(2, 0) += 0.3 * sin(6.28 * 8 * s);
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd5 = cmd.alpha;
        } else if (ti > Ti(4) && ti <= Ti(5)) {
            double s = (ti - Ti(4)) * 1.0 / (Ti(5) - Ti(4));
            cmd.alpha = tmpCmd5 + (tmpCmd0 - tmpCmd5) * s;
            cmd.torq.setZero();
            cmd.blta = mathFun_.JacobianT(tmpForceFoot, cmd.alpha) * s;
        }
        cmd.k2.fill(50.0);
        cmd.k1.fill(3.0);
        cmd.k3.fill(1.0);
        cmdQ_->push_back(cmd);
    }
    return cmdQ_->size();
}
int QrTask::pushUp(int pushNum)
{
    CmdVal2 cmd;
    Vec7<double> Ti, deltaTi;
    deltaTi << pushNum * 500 - 250, 100, 100, 100, (pushNum + 1) * 500, 600, 300;
    Ti(0) = deltaTi(0);
    Ti(1) = deltaTi(0) + deltaTi(1);
    Ti(2) = deltaTi(0) + deltaTi(1) + deltaTi(2);
    Ti(3) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3);
    Ti(4) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4);
    Ti(5) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4) + deltaTi(5);
    Ti(6) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4) + deltaTi(5) + deltaTi(6);

    // 开始计算
    Mat34<double> tmpCmd0, tmpCmd1, tmpCmd2, tmpCmd3, tmpCmd4, tmpCmd5, tmpCmd6;
    for (int ti = 0; ti < Ti(6); ti++) {
        if (ti >= 0 && ti <= Ti(0)) {
            double deltaH = 0.15 * ((cos(3.14 + 6.28 / 500 * ti) + 1.0) * 0.5);
            Mat34<double> pFootTmp = pFoot;
            pFootTmp.row(2).fill(-bodyHeight + deltaH);
            cmd.alpha = mathFun_.InverseKinematics_new(pFootTmp, pRoll);
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(0) && ti <= Ti(1)) {
            Mat34<double> pFootTmp = pFoot;
            pFootTmp.row(2).fill(-bodyHeight + 0.15);
            cmd.alpha = mathFun_.InverseKinematics_new(pFootTmp, pRoll);
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(1) && ti <= Ti(2)) {
            Mat34<double> pFootTmp = pFoot;
            double deltaH = 0.15 * (ti - Ti(1)) / (Ti(2) - Ti(1));
            pFootTmp.row(2).fill(-bodyHeight + 0.15 - deltaH);
            cmd.alpha = mathFun_.InverseKinematics_new(pFootTmp, pRoll);
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(2) && ti <= Ti(3)) {
            Mat34<double> pFootTmp = pFoot;
            double s = (ti - Ti(2)) / (Ti(3) - Ti(2));
            cmd.alpha = mathFun_.InverseKinematics_new(pFootTmp, pRoll);
            cmd.alpha(0, 0) -= s * 0.2;
            cmd.alpha(0, 1) += s * 0.2;
            cmd.alpha(1, 2) -= s * 0.9;
            cmd.alpha(1, 3) -= s * 0.9;
            cmd.alpha(2, 2) -= s * 0.7;
            cmd.alpha(2, 3) -= s * 0.7;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd3 = cmd.alpha;
        } else if (ti > Ti(3) && ti <= Ti(4)) {
            double s = (ti - Ti(3)) / (Ti(4) - Ti(3));
            tmpCmd4 = tmpCmd3;
            tmpCmd4(2, 0) += 0.6 * sin((pushNum + 0.25) * 6.28 * s);
            tmpCmd4(2, 1) += 0.6 * sin((pushNum + 0.25) * 6.28 * s);
            cmd.alpha = tmpCmd4;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd5 = tmpCmd4;
        } else if (ti > Ti(4) && ti <= Ti(5)) {
            double s = (ti - Ti(4)) / (Ti(5) - Ti(4));
            Mat34<double> pFootTmp = pFoot;
            pFootTmp.row(2).fill(-bodyHeight + 0.15);
            Mat34<double> qTmp = mathFun_.InverseKinematics_new(pFootTmp, pRoll);
            cmd.alpha = tmpCmd5 + (qTmp - tmpCmd5) * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd6 = cmd.alpha;
        } else if (ti > Ti(5) && ti <= Ti(6)) {
            double s = (ti - Ti(5)) / (Ti(6) - Ti(5));
            // Mat34<double> pFootTmp = pFoot;
            // double deltaH = 0.15 * (1 - (ti - Ti(5)) / (Ti(6) - Ti(5)));
            // pFootTmp.row(2).fill(-bodyHeight + deltaH);
            Mat34<double> qTmp = mathFun_.InverseKinematics_new(pFoot, pRoll);
            cmd.alpha = tmpCmd6 + (qTmp - tmpCmd6) * s;
            // cmd.alpha = mathFun_.InverseKinematics_new(pFoot, pRoll);
            cmd.torq.setZero();
            cmd.blta.setZero();
        }
        cmd.k2.fill(50.0);
        cmd.k1.fill(3.0);
        cmd.k3.fill(1.0);
        cmdQ_->push_back(cmd);
    }
    return cmdQ_->size();
}

int QrTask::rpyDance()
{
    CmdVal2 cmd;
    Vec8<double> Ti, deltaTi;
    deltaTi << 2000, 2500, 2000, 200, 200, 600, 300, 3000;
    Ti(0) = deltaTi(0);
    Ti(1) = deltaTi(0) + deltaTi(1);
    Ti(2) = deltaTi(0) + deltaTi(1) + deltaTi(2);
    Ti(3) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3);
    Ti(4) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4);
    Ti(5) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4) + deltaTi(5);
    Ti(6) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4) + deltaTi(5) + deltaTi(6);
    Ti(7) = deltaTi(0) + deltaTi(1) + deltaTi(2) + deltaTi(3) + deltaTi(4) + deltaTi(5) + deltaTi(6) + deltaTi(7);

    // 开始计算
    Mat34<double> tmpCmd0, tmpCmd1, tmpCmd2, tmpCmd3, tmpCmd4, tmpCmd5, tmpCmd6;
    for (int ti = 0; ti < Ti(7); ti++) {
        if (ti >= 0 && ti <= Ti(0)) {
            double s = ti * 1.0 / Ti(0);
            Mat34<double> pFootTmp = pFoot;
            for (int i = 0; i < 4; i++) {
                pFootTmp.col(i) = mathFun_.Rx(pFootTmp.col(i), -0);  // 注意：存在角度限制，否则容易出现超过工作空间
                pFootTmp.col(i) = mathFun_.Ry(pFootTmp.col(i), -0.5 * sin(3 * 6.28 * s));
                pFootTmp.col(i) = mathFun_.Rz(pFootTmp.col(i), -0);
            }
            cmd.alpha = mathFun_.InverseKinematics_new(pFootTmp, pRoll);
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(0) && ti <= Ti(1)) {
            double s = (ti - Ti(0)) * 1.0 / (Ti(1) - Ti(0));
            Mat34<double> pFootTmp = pFoot;
            for (int i = 0; i < 4; i++) {
                // pFootTmp(2, i) += -sin(4 * 6.28 * s) * 0.02;
                pFootTmp.col(i) = mathFun_.Rx(pFootTmp.col(i), -0);  // 注意：存在角度限制，否则容易出现超过工作空间
                pFootTmp.col(i) = mathFun_.Ry(pFootTmp.col(i), 0);
                pFootTmp.col(i) = mathFun_.Rz(pFootTmp.col(i), -0.5 * sin(4 * 6.28 * s));
            }
            cmd.alpha = mathFun_.InverseKinematics_new(pFootTmp, pRoll);
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(1) && ti <= Ti(2)) {
            double s = (ti - Ti(1)) * 1.0 / (Ti(2) - Ti(1));
            Mat34<double> pFootTmp = pFoot;
            for (int i = 0; i < 4; i++) {
                // pFootTmp(2, i) += -sin(3 * 6.28 * s) * 0.02;
                pFootTmp.col(i) = mathFun_.Rx(pFootTmp.col(i), -0.3 * sin(6 * 6.28 * s));  // 注意：存在角度限制，否则容易出现超过工作空间
                pFootTmp.col(i) = mathFun_.Ry(pFootTmp.col(i), 0);
                pFootTmp.col(i) = mathFun_.Rz(pFootTmp.col(i), 0);
            }
            cmd.alpha = mathFun_.InverseKinematics_new(pFootTmp, pRoll);
            cmd.torq.setZero();
            cmd.blta.setZero();
        } else if (ti > Ti(2) && ti <= Ti(3)) {
            double s = (ti - Ti(2)) * 1.0 / (Ti(3) - Ti(2));
            cmd.alpha = mathFun_.InverseKinematics_new(pFoot, pRoll);
            cmd.alpha(2, 0) += 0.5 * s;
            cmd.alpha(2, 1) += 0.5 * s;
            cmd.alpha(2, 2) += 0.5 * s;
            cmd.alpha(2, 3) += 0.5 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd3 = cmd.alpha;
        } else if (ti > Ti(3) && ti <= Ti(4)) {
            double s = (ti - Ti(3)) * 1.0 / (Ti(4) - Ti(3));
            cmd.alpha = tmpCmd3;
            cmd.alpha(1, 0) += 0.8 * s;
            cmd.alpha(1, 1) += 0.8 * s;
            cmd.alpha(2, 0) -= 0.8 * s;
            cmd.alpha(2, 1) -= 0.8 * s;
            cmd.alpha(1, 2) += 0.9 * s;
            cmd.alpha(1, 3) += 0.9 * s;
            cmd.alpha(2, 2) -= 0.9 * s;
            cmd.alpha(2, 3) -= 0.9 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd4 = cmd.alpha;
        } else if (ti > Ti(4) && ti <= Ti(5)) {
            double s = (ti - Ti(4)) * 1.0 / (Ti(5) - Ti(4));
            cmd.alpha = tmpCmd4;
            cmd.alpha(1, 0) -= 1.3 * s;
            cmd.alpha(1, 1) -= 1.3 * s;
            cmd.alpha(2, 0) -= 0.3 * s;
            cmd.alpha(2, 1) -= 0.3 * s;
            cmd.alpha(1, 2) -= 0.7 * s;
            cmd.alpha(1, 3) -= 0.7 * s;
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd5 = cmd.alpha;
        } else if (ti > Ti(5) && ti <= Ti(6)) {
            double s = (ti - Ti(5)) * 1.0 / (Ti(6) - Ti(5));
            cmd.alpha = tmpCmd5;
            cmd.alpha(0, 0) += 0.3 * s;
            cmd.alpha(0, 1) -= 0.3 * s;
            cmd.alpha(1, 0) += 0.5 * s;
            cmd.alpha(1, 1) += 0.5 * s;
            cmd.alpha(2, 0) += 1.0 * s;
            cmd.alpha(2, 1) += 1.0 * s;
            // cmd.alpha(0, 0) += 0.4 * sin(6.28 * 8 * s);
            // cmd.alpha(0, 1) -= 0.4 * sin(6.28 * 8 * s);
            cmd.torq.setZero();
            cmd.blta.setZero();
            tmpCmd6 = cmd.alpha;
        } else if (ti > Ti(6) && ti < Ti(7)) {
            double s = (ti - Ti(6)) * 1.0 / (Ti(7) - Ti(6));
            cmd.alpha = tmpCmd6;
            cmd.alpha(0, 0) += 0.4 * sin(6.28 * 8 * s);
            cmd.alpha(0, 1) -= 0.4 * sin(6.28 * 8 * s);
        }
        cmd.k2.fill(50.0);
        cmd.k1.fill(3.0);
        cmd.k3.fill(1.0);
        cmdQ_->push_back(cmd);
    }
    return cmdQ_->size();
}
RetState QrTask::SendCmd(int len)
{
    RetState resp = RetState::timeout;
    RpcTrySend("qr::SetMotorCmdQ", cmdQ_, &resp);
    if (resp != RetState::ok) {
        LOG_INFO("dance error {}", TimerTools::GetNowTickMs());
        return resp;
    }

    // 最长10秒等待
    auto ret = MsgBlockRecv<RetState>("qr::SetMotorCmdQComplete", this, len * 1.2 * 2);
    if (ret) {
        LOG_INFO("dance end {}", TimerTools::GetNowTickMs());
        resp = ret.value();
    } else {
        resp = RetState::timeout;
        LOG_INFO("dance timeout {}", TimerTools::GetNowTickMs());
    }
    return resp;
}
