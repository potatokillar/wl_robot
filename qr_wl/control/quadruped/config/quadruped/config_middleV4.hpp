
#pragma once

#include "config_middleV3.hpp"

struct Config_middleV4 : public Config_middleV3
{
    Config_middleV4()
    {
        model.insert(QrModel::middleV4);

        tStance[WalkMode::normal] = 0.26;  // 0.3
        tSwing[WalkMode::normal] = 0.2;    // 0.24已经出现左右不是特别稳的情况
        stepZ[WalkMode::normal] = 0.1;

        tStance[WalkMode::climb] = 0.36;  // 0.6
        tSwing[WalkMode::climb] = 0.24;   // 0.3
        stepZ[WalkMode::climb] = 0.26;    // 0.2

        tStance[WalkMode::light] = 0.38;
        tSwing[WalkMode::light] = 0.24;
        stepZ[WalkMode::light] = 0.13;

        // standUpTime = 1.3;
        // lieDownTime = 1.3;

        DOG_M = 33;  // 由于攀爬模式，抬腿高度变高了，高度有损失

        // BODY_Ix = 101509190.7 * 10e-9;
        // BODY_Iy = 431323650.64 * 10e-9;
        // BODY_Iz = 497082269.97 * 10e-9;

        // bodyI << BODY_Ix, 0.0, 0.0, 0.0, BODY_Iy, 0.0, 0.0, 0.0, BODY_Iz;

        BODY_Lx = 0.628;
        BODY_Ly = 0.14;

        LEG_L0 = 0.114;
        //  LEG_L1 = 0.3;
        // LEG_L2 = 0.3 + 0.019;

        // kneeRatio = 1;
        bodyHeight = 0.45;
        paramRange.height.max = 0.55;
        paramRange.height.min = 0.26;
        // paramRange.LinearVelocity.x.max = 0.8;
        // paramRange.LinearVelocity.x.min = -0.4;
        paramRange.LinearVelocity.y.max = 0.3;
        paramRange.LinearVelocity.y.min = -0.3;
        //  paramRange.AngularVelocity.z.max = 1.0;
        //  paramRange.AngularVelocity.z.min = -1.0;
        //  paramRange.Pose.roll.max = 0.2;
        //  paramRange.Pose.roll.min = -0.2;
        //  paramRange.Pose.pitch.max = 0.3;
        //  paramRange.Pose.pitch.min = -0.3;
        //  paramRange.Pose.yaw.max = 0.3;
        //   paramRange.Pose.yaw.min = -0.3;
        paramRange.loadMass.max = 15.0;
        paramRange.loadMass.min = 0.0;
        //  fZmax = 500;
        // kW = 0.1;
        //   pFootzMin = -0.19;
        //  ikLegRange.min = 0.125;
        //   ikLegRange.max = 0.59;
        //  qCmdStandUpStep2 << 0, 0, 0, 0, -80, -80, -80, -80, 150, 150, 150, 150;
        //   qCmdLieDownStep1 = qCmdStandUpStep2;

        //   pComBias << 0.0, 0.0, 0.0;  //+后移  -前移

        // 8.6度  19.34度  161度
        // qInit << -0.30, 0.30, -0.27, 0.27, 0.33, 0.33, 0.33, 0.33, 2.81, 2.81, 2.81, 2.81;  // abad 绝对值越大越来越内八，Hip关节值越大，越靠后
        // qInit << -0.12, 0.12, -0.12, 0.12, 0.33, 0.33, 0.33, 0.33, 2.81, 2.81, 2.81, 2.81;  // 修改过
        // qInit << -0.14, 0.10, -0.14, 0.10, 0.33, 0.33, 0.33, 0.33, 2.81, 2.81, 2.81, 2.81;  // 修改过
        qInit << -0.10, 0.14, -0.10, 0.14, 0.33, 0.33, 0.33, 0.33, 2.81, 2.81, 2.81, 2.81;  // 修改过
        motorPara[STATE_LIE].k2.fill(0);
        motorPara[STATE_LIE].k1.fill(0);
        motorPara[STATE_LIE].k3.fill(0);

        motorPara[STATE_STAND].k2.fill(200);
        motorPara[STATE_STAND].k1.fill(10.0);
        motorPara[STATE_STAND].k3.fill(1);

        motorPara[STATE_WALK].k2.fill(30);
        motorPara[STATE_WALK].k1.fill(4);
        motorPara[STATE_WALK].k3.fill(1);

        motorPara[STATE_SLOW_DOWN].k2 = motorPara[STATE_WALK].k2;
        motorPara[STATE_SLOW_DOWN].k1 = motorPara[STATE_WALK].k1;
        motorPara[STATE_SLOW_DOWN].k3 = motorPara[STATE_WALK].k3;

        motorPara[STATE_FALL].k2.fill(0);
        motorPara[STATE_FALL].k1.fill(1.0);  // 中型的在算法里面制定了其他值
        motorPara[STATE_FALL].k3.fill(0);

        motorPara[STATE_RECOVER].k2.fill(200);
        motorPara[STATE_RECOVER].k1.fill(4.0);
        motorPara[STATE_RECOVER].k3.fill(1.0);

        motorPara[STATE_TRANSIT0_1].k2.fill(200);
        motorPara[STATE_TRANSIT0_1].k1.fill(4.0);
        motorPara[STATE_TRANSIT0_1].k3.fill(1);

        motorPara[STATE_TRANSIT1_0].k2 = motorPara[STATE_TRANSIT0_1].k2;
        motorPara[STATE_TRANSIT1_0].k1 = motorPara[STATE_TRANSIT0_1].k1;
        motorPara[STATE_TRANSIT1_0].k3 = motorPara[STATE_TRANSIT0_1].k3;

        // 这部分根据电机不同，修改
        qLimit.min << -60, -60, -60, -60, -135, -135, -180, -180, 59, 59, 59, 59;  // 关节命令限制参数
        qLimit.max << 60, 60, 60, 60, 175, 175, 130, 130, 161, 161, 161, 161;      // 可以适当放大一些，起码比qnit大

        qLimit.min *= (M_PI / 180);  // 转换成弧度，再乘以安全系数
        qLimit.max *= (M_PI / 180);

        qdLimit.min.fill(-15.7);
        qdLimit.max.fill(15.7);

        tauLimit.min.fill(-70);
        tauLimit.max.fill(70);
    }
};