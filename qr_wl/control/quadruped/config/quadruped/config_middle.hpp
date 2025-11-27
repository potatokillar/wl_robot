
#pragma once

#include "config_base.hpp"

struct Config_middle : public Config_base
{
    Config_middle()
    {
        model.insert(QrModel::middle);
        pidStrategy = PidStrategyType::middle;
        torqueModifyStrategy = TorqueModifyStrategyType::middle;
        tStance[WalkMode::normal] = 0.34;
        tSwing[WalkMode::normal] = 0.24;
        stepZ[WalkMode::normal] = 0.1;

        tStance[WalkMode::climb] = 0.48;  // 0.6
        tSwing[WalkMode::climb] = 0.26;   // 0.3
        stepZ[WalkMode::climb] = 0.5;     // 0.2

        standUpTime = 1.8;
        lieDownTime = 1.8;

        DOG_M = 25;  // 由于攀爬模式，抬腿高度变高了，高度有损失

        DOG_M_LEG00 = 1.1;  // 给拉格朗日方程用的
        DOG_M_LEG01 = 1.6;  // 给拉格朗日方程用的
        DOG_M_LEG02 = 0.8;  // 给拉格朗日方程用的

        BODY_Ix = 101509190.7 * 10e-9;
        BODY_Iy = 431323650.64 * 10e-9;
        BODY_Iz = 497082269.97 * 10e-9;

        bodyI << BODY_Ix, 0.0, 0.0, 0.0, BODY_Iy, 0.0, 0.0, 0.0, BODY_Iz;

        BODY_Lx = 0.59;
        BODY_Ly = 0.16;

        LEG_L0 = 0.0947;
        LEG_L1 = 0.3;
        LEG_L2 = 0.3 + 0.02;

        kneeRatio = 1;
        bodyHeight = 0.424;
        paramRange.height.max = bodyHeight * 1.25;
        paramRange.height.min = bodyHeight * 0.9;
        fZmax = 500;
        kW = 0.1;
        pFootzMin = -0.17;
        ikLegRange.min = 0.17;
        ikLegRange.max = 0.59;
        qCmdStandUpStep2 << 0, 0, 0, 0, -80, -80, -80, -80, 140, 140, 140, 140;
        qCmdLieDownStep1 = qCmdStandUpStep2;

        pComBias << 0.13, 0.0, 0.0;  //+后移  -前移

        qInit << -0.5585, 0.5585, -0.5585, 0.5585, 0.54, 0.54, 0.54, 0.54, 2.6354, 2.6354, 2.6354, 2.6354;

        motorPara[STATE_LIE].k2 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        motorPara[STATE_LIE].k1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        motorPara[STATE_LIE].k3 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        motorPara[STATE_STAND].k2.fill(160);
        motorPara[STATE_STAND].k1.fill(15);
        motorPara[STATE_STAND].k3.fill(1);

        motorPara[STATE_WALK].k2.fill(80);
        motorPara[STATE_WALK].k1.fill(7);
        motorPara[STATE_WALK].k3.fill(1);

        motorPara[STATE_SLOW_DOWN].k2 = motorPara[STATE_WALK].k2;
        motorPara[STATE_SLOW_DOWN].k1 = motorPara[STATE_WALK].k1;
        motorPara[STATE_SLOW_DOWN].k3 = motorPara[STATE_WALK].k3;

        motorPara[STATE_FALL].k2.fill(10);
        motorPara[STATE_FALL].k1.fill(1);
        motorPara[STATE_FALL].k3 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        motorPara[STATE_RECOVER].k2.fill(80);
        motorPara[STATE_RECOVER].k1.fill(7);
        motorPara[STATE_RECOVER].k3.fill(0);

        motorPara[STATE_TRANSIT0_1].k2.fill(160);
        motorPara[STATE_TRANSIT0_1].k1.fill(15);
        motorPara[STATE_TRANSIT0_1].k3.fill(1);

        motorPara[STATE_TRANSIT1_0].k2 = motorPara[STATE_TRANSIT0_1].k2;
        motorPara[STATE_TRANSIT1_0].k1 = motorPara[STATE_TRANSIT0_1].k1;
        motorPara[STATE_TRANSIT1_0].k3 = motorPara[STATE_TRANSIT0_1].k3;

        adaptivePid.k2.min.fill(80);
        adaptivePid.k1.min.fill(7);
        adaptivePid.k3.min.fill(1);

        adaptivePid.k2.max.fill(80);
        adaptivePid.k1.max.fill(7);
        adaptivePid.k3.max.fill(1);

        qLimit.min.fill(-12.5);
        qLimit.max.fill(12.5);
        qdLimit.min.fill(-8);
        qdLimit.max.fill(8);
        tauLimit.min.fill(-144);
        tauLimit.max.fill(144);

#if 0
        motorPara[STATE_STAND].k2.fill(50);
        motorPara[STATE_STAND].k1.fill(1);
        motorPara[STATE_STAND].k3.fill(1);

        motorPara[STATE_WALK].k2.fill(20);
        motorPara[STATE_WALK].k1.fill(1);
        motorPara[STATE_WALK].k3.fill(1);

        motorPara[STATE_SLOW_DOWN].k2 = motorPara[STATE_WALK].k2;
        motorPara[STATE_SLOW_DOWN].k1 = motorPara[STATE_WALK].k1;
        motorPara[STATE_SLOW_DOWN].k3 = motorPara[STATE_WALK].k3;

        motorPara[STATE_FALL].k2.fill(10);
        motorPara[STATE_FALL].k1.fill(1);
        motorPara[STATE_FALL].k3 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        motorPara[STATE_RECOVER].k2.fill(50);
        motorPara[STATE_RECOVER].k1.fill(1);
        motorPara[STATE_RECOVER].k3.fill(0);

        motorPara[STATE_TRANSIT0_1].k2.fill(50);
        motorPara[STATE_TRANSIT0_1].k1.fill(1);
        motorPara[STATE_TRANSIT0_1].k3.fill(1);

        motorPara[STATE_TRANSIT1_0].k2 = motorPara[STATE_TRANSIT0_1].k2;
        motorPara[STATE_TRANSIT1_0].k1 = motorPara[STATE_TRANSIT0_1].k1;
        motorPara[STATE_TRANSIT1_0].k3 = motorPara[STATE_TRANSIT0_1].k3;

        adaptivePid.k2.min.fill(20);
        adaptivePid.k1.min.fill(1);
        adaptivePid.k3.min.fill(1);

        adaptivePid.k2.max.fill(20);
        adaptivePid.k1.max.fill(1);
        adaptivePid.k3.max.fill(1);

#endif
    }
};