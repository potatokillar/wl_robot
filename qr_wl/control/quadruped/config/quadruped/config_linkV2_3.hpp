
#pragma once

#include "config_base.hpp"

struct Config_linkV2_3 : public Config_base
{
    Config_linkV2_3()
    {
        model.insert(QrModel::linkV2_3);
        BODY_Lx = 0.385;
        LEG_L0 = 0.0955;

        DOG_M = 13.5;

        motorPara[STATE_WALK].k2.fill(10);
        motorPara[STATE_WALK].k1.fill(1.5);  // 经过对比J6和J9，平衡站立时无明显软硬区别，采用同一套参数
        motorPara[STATE_WALK].k3.fill(1);

        tSwing[WalkMode::climb] = 0.2;
        tStance[WalkMode::climb] = 0.24;
        stepZ[WalkMode::climb] = 0.16;

        tSwing[WalkMode::normal] = 0.17;  // 实物实时性更好
        tStance[WalkMode::normal] = 0.17;

        pComBias << 0.0, 0.0, 0.0;
        qInit << -0.30, 0.30, -0.30, 0.30, 0.51, 0.51, 0.54, 0.54, 2.65, 2.65, 2.65, 2.65;
        kneeRatio = 1.0;  // 这是传动减速比，不是电机减速比
    }
};