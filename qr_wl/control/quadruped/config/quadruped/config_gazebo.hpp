
#pragma once

#include "config_base.hpp"

struct Config_gazebo : public Config_base
{
    Config_gazebo()
    {
        model.insert(QrModel::gazebo);
        /*unitree_go1*/
        LEG_L0 = 0.08;
        LEG_L1 = 0.213;
        LEG_L2 = 0.213;
        // bodyHeight = sqrt(pow(LEG_L1, 2) + pow(LEG_L2, 2));  // 始终是直角
        // paramRange.height.max = bodyHeight * 1.25;
        // paramRange.height.min = bodyHeight * 0.9;

        BODY_Ix = 0.02;  // 量纲kg*m2，数据来源Go1,gazebo，建模时，用的是这个
        BODY_Iy = 0.07;
        BODY_Iz = 0.08;

        BODY_Lx = 0.38;  // from real Go1 robot
        BODY_Ly = 0.093;

        tSwing[WalkMode::normal] = 0.17;  // 实物实时性更好
        tStance[WalkMode::normal] = 0.17;

        tSwing[WalkMode::climb] = 0.2;
        tStance[WalkMode::climb] = 0.3;

        DOG_M = 13.5;  // from go1 model

        qInit << -0.3582, 0.3582, -0.3582, 0.3582, 0.3279, 0.3279, 0.3279, 0.3279, 2.818, 2.818, 2.818, 2.818;  // Hip关节值越大，越靠后
        // pComBias << 0.0, 0.0, 0.0;
    }
};