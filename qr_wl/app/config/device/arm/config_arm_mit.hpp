
#pragma once

#include "config_arm.hpp"

struct CfgDev_arm_mit : public CfgDev_arm
{
    CfgDev_arm_mit()
    {
        for (int i = 0; i < 6; i++) {
            motorCfg.push_back({MotorModel::haitai, false, 9});
        }

        motorCfg[1].model = MotorModel::tMotor_80_64;
        motorCfg[1].chiral = true;
        motorCfg[1].ratio = 64;
        motorCfg[2].chiral = true;

        ethercat.clear();
    }
};