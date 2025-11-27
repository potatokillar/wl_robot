
#pragma once

#include "config_arm.hpp"

struct CfgDev_arm_iris : public CfgDev_arm
{
    CfgDev_arm_iris()
    {
        ethercat = "eth0";
        for (int i = 0; i < 6; i++) {
            motorCfg.push_back({MotorModel::zeroErr, true, 100, 19});
        }
    }
};