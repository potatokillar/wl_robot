
#pragma once

#include "config_arm.hpp"

struct CfgDev_arm_ga701 : public CfgDev_arm
{
    CfgDev_arm_ga701()
    {
        ethercat = "enp2s0";
        // ethercat = "enp5s0";
        for (int i = 0; i < 7; i++) {
            motorCfg.push_back({MotorModel::zeroErr, true, 100, 19});
        }
    }
};