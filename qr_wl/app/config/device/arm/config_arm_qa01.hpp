
#pragma once

#include "config_arm.hpp"

struct CfgDev_arm_qa01 : public CfgDev_arm
{
    CfgDev_arm_qa01()
    {
        ethercat = "eth1";
        for (int i = 0; i < 6; i++) {
            motorCfg.push_back({MotorModel::zeroErr, true, 100, 19});
        }
    }
};