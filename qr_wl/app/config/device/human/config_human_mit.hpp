
#pragma once

#include "config_human.hpp"

struct CfgDev_human_mit : public CfgDev_human
{
    CfgDev_human_mit()
    {
        for (int i = 0; i < 2; i++) {
            armMotorCfg[i].resize(6);
            legMotorCfg[i].resize(5);
        }
    }
};