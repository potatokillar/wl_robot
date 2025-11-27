
#pragma once

#include "../config_base.hpp"

struct CfgDev_arm : public CfgDev_base
{
    std::vector<MotorCfg> motorCfg;
    std::string ethercat;
    std::vector<std::string> uartGripper;
    CfgDev_arm()
    {
        uartGripper.clear();
        uartGripper.push_back("/dev/ttyAMA2");
    }

    template <typename T>
    void Load(const T& in)
    {
        *this = static_cast<CfgDev_arm>(in);
    }
};
