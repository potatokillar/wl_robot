
#pragma once

#include "../config_base.hpp"

struct CfgDev_human : public CfgDev_base
{
    std::array<std::vector<MotorCfg>, 2> legMotorCfg;  // 0 左，1 右
    std::array<std::vector<MotorCfg>, 2> armMotorCfg;
    CfgDev_human() {}

    /**
     * @description: 重载Load
     * @return {}
     */
    template <typename T>
    void Load(const T& in)
    {
        *this = static_cast<CfgDev_human>(in);
    }
};