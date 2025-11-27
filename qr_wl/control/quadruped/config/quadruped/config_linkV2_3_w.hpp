
#pragma once

#include "config_base.hpp"
#include "config_linkV2_3.hpp"

struct Config_linkV2_3_w : public Config_linkV2_3
{
    Config_linkV2_3_w()
    {
        model.insert(QrModel::linkV2_3_w);
        qInit << -0.5, 0.5, -0.5, 0.5, 0.71, 0.71, 0.71, 0.71, 2.563, 2.563, 2.563, 2.563;
        qInitUnitree << -0.5, 0.5, -0.5, 0.5, 0.86, 0.86, 0.86, 0.86, -2.563, -2.563, -2.563, -2.563;
    }
};