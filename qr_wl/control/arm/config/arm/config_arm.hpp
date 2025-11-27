
#pragma once

#include "config_base.hpp"

struct CfgArm_arm : public CfgArm_base
{
    CfgArm_arm()
    {
        model = ArmModel::arm;
        mode = NetMode::can;
        ArmAixsNum = 6;

        base.base = {0, 0, 0.143, 0, 0, 0};
        tool.tool = {0, 0, 0.1225, 0, 0, 0};
        kine_param.a = {0, 0, 0.345, 0.0866, 0, 0};
        kine_param.d = {0.0, 0, 0, 0.4445, 0, 0.0};
        kine_param.alpha = {0, -0.5 * M_PI, 0, -0.5 * M_PI, 0.5 * M_PI, -0.5 * M_PI};
        kine_param.offset = {0, -M_PI, 0.5 * M_PI, 0, 0.5 * M_PI, 0};

        coor.tool["tool0"] = {0, 0, 0, 0, 0, 0};
        // coor.tool["tool1"] << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

        axis.S_max = {3.1415, 3.6, 0, 2.7, 0.3, 3.14};
        axis.S_min = {-2.6, 0, -3.0, -2.7, -3.7, -3.14};  // My&algo
        axis.S_min_real = {-3.142, -3.6, -3.0, -2.7, -1.1, -3.14};
        axis.S_max_real = {2.6, 0, 0, 2.7, 2.9, 3.14};
        axis.V_max = {1, 1, 1, 1, 1, 1};
        axis.A_max = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        axis.J_max = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25};

        cart.V_cart << 1, 1, 1, 1, 1, 1;
        cart.A_cart << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        cart.J_cart << 0.25, 0.25, 0.25, 0.25, 0.25, 0.25;
    }
};