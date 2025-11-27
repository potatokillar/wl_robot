
#pragma once

#include "config_base.hpp"

struct CfgArm_ga701 : public CfgArm_base
{
    CfgArm_ga701()
    {
        model = ArmModel::ga701;
        mode = NetMode::ethercat;
        ArmAixsNum = 7;

        base.base = {0, 0, 0, 0, 0, 0};
        tool.tool = {0, 0, 0, 0, 0, 0};

        coor.tool["tool0"] = {0, 0, 0, 0, 0, 0};
        coor.tool["tool1"] = {0, 0, 0, 0, 0, 0};

        kine_param.d = {0.263, 0.0, 0.357, 0.0, 0.343, 0.0, 0.197};
        kine_param.a = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        kine_param.alpha = {-0.5 * M_PI, 0.5 * M_PI, -0.5 * M_PI, 0.5 * M_PI, -0.5 * M_PI, 0.5 * M_PI, 0};
        kine_param.offset = {0, 0, 0, 0, 0, 0, 0};

        // axis.S_max_7 = {3.01, 2.25, 3.01, 2.35, 2.96, 2.23, 2.96;
        // axis.S_min_7 = {-3.01, -2.25, -3.01, -2.35, -2.96, -2.23, -2.96;
        axis.S_max_user = {2.96, 2.25, 2.96, 2.15, 2.95, 2.0, 2.95};
        axis.S_min_user = {-2.96, -2.25, -2.96, -2.15, -2.95, -2.0, -2.95};  // for User
        axis.S_max_real = {2.96, 2.25, 2.96, 2.15, 2.95, 2.0, 2.95};
        axis.S_min_real = {-2.96, -2.25, -2.96, -2.15, -2.95, -2.0, -2.95};  // for User
        axis.S_max_reset = {3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14};
        axis.S_min_reset = {-3.14, -3.14, -3.14, -3.14, -3.14, -3.14, -3.14};

        axis.V_max = {3.14, 3.14, 3.14, 3.14, 3.14, 3.14, 3.14};
        axis.A_max = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
        axis.gear_flip = {1, -1, 1, -1, 1, -1, 1};

        cart.V_cart << 1, 1, 1, 1, 1, 1;
        cart.A_cart << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        cart.J_cart << 0.25, 0.25, 0.25, 0.25, 0.25, 0.25;
    }
};