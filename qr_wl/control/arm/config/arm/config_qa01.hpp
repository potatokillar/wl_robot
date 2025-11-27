
#pragma once

#include "config_base.hpp"

struct CfgArm_qa01 : public CfgArm_base
{
    CfgArm_qa01()
    {
        model = ArmModel::qa01;
        mode = NetMode::ethercat;
        ArmAixsNum = 6;

        base.base = {0, 0, 0.1645, 0, 0, 0};
        tool.tool = {0, 0, 0.1755, 0, 0, 0};
        kine_param.a = {0, 0, 0.41, 0.085, 0, 0};
        kine_param.d = {0.0, 0, 0, 0.391, 0, 0.0};
        kine_param.alpha = {0, -0.5 * M_PI, 0, -0.5 * M_PI, 0.5 * M_PI, -0.5 * M_PI};
        kine_param.offset = {0, -0.5 * M_PI, 0, 0, 0, 0};

        coor.tool["tool0"] = {0, 0, 0.2, 0, 0, 0};
        // coor.tool["tool1"] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

        axis.S_max_user = {3.0, 1.5, 1.5, 3.0, 1.55, 3.0};
        axis.S_min_user = {-2.2, -1.5, -1.5, -2.2, -1.55, -2.2};  // for User
        axis.S_min_real = {-2.3, -1.57, -1.57, -2.3, -1.57, -2.3};
        axis.S_max_real = {3.14, 1.57, 1.57, 3.14, 1.57, 3.14};

        axis.V_max = {3.14, 3.14, 3.14, 3.14, 3.14, 3.14};
        axis.A_max = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
        axis.J_max = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25};

        axis.gear_flip = {1, -1, 1, 1, 1, 1};

        dyna_param.M = {0.26, 2.97, 1.36, 1.46, 1.25, 0.3};

        cart.V_cart << 1, 1, 1, 1, 1, 1;
        cart.A_cart << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        cart.J_cart << 0.25, 0.25, 0.25, 0.25, 0.25, 0.25;
    }
};