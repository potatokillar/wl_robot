
#pragma once

#include "config_base.hpp"

struct CfgArm_iris : public CfgArm_base
{
    CfgArm_iris()
    {
        model = ArmModel::iris;
        mode = NetMode::ethercat;
        ArmAixsNum = 6;

        base.base = {0, 0, 0.219, 0, 0, 0};
        tool.tool = {0, 0, 0.243, 0, 0, 0};
        kine_param.a = {0.0, 0.0, 0.26, 0.0, 0.0, 0.0};
        kine_param.d = {0.0, 0.0, 0.0, 0.26, 0.0, 0.0};

        kine_param.alpha = {0, -0.5 * M_PI, 0, -0.5 * M_PI, 0.5 * M_PI, -0.5 * M_PI};
        kine_param.offset = {0, -0.5 * M_PI, -0.5 * M_PI, 0, 0, 0};

        coor.tool["tool0"] = {0, 0, 0, 0, 0, 0};
        coor.tool["tool1"] = {0, 0, 0, 0, 0, 0};

        axis.S_max_user = {2.95, 2.25, 2.43, 2.86, 1.6, 2.8};
        axis.S_min_user = {-2.95, -2.25, -2.43, -2.86, -1.6, -2.8};  // for User
        axis.S_max_real = {2.96, 2.26, 2.44, 2.87, 1.7, 2.9};
        axis.S_min_real = {-2.96, -2.26, -2.44, -2.87, -1.7, -2.9};  // My&algo
        axis.S_max_reset = {3.14, 2.5, 3.0, 3.14, 2.5, 3.14};
        axis.S_min_reset = {-3.14, -2.5, -3.0, -3.14, -2.5, -3.14};
        // axis.V_max = {3.2, 3.2, 3.2, 3.2, 3.2, 3.2;
        // axis.A_max = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        axis.V_max = {6.0, 6.0, 6.0, 6.0, 6.0, 6.0};
        axis.A_max = {6.28, 6.28, 6.28, 6.28, 6.28, 6.38};
        axis.J_max = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
        axis.gear_flip = {1, -1, 1, 1, -1, 1};

        cart.V_cart << 1, 1, 1, 1, 1, 1;
        cart.A_cart << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
        cart.J_cart << 0.25, 0.25, 0.25, 0.25, 0.25, 0.25;
    }
};