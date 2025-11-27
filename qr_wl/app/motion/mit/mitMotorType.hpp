
#pragma once

#include "deviceType.hpp"

enum class MotorState
{
    ok,
    disable,
    enable,
    error,
    timeout,
};

enum class MotorCtrl
{
    enable,
    zero,
    disable,
};

struct MitMotorCfg
{
    MotorModel model{MotorModel::haitai};
    bool chiral{false};
    double ratio{6};

    double zeroPos{0};
};