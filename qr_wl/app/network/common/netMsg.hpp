
#pragma once
#include "baseline.hpp"

namespace msg::net
{

using motor_cmd = msg::qr::motor_cmd;
using motor_data = msg::qr::motor_ret;
using imu_data = msg::imu_data;
using watch_data = msg::watch_data;
using arm_cmd = msg::arm_cmd;
using arm_data = msg::arm_data;
// using arm_teach = msg::arm_teach;
struct leg_debug
{
    motor_data cmd;
    motor_data ret;
};

struct rpy_debug
{
    imu_data cmd;
    imu_data ret;
};

struct arm_teachs
{
    msg::arm_teach cmd;
    msg::arm_teach ret;
};

};  // namespace msg::net
