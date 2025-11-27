
#pragma once

#include "config_gazebo.hpp"

struct Config_gazebo_w : public Config_gazebo
{
    Config_gazebo_w() { model.insert(QrModel::gazebo_w); }
};