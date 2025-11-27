
#pragma once

#include <map>
#include <string>
#include <vector>

#include "baseline.hpp"
#include "device/human/config_human.hpp"
#include "deviceType.hpp"

class DevHumanParam : public CfgDev_human
{
public:
    static DevHumanParam& GetInstance()
    {
        static DevHumanParam instance;
        return instance;
    }
    DevModel::human model;

private:
    DevHumanParam();
    void ReInit();
    void GetConfig();
};

const inline DevHumanParam& GetDevHumanParam() { return DevHumanParam::GetInstance(); }