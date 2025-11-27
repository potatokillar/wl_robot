
#pragma once

#include <map>
#include <string>
#include <vector>

#include "baseline.hpp"
#include "device/arm/config_arm.hpp"
#include "deviceType.hpp"

class DevArmParam : public CfgDev_arm
{
public:
    static DevArmParam& GetInstance()
    {
        static DevArmParam instance;
        return instance;
    }
    DevModel::arm model;

private:
    DevArmParam();
    void ReInit();
    void GetConfig();
};

const inline DevArmParam& GetDevArmParam() { return DevArmParam::GetInstance(); }