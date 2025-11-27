
#pragma once

#include <map>
#include <string>
#include <vector>

#include "arm/config_base.hpp"
#include "baseline.hpp"
#include "innerType.hpp"

// 均是单例，即时单独声明，也可其效果
class ArmParam : public CfgArm_base
{
public:
    static ArmParam& GetInstance()
    {
        static ArmParam instance;
        return instance;
    }

private:
    ArmParam();
    void ReInit();
    void GetConfig();
};

const inline ArmParam& GetArmParam() { return ArmParam::GetInstance(); }