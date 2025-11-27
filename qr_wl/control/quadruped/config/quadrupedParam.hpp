
#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "baseline.hpp"
#include "quadruped/config_base.hpp"

static const Vec3<double> G(0.0, 0.0, 9.8);

// 均是单例，即时单独声明，也可其效果
class QuadrupedParam : public Config_base
{
public:
    static QuadrupedParam& GetInstance()
    {
        static QuadrupedParam instance;
        return instance;
    }

    Mat34<double> pRoll;  // roll关节相对于几何中心位置
    Mat34<double> pHip;   // hip关节相对于几何中心位置
    Mat34<double> pFoot;  // foot相对于几何中心位置
    Vec3<double> pCom;    // 几何中心位置
    double loadMass{0};

private:
    QuadrupedParam();
    void InitAfter();
    void ReInit();
    void GetConfig();
};

const inline QuadrupedParam& GetQrParam() { return QuadrupedParam::GetInstance(); }