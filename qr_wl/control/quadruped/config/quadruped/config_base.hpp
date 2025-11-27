
#pragma once

#include <map>

#include "baseline.hpp"
#include "innerType.hpp"

// 型号配置，仿标准库写法
class QrModelSet
{
public:
    bool contains(QrModel model) const { return modelSet_.count(model); }
    void insert(QrModel model)
    {
        if (modelSet_.count(model) == 0) {
            modelSet_.insert(model);
            modelVec_.push_back(model);
        }
    }
    void clear()
    {
        modelSet_.clear();
        modelVec_.clear();
    }
    // 获取最后一个，即机子最终型号
    QrModel back() const { return modelVec_.back(); }

private:
    std::set<QrModel> modelSet_;
    std::vector<QrModel> modelVec_;
};

// 基类配置选项，所有的配置类建议均已Config_开头
struct Config_base
{
    QrModelSet model;  // 该字段也记录了继承关系
    double DOG_M_LEG00 = 0;
    double DOG_M_LEG01 = 0;
    double DOG_M_LEG02 = 0;
    double DOG_M = 9 + 2;  // 补偿楼梯模式高度损失太多的问题

    // double BODY_Ix = 0.02974f;
    // double BODY_Iy = 0.06308f;
    // double BODY_Iz = 0.07984f;

    double BODY_Ix = 11253 * 1e-6;  // 量纲kg*m2，数据来源MIT，因为动力学建模时，用的是这个
    double BODY_Iy = 36203 * 1e-6;
    double BODY_Iz = 42673 * 1e-6;

    double BODY_Lx = 0.37;
    double BODY_Ly = 0.1f;

    double LEG_L0 = 0.07;
    double LEG_L1 = 0.2f;
    double LEG_L2 = 0.2f;

    /*unitree_go1*/
    // double LEG_L0 = 0.08;
    // double LEG_L1 = 0.213f;
    // double LEG_L2 = 0.213f;

    double kneeRatio = 1.0;  // 只有实物采用
    // double kneeRatio = 1;

    double linkAngle = 0.0;

    double dt = 0.002;

    double bodyHeight;
    // double bodyHeight = 0.283;  // 起始身体高度
    Mat3<double> bodyI;
    Mat6<double> matrixS;
    // double kW = 0.1;  // 根据斜面上走的那篇论文尅看出，这个系数越大，QP求解出来的值力越接近，反之力差异越大，0.01~1.0
    double kW = 1;
    double kU = 0.1;
    // double kW = 0.1;
    double mu = 0.4;
    double fZmax = 135;  // 15Nm * 1.55/0.1335 = 173N     18 * 1 / 0.1335 = 135N
    // kW = 0.001;  //real
    // kW = 0.1;  // sim
    // 该系数等于0.001时，速度可以达到约2m/s，论文high-slope中的alpha = 0.01时，alpha调大，足部力差异越小，调小力差异越大（不稳）。
    // 20220426系数从0.001调整为0.1，侧向行走不稳定突变消失
    // mu = 0.5;
    // fZmax = 250;  // 15Nm * 1.55/0.1335 = 173N
    double standUpTime = 1.0;
    double lieDownTime = 1.0;
    Mat34<double> qInit, qInitUnitree;
    MotorCtrlPara motorPara[STATE_TOTALNUM];
    Vec3<double> pComBias;
    double pFootzMin = -0.15;
    // 框架所用的参数
    AlgoModel algoModel{AlgoModel::jp};  // 具体的算法模型
    ParamRange paramRange;
    ValueRange<double> ikLegRange{0.13, 0.39};
    Mat34<double> qCmdStandUpStep2;  // 站起来第二个阶段关机角命令
    Mat34<double> qCmdLieDownStep1;
    // double tStanceRef = 0.19;
    // double tSwingRef = 0.19;
    // double stepzRef = 0.2;

    ValueRange<Mat34<double>> qLimit;
    ValueRange<Mat34<double>> qdLimit;
    ValueRange<Mat34<double>> tauLimit;

    std::map<WalkMode, double> tStance;
    std::map<WalkMode, double> tSwing;
    std::map<WalkMode, double> stepZ;
    PidStrategyType pidStrategy{PidStrategyType::statical};
    AdaptivePidPara adaptivePid;
    TorqueModifyStrategyType torqueModifyStrategy{TorqueModifyStrategyType::base};
    Config_base()
    {
        model.insert(QrModel::base);
        tSwing[WalkMode::normal] = 0.16;
        tStance[WalkMode::normal] = 0.16;
        stepZ[WalkMode::normal] = 0.1;

        tSwing[WalkMode::climb] = 0.22;
        tStance[WalkMode::climb] = 0.30;
        stepZ[WalkMode::climb] = 0.18;

        tSwing[WalkMode::light] = 0.22;
        tStance[WalkMode::light] = 0.34;
        stepZ[WalkMode::light] = 0.1;

        // qInit << -0.6760, 0.6760, -0.6760, 0.6760, 0.6109, 0.6109, 0.6109, 0.61090, 2.6180, 2.6180, 2.6180, 2.6180;
        /*unitree_go1*/
        qInit << -0.358, 0.358, -0.358, 0.358, 0.33, 0.33, 0.33, 0.33, 2.81, 2.81, 2.81, 2.81;  // 仿真时只影响起立趴下其实姿态,实物时，影响全部关节角
        pComBias << -0.0, 0.0, 0.0;                                                             // 相对于几何中心的偏移，参考high-slope的论文,注意，之前做法可能不对
        bodyI << BODY_Ix, 0.0, 0.0, 0.0, BODY_Iy, 0.0, 0.0, 0.0, BODY_Iz;
        Vec6<double> s;
        // s << 20, 20, 50, 450, 450, 450;  // 宇树go1
        //  s << 1, 1, 1, 1, 1, 1;
        s << 5, 5, 10, 10, 10, 10;
        matrixS = s.asDiagonal();
        qCmdStandUpStep2 << 0, 0, 0, 0, -60, -60, -60, -60, 150, 150, 150, 150;
        qCmdLieDownStep1 = qCmdStandUpStep2;

        // bodyHeight = sqrt(pow(LEG_L1, 2) + pow(LEG_L2, 2));
        bodyHeight = 0.30;
        paramRange.height.max = 0.35;
        paramRange.height.min = 0.25;

        paramRange.loadMass.max = 3.0;
        paramRange.loadMass.min = 0.0;

        motorPara[Enum2Num(AlgoState::lie)].k2.fill(0);
        motorPara[Enum2Num(AlgoState::lie)].k1.fill(0);
        motorPara[Enum2Num(AlgoState::lie)].k3.fill(0);

        motorPara[STATE_STAND].k2 << 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0;
        motorPara[STATE_STAND].k1 << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0;
        motorPara[STATE_STAND].k3 << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

        // motorPara[STATE_WALK].k2 << 20.0, 20.0, 20.0, 20.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 30.0;
        // motorPara[STATE_WALK].k1 << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        // motorPara[STATE_WALK].k3 << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        motorPara[STATE_WALK].k2 << 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0;
        motorPara[STATE_WALK].k1.fill(3.5);
        motorPara[STATE_WALK].k3.fill(1.0);

        motorPara[STATE_SLOW_DOWN].k2 = motorPara[STATE_WALK].k2;
        motorPara[STATE_SLOW_DOWN].k1 = motorPara[STATE_WALK].k1;
        motorPara[STATE_SLOW_DOWN].k3 = motorPara[STATE_WALK].k3;

        motorPara[STATE_FALL].k2.fill(0);
        motorPara[STATE_FALL].k1.fill(3.0);
        motorPara[STATE_FALL].k3.fill(0);

        motorPara[STATE_RECOVER].k2.fill(60);
        motorPara[STATE_RECOVER].k1.fill(3);
        motorPara[STATE_RECOVER].k3.fill(1.0);

        motorPara[STATE_BACKFLIP].k2 << 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0;
        motorPara[STATE_BACKFLIP].k1 << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        motorPara[STATE_BACKFLIP].k3 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        motorPara[STATE_TRANSIT0_1].k2 << 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0;
        motorPara[STATE_TRANSIT0_1].k1 << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0;
        motorPara[STATE_TRANSIT0_1].k3 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        motorPara[STATE_TRANSIT1_0].k2 << 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0;
        motorPara[STATE_TRANSIT1_0].k1 << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0;
        motorPara[STATE_TRANSIT1_0].k3 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        motorPara[STATE_HANDSTAND].k2 = motorPara[STATE_STAND].k2;
        motorPara[STATE_HANDSTAND].k1 = motorPara[STATE_STAND].k1;
        motorPara[STATE_HANDSTAND].k3 = motorPara[STATE_STAND].k3;

        motorPara[STATE_UPRIGHT].k2 = motorPara[STATE_STAND].k2;
        motorPara[STATE_UPRIGHT].k1 = motorPara[STATE_STAND].k1;
        motorPara[STATE_UPRIGHT].k3 = motorPara[STATE_STAND].k3;

        motorPara[STATE_WALK2HANDSTAND].k2 = motorPara[STATE_STAND].k2;
        motorPara[STATE_WALK2HANDSTAND].k1 = motorPara[STATE_STAND].k1;
        motorPara[STATE_WALK2HANDSTAND].k3 = motorPara[STATE_STAND].k3;

        motorPara[STATE_WALK2UPRIGHT].k2 = motorPara[STATE_STAND].k2;
        motorPara[STATE_WALK2UPRIGHT].k1 = motorPara[STATE_STAND].k1;
        motorPara[STATE_WALK2UPRIGHT].k3 = motorPara[STATE_STAND].k3;

        motorPara[STATE_HANDSTAND2WALK].k2 = motorPara[STATE_STAND].k2;
        motorPara[STATE_HANDSTAND2WALK].k1 = motorPara[STATE_STAND].k1;
        motorPara[STATE_HANDSTAND2WALK].k3 = motorPara[STATE_STAND].k3;

        motorPara[STATE_UPRIGHT2WALK].k2 = motorPara[STATE_STAND].k2;
        motorPara[STATE_UPRIGHT2WALK].k1 = motorPara[STATE_STAND].k1;
        motorPara[STATE_UPRIGHT2WALK].k3 = motorPara[STATE_STAND].k3;

        // 20240621开始使用linkV2-3参数作为通用参数
        qLimit.min << -60, -60, -60, -60, -87, -87, -87, -87, 50, 50, 50, 50;  // 关节命令限制参数
        qLimit.max << 60, 60, 60, 60, 138, 138, 138, 138, 156, 156, 156, 156;  // 可以适当放大一些，起码比qnit大

        qLimit.min *= (M_PI / 180);  // 转换成弧度，再乘以安全系数
        qLimit.max *= (M_PI / 180);

        qdLimit.min.fill(-45);
        qdLimit.max.fill(45);

        tauLimit.min.fill(-36);
        tauLimit.max.fill(36);
    }

    template <typename T>
    void Load(const T& in)
    {
        *this = static_cast<Config_base>(in);
    }
};
