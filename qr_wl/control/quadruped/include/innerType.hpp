
#pragma once

#include "baseline.hpp"
#include "pub/contrType.hpp"

// 支持的算法模型，目标只支持一种，且为默认值
enum class AlgoModel
{
    jp,
    oneleg,
    other,
};

// 电机PID参数
class MotorCtrlPara
{
public:
    Mat34<double> k2;
    Mat34<double> k1;
    Mat34<double> k3;
    MotorCtrlPara()
    {
        k2.setZero();
        k1.setZero();
        k3.setZero();
    }
};
using MotorPidTPara = MotorCtrlPara;

struct AdaptivePidPara
{
    ValueRange<Mat34<double>> k2;
    ValueRange<Mat34<double>> k1;
    ValueRange<Mat34<double>> k3;
    AdaptivePidPara()
    {
        k2.min.setZero();
        k1.min.setZero();
        k3.min.setZero();

        k2.max.setZero();
        k1.max.setZero();
        k3.max.setZero();
    }
};

// 以下用于C枚举
enum
{
    STATE_LIE = Enum2Num(AlgoState::lie),
    STATE_STAND = Enum2Num(AlgoState::stand),
    STATE_WALK = Enum2Num(AlgoState::walk),
    STATE_TRANSIT0_1 = Enum2Num(AlgoState::stand2),
    STATE_TRANSIT1_0 = Enum2Num(AlgoState::lie2),
    STATE_SLOW_DOWN = Enum2Num(AlgoState::slowDown),
    STATE_FALL = Enum2Num(AlgoState::fall),
    STATE_RECOVER = Enum2Num(AlgoState::recover),
    STATE_BACKFLIP = Enum2Num(AlgoState::backFlip),
    STATE_HANDSTAND = Enum2Num(AlgoState::handStand),
    STATE_UPRIGHT = Enum2Num(AlgoState::upright),
    STATE_WALK2HANDSTAND = Enum2Num(AlgoState::walk3),
    STATE_WALK2UPRIGHT = Enum2Num(AlgoState::walk5),
    STATE_HANDSTAND2WALK = Enum2Num(AlgoState::walk4),
    STATE_UPRIGHT2WALK = Enum2Num(AlgoState::upright2walk),
    STATE_TOTALNUM = Enum2Num(AlgoState::total)
};

// 所有的pid策略
enum class PidStrategyType
{
    statical,  // 静态策略，使用兼容的旧策略
    base,      // 自适应pid之基本算法
    middle,    // 自适应pid之middle算法
    swingLowKpKd,
};

enum class TorqueModifyStrategyType
{
    base,
    middle,
};

// 控制算法行为的命令
struct ControllerCmd
{
    Vec3<double> linearV;
    Vec3<double> angularV;
    Vec3<double> pose;
    bool holdFlag;
    double height;
    WalkMode mode;
    GaitType gait;
    AlgoState nextState;
    double loadMass = 0.0;
    ControllerCmd();
};

// 机器人型号
enum class QrModel
{
    unused,  // 不启用
    base,
    gazebo,
    linkV2_3,
    middle,
    middleV3,
    middleV4,
    gazebo_w,
    linkV2_3_w,
};

template <typename T>
struct motorValueTemplate
{
    T alpha;
    T torq;
    T blta;

    motorValueTemplate()  // 初始化
    {
        setZero();
    }

    void setZero()
    {
        alpha.setZero();
        torq.setZero();
        blta.setZero();
    }

    motorValueTemplate& operator+=(const motorValueTemplate& data)
    {
        alpha += data.alpha;
        torq += data.torq;
        blta += data.blta;
        return *this;
    }

    motorValueTemplate& operator-=(const motorValueTemplate& data)
    {
        alpha -= data.alpha;
        torq -= data.torq;
        blta -= data.blta;
        return *this;
    }

    motorValueTemplate& operator*=(int size)
    {
        alpha *= size;
        torq *= size;
        blta *= size;
        return *this;
    }
};

template <typename T>
struct motorValuePidTemplate : public motorValueTemplate<T>
{
    T k2;
    T k1;
    T k3;
    motorValuePidTemplate()  // 初始化
    {
        setZero();
    }

    // motorValueTemplate到motorValuePidTemplate的拷贝构造函数
    motorValuePidTemplate(const motorValueTemplate<T>& data)
    {
        motorValueTemplate<T>::operator=(data);
        k2.setZero();
        k1.setZero();
        k3.setZero();
    }

    void setZero()
    {
        motorValueTemplate<T>::setZero();
        k2.setZero();
        k1.setZero();
        k3.setZero();
    }

    // motorValue 赋值到motorValuePid的函数，部分参数使用默认值
    motorValuePidTemplate& operator=(const motorValueTemplate<T>& data)
    {
        motorValueTemplate<T>::operator=(data);
        return *this;
    }

    motorValuePidTemplate& operator+=(const motorValuePidTemplate<T>& data)
    {
        motorValueTemplate<T>::operator+=(data);
        k2 += data.k2;
        k1 += data.k1;
        k3 += data.k3;
        return *this;
    }

    motorValuePidTemplate& operator-=(const motorValuePidTemplate<T>& data)
    {
        motorValueTemplate<T>::operator-=(data);
        k2 -= data.k2;
        k1 -= data.k1;
        k3 -= data.k3;
        return *this;
    }

    motorValuePidTemplate& operator*=(int size)
    {
        motorValueTemplate<T>::operator*=(size);
        k2 *= size;
        k1 *= size;
        k3 *= size;
        return *this;
    }
};

using DataCmd = motorValueTemplate<Mat34<double>>;
using CmdVal2 = motorValuePidTemplate<Mat34<double>>;

using CmdVal4 = motorValueTemplate<Mat44<double>>;
using DataValue2 = motorValuePidTemplate<Mat44<double>>;