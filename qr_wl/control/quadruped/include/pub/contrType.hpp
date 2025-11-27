
#pragma once

#include "baseline.hpp"

enum class AlgoState
{

    null,
    lie,
    stand2,
    stand,
    lie2,
    walk,
    slowDown,
    fall,
    recover,
    backFlip,
    handStand,
    walk3,
    walk4,
    upright,
    walk5,
    upright2walk,
    total,  // 用于计数
};
using RunState = AlgoState;

// 步态类型
enum class GaitType
{
    tort,    // 行走
    stroll,  // 踱步
    bound,
    pace,
    Gallop,
    pronk,
    null,  // 起始步态，为了与初始期望值不等，从而进入步态参数选择函数
};

enum class WalkMode
{
    normal,
    climb,
    light,
    aiNormal,
    aiClimb
};

struct ControllerData
{
    AlgoState state;
    WalkMode mode;
    GaitType gait;
    double height;
    Vec3<double> linearV;
    Vec3<double> angularV;
    Vec3<double> pose;
    bool holdFlag;
    double loadMass = 0;
    ControllerData();
};

template <typename T>
struct VelocityType
{
    T x;
    T y;
    T z;
    VelocityType() {}
    VelocityType(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
    T operator()(int i)
    {
        if (i == 0) {
            return x;
        } else if (i == 1) {
            return y;
        } else if (i == 2) {
            return z;
        } else {
            throw std::range_error("index must < 3!");
        }
    }
};

template <typename T>
struct PoseType
{
    T roll;
    T pitch;
    T yaw;
    PoseType() {}
    PoseType(T r, T p, T y) : roll(r), pitch(p), yaw(y) {}
    T operator()(int i)
    {
        if (i == 0) {
            return roll;
        } else if (i == 1) {
            return pitch;
        } else if (i == 2) {
            return yaw;
        } else {
            throw std::range_error("index must < 3!");
        }
    }
};

struct ParamRange
{
    struct
    {
        // ValueRange<double> x{-0.8, 1.5};
        ValueRange<double> x{-2.5, 2.5};
        ValueRange<double> y{-0.6, 0.6};
        ValueRange<double> z{0.0, 0.0};
    } LinearVelocity;

    struct
    {
        ValueRange<double> x{0.0, 0.0};
        ValueRange<double> y{0.0, 0.0};
        // ValueRange<double> z{-0.8, 0.8};
        ValueRange<double> z{-0.785, 0.785};
    } AngularVelocity;

    struct
    {
        ValueRange<double> roll{-0.1, 0.1};
        ValueRange<double> pitch{-0.3, 0.3};
        ValueRange<double> yaw{-0.3, 0.3};
    } Pose;

    ValueRange<double> height{0.0, 0.0};

    ValueRange<double> loadMass{0, 3};

    std::vector<AlgoState> runState{AlgoState::lie, AlgoState::stand, AlgoState::walk};
    std::vector<WalkMode> walkMode{WalkMode::normal, WalkMode::climb, WalkMode::light};
    std::vector<std::string> motorState{"enable", "disable"};
    std::vector<GaitType> gaitType{GaitType::tort};
};
