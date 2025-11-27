
#pragma once

#include "cppType.hpp"

// 参数范围类型
template <typename T>
struct ValueRange
{
    T min;
    T max;
    ValueRange() {}
    ValueRange(const T& a, const T& b) : min(a), max(b) {}
};

struct BootArgs
{
    bool noMotor{false};
    bool isSim{false};
};

inline BootArgs bootArgs;
