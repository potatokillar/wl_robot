
#pragma once
#include "baseline.hpp"
#include "mathTools.hpp"

namespace ctrl
{
template <typename T>
struct VelocityType
{
    T x;
    T y;
    T z;
    VelocityType() {}
    VelocityType(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
    T& operator()(int i)
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

/**
 * @description: 比例值转实际值
 * @param v
 * @param range
 * @return {}
 */
double Ratio2Value(double v, ValueRange<double> range);

/**
 * @description: 实际值转比例值
 * @param r
 * @param range
 * @return {}
 */
double Value2Ratio(double r, ValueRange<double> range);

bool DoubleIsEqual(double a, double b);
};  // namespace ctrl