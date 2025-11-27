
#include "ctrlBase.hpp"

namespace ctrl
{

/**
 * @description: 比例值转实际值
 * @param v
 * @param range
 * @return {}
 */
double Ratio2Value(double v, ValueRange<double> range)
{
    v = std::max(std::min(v, 1.0), -1.0);  // 限制比例值

    // 范围有正有负，正负系数独立计算
    if ((range.min <= 0) && (range.max >= 0)) {
        return (v >= 0) ? (range.max * v) : (range.min * (-v));
    }

    // 范围永远为正值，那负值系数无意义
    if ((range.min >= 0) && (range.max >= 0)) {
        return (v >= 0) ? (range.min + (range.max - range.min) * v) : 0;
    }

    // 范围永远为负值，那正值系数无意义
    if ((range.min <= 0) && (range.max <= 0)) {
        return (v <= 0) ? (range.min + (range.max - range.min) * (-v)) : 0;
    }
    // LOG_ERROR("Ratio2Value reach unexpected!");
    return 0;  // 决不会运行到这里
}

/**
 * @description: 实际值转比例值
 * @param r
 * @param range
 * @return {}
 */
double Value2Ratio(double r, ValueRange<double> range)
{
    if (r < range.min) {
        // LOG_ERROR("Value2Ratio < range min! return min");
        return range.min;
    }
    if (r > range.max) {
        // LOG_ERROR("Value2Ratio > range max! return max");
        return range.max;
    }

    // 范围有正有负，正负系数独立计算
    if ((range.min <= 0) && (range.max >= 0)) {
        return (r >= 0) ? (r / range.max) : (r / range.min);
    }

    // 范围永远为正值，比例一定是正值
    if ((range.min >= 0) && (range.max >= 0)) {
        return (r - range.min) / (range.max - range.min);
    }

    // 范围永远为负值，比例一定是负值
    if ((range.min <= 0) && (range.max <= 0)) {
        return (r - range.max) / (range.max - range.min);
    }

    // LOG_ERROR("Value2Ratio reach unexpected!");
    return 0;  // 决不会运行到这里
}

bool DoubleIsEqual(double a, double b) { return fabs(a - b) < 0.001 ? true : false; }
}  // namespace ctrl