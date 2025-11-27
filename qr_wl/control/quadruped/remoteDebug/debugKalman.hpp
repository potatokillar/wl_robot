
#pragma once
#include <vector>

#include "../pub/contrType.hpp"
#include "debug.hpp"

class KalmanEstimator;
class DebugKalmanEstimator
{
public:
    void Init(KalmanEstimator* set)
    {
        obj_ = set;

        contrParamHotUpdate.AddParamNum(
            "kalman-p",
            [this](std::vector<double> set) { return this->set_param(set); },
            [this]() { return this->get_param(); },
            [this]() { return this->get_param_info(); });
    }

    bool set_param(std::vector<double> set);
    std::vector<double> get_param();
    ParamInfo get_param_info();

private:
    KalmanEstimator* obj_;
    ValueRange<double> param{-10000, 10000};
    static constexpr uint P_COUNT = 6;  // 数量是6
};