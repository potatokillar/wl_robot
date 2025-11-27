
#include "debugKalman.hpp"

#include "kalmanEstimator.hpp"

bool DebugKalmanEstimator::set_param(std::vector<double> set)
{
    if (set.size() != P_COUNT) {
        return false;
    }

    for (unsigned int i = 0; i < P_COUNT; i++) {
        if ((set[i] < param.min) || (set[i] > param.max)) {
            return false;
        }
        obj_->paramList_[i] = set[i];
    }
    return true;
}
std::vector<double> DebugKalmanEstimator::get_param()
{
    std::vector<double> ret;

    std::copy(obj_->paramList_.begin(), obj_->paramList_.end(), std::back_inserter(ret));
    return ret;
}
ParamInfo DebugKalmanEstimator::get_param_info()
{
    ParamInfo info;
    info.type = ParamType::range;
    info.num.push_back(param.min);
    info.num.push_back(param.max);
    return info;
}
