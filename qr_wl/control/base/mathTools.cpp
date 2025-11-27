

#include "mathTools.hpp"

#include <iostream>

#include "baseline.hpp"

using namespace ::std;
namespace ctrl
{
double MathBaseTool::SetValueRange(double value, double min, double max, const std::string& str, bool isPrint)
{
    double result = 0.0;
    if (max >= min) {
        if (value > max) {
            result = max;
            if (setValueRangePrint_.count(str) == 0) {
                setValueRangePrint_.insert(str);
                if (isPrint) {
                    LOG_WARN("{} > MAX! max:{}, value:{}", str, max, value);
                }
            }
        } else if (value < min) {
            result = min;
            if (setValueRangePrint_.count(str) == 0) {
                setValueRangePrint_.insert(str);
                if (isPrint) {
                    LOG_WARN("{} < MIN! min:{}, value:{}", str, min, value);
                }
            }
        } else {
            result = value;
            setValueRangePrint_.erase(str);
        }
    } else {
        if (isPrint) {
            LOG_ERROR("MIN < MAX! min:{}, max:{}", min, max);
        }
    }
    return result;
}
}  // namespace ctrl