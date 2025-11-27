
#pragma once
#include "baseline.hpp"

namespace ctrl
{
class MathBaseTool
{
public:
    double SetValueRange(double value, double min, double max, const std::string& str, bool isPrint = true);

private:
    bool ikErrFlag;
    std::set<std::string> setValueRangePrint_;
};
}  // namespace ctrl