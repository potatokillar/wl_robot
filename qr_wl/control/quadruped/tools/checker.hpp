
#pragma once
#include "baseline.hpp"
#include "contrType.hpp"
#include "innerType.hpp"

class Checker
{
public:
    DataCmd &Lie(DataCmd &motorCmd);
    DataCmd &LieDown(DataCmd &motorCmd);

    DataCmd &StandUp(DataCmd &motorCmd);
    DataCmd &Stand(DataCmd &motorCmd);

    DataCmd &Walk(DataCmd &motorCmd);
    DataCmd &Fall(DataCmd &motorCmd);

    DataCmd &Recover(DataCmd &motorCmd);
    DataCmd &BackFlip(DataCmd &motorCmd);

private:
    DataCmd &checkCmd(DataCmd &motorCmd, const Mat43<double> &range);
    DataCmd motorCmdOld_;
};

// 暂时放这里
std::optional<Mat34<double>> VecToMat34(const std::vector<double> &set);
std::vector<double> Mat34ToVec(const Mat34<double> &set);