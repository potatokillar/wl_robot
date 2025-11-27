
#pragma once

#include <memory>
#include <vector>

#include "basicEstimator.hpp"
#include "jpCtrlData.hpp"
#include "kalmanEstimator.hpp"
#include "stateEstimatorData.hpp"

class StateEstimatorContainer
{
public:
    void Init();
    // 恢复到最初状态
    void DeInit() { Init(); }

    void LoadData(const ImuData& imuData);
    void LoadData(const DataCmd& motorData);
    void LoadData(const JpDataOutput& jpData);
    const StateEstimatorOutputData& GetResult() const { return outputData_; }

    void Run(AlgoState sta);

private:
    StateEstimatorOutputData outputData_;

    BasicEstimator basicEstimator;    // 基础估计器
    KalmanEstimator kalmanEstimator;  // 卡尔曼估计器
};