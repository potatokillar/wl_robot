
#include "stateEstimatorContainer.hpp"

/**
 * @description: 数据载入
 * @param imuData
 * @return {}
 */
void StateEstimatorContainer::LoadData(const ImuData& imuData) { basicEstimator.LoadData(imuData); }
void StateEstimatorContainer::LoadData(const DataCmd& motorData) { basicEstimator.LoadData(motorData); }
void StateEstimatorContainer::LoadData(const JpDataOutput& jpData)
{
    basicEstimator.LoadData(jpData);
    kalmanEstimator.LoadData(jpData);
}

void StateEstimatorContainer::Init()
{
    basicEstimator.Init();
    outputData_ = basicEstimator.GetResult();

    kalmanEstimator.LoadData(outputData_);
    kalmanEstimator.Init();
    outputData_ = kalmanEstimator.GetResult();
}

/**
 * @description: 数据运行
 * @param sta
 * @return {}
 */
void StateEstimatorContainer::Run(AlgoState sta)
{
    basicEstimator.Run(sta);
    outputData_ = basicEstimator.GetResult();

    kalmanEstimator.LoadData(outputData_);  // 把基础估计器的输出作为卡尔曼估计器的输入
    kalmanEstimator.Run(sta);
    outputData_ = kalmanEstimator.GetResult();
}
