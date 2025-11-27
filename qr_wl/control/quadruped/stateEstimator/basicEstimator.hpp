

#pragma once
#include <deque>
#include <functional>
#include <iostream>
#include <memory>

#include "imuData.hpp"
#include "jpCtrlData.hpp"
#include "mathTools.hpp"
#include "quadrupedParam.hpp"
#include "stateEstimatorData.hpp"

// 过滤器类模板，参数为类型，过滤个数
template <typename T, size_t N = 10>
class Filter
{
public:
    T MeanFilter(const T &data)
    {
        sum += data;                // 加新值
        dataDeque.push_back(data);  // 队尾添加
        if (dataDeque.size() > N) {
            sum -= dataDeque.front();  // 减旧值
            dataDeque.pop_front();     // 删除队头
        }
        return sum / dataDeque.size();
    }

    // 此函数只能被使用于float，其它类型将导致错误
    // 但其他类型也会定义该函数，目前无修改方案
    double MeanFilterRpy(const double &data)
    {
        if (!dataDeque.empty()) {
            //// 如果发生阶跃，就把滤波缓存值全部替换为当前值，经过滤波后仍然为最新值,解决+pi~-pi之间阶跃
            if (fabs(data - dataDeque.back()) > 1.0) {
                dataDeque.clear();
                dataDeque.push_back(data);
                sum = data;
                return sum;
            }
        }

        return MeanFilter(data);
    }

    void Fill(const T &data)
    {
        dataDeque.clear();
        dataDeque.push_back(data);  // 队尾添加
        sum = data;
    }

private:
    std::deque<T> dataDeque;
    T sum;
};

class BasicEstimator
{
public:
    BasicEstimator();
    void Init();
    void Run(AlgoState sta);
    void LoadData(const ImuData &imuData);
    void LoadData(const DataCmd &motorData);
    void LoadData(const JpDataOutput &jpData);
    const StateEstimatorOutputData &GetResult() const;

private:
    Vec4<bool> realContactCheck(Mat34<double> fFoot, double valueSim, double valueReal);
    Mat34<double> forwardDynamics(Mat34<double> tau, Mat34<double> q);
    Mat34<double> forwardKinematics(Mat34<double> q);
    double calBodyHeight(Vec4<double> rtHeight, Vec4<bool> contactFlag, Vec4<bool> realContactFlag);
    Mat3<double> calGroundR(Mat34<double> pFootCur, Vec4<bool> contactFlagCur, Mat3<double> imuR);
    Vec3<double> calGroundR2Body(Mat34<double> pFootCur, Vec4<bool> contactFlagCur, Vec3<double> rpyOld);

    Mat3<double> calFootR(Mat34<double> pFootCur, Mat34<double> fFootCur, Vec4<bool> contactFlagCur);
    Mat34<double> calFootVelocity(Mat34<double> q, Mat34<double> qd);
    Vec3<double> calBodyMeanVelocity(Mat34<double> vFootMat, Vec4<bool> contactFlagVec);
    int calFallDown(Vec3<double> rpyCur, AlgoState curState);
    Vec4<bool> realContactCheckMiddle(Mat34<double> fFoot);

    std::pair<bool, Vec2<double>> contactDetector(double sPhi, double phi, double muc[2], double variance[2], double pz, double muzg, double varianceZg, double fz, double mufc,
                                                  double varianceFc, double Pc);
    double b_erf(double x);
    double rt_powd_snf(double u0, double u1);

    DMat<double> vCom100 = DMat<double>::Zero(3, 100);

private:
    MathFun mathFun;

    // 仿真
    // Filter<double, 20> filterRpy[3];
    // Filter<Vec3<double>, 20> filterW;
    // Filter<Vec3<double>, 100> filterA;
    // 实物
    Filter<double, 10> filterRpy[3];
    Filter<Vec3<double>, 10> filterW;
    // Filter<Vec3<double>, 100> filterA;
    Filter<Vec3<double>, 200> filterA;

    Filter<double, 1> rpySourceData[3];

    AlgoState _curState{AlgoState::lie};  // 当前状态

    Vec4<std::deque<double>> fFootData;

    ImuData imuData_;
    DataCmd motorData_;
    JpDataOutput jpData_;
    StateEstimatorOutputData output_;
    const QuadrupedParam &qrParam{GetQrParam()};

    // double rpy0Bias_ = 0;
};