
#pragma once
#include <memory>
#include <mutex>
#include <thread>

#include "baseline.hpp"
#include "debugKalman.hpp"
#include "jpCtrlData.hpp"
#include "quadrupedParam.hpp"
#include "simpleSemaphore.h"
#include "stateEstimatorData.hpp"

struct StateEstimate
{
    Vec4<double> contactEstimate;
    Vec3<double> position;
    Vec3<double> vBody;
    Quat<double> orientation;
    Vec3<double> omegaBody;
    RotMat<double> rBody;
    Vec3<double> rpy;
    Vec3<double> omegaWorld;
    Vec3<double> vWorld;
    Vec3<double> aBody, aWorld;

    StateEstimate()
    {
        contactEstimate.fill(1.0);
        position.setZero();
        vBody.setZero();
        orientation << 1.0, 0.0, 0.0, 0.0;
        omegaBody.setZero();
        rBody.setIdentity();
        rpy.setZero();
        omegaWorld.setZero();
        vWorld.setZero();
        aBody = G;
        aWorld = G;
    };
};
struct VectorNavDataKalman
{
    Vec3<double> accelerometer;
    Vec3<double> gyro;
    Quat<double> quat;
    // todo is there status for the vectornav?
    VectorNavDataKalman()
    {
        accelerometer.setZero();
        gyro.setZero();
        quat << 1.0, 0.0, 0.0, 0.0;
    }
};
struct LegControllerData
{
    Vec3<double> q, qd, p, v;
    Mat3<double> J;
    Vec3<double> tauEstimate;
    LegControllerData()
    {
        q.setZero();
        qd.setZero();
        p.setZero();
        v.setZero();
        J.setIdentity();
        tauEstimate.setZero();
    }
};
struct StateEstimatorData
{
    StateEstimate result;  // where to write the output to
    VectorNavDataKalman vectorNavData;
    LegControllerData legControllerData[4];
    Vec4<bool> contactPhase;
    StateEstimatorData() { contactPhase.fill(1); }
};

struct KalmanPara
{
    Vec3<double> x;  // 状态向量
    Vec3<double> u;  // 控制向量
    Vec3<double> y;  // 观测向量

    Mat3<double> P;  // 状态协方差矩阵

    Mat3<double> A;  // 状态转移矩阵
    Mat3<double> B;  // 外部控制系数矩阵

    Mat3<double> Q;  // 状态转移协方差矩阵
    Mat3<double> H;  // 观测矩阵

    Mat3<double> K;  // 卡尔曼增益
    Mat3<double> R;  // 观测噪声协方差
    Mat3<double> I;  // 单位矩阵
};
struct KalmanPara9
{
    Vec9<double> x;  // 状态向量
    Vec9<double> u;  // 控制向量
    Vec9<double> y;  // 观测向量

    Mat9<double> P;  // 状态协方差矩阵

    Mat9<double> A;  // 状态转移矩阵
    Mat9<double> B;  // 外部控制系数矩阵

    Mat9<double> Q;  // 状态转移协方差矩阵
    Mat9<double> H;  // 观测矩阵

    Mat9<double> K;  // 卡尔曼增益
    Mat9<double> R;  // 观测噪声协方差
    Mat9<double> I;  // 单位矩阵
};

// todo因会产生线程，禁止派生
class KalmanEstimator final
{
    friend class DebugKalmanEstimator;
    DebugKalmanEstimator debug_;

public:
    KalmanEstimator();
    ~KalmanEstimator() {};
    void Init();
    void Run(AlgoState sta);
    void LoadData(const StateEstimatorOutputData& data);
    void LoadData(const JpDataOutput& data);

    const StateEstimatorOutputData& GetResult();

private:
    void updateFilter3(KalmanPara* para, Vec3<double> y, Vec3<double> u);
    void updateFilter9(KalmanPara9* para, Vec9<double> y, Vec9<double> u);

private:
    std::unique_ptr<PeriodicMemberFunction<KalmanEstimator>> _klTask;
    std::unique_ptr<SimpleSemaphore> _klSemaphore;  // 信号量
    void Loop();                                    // 多线程循环
    mutable std::timed_mutex mutex_;
    AlgoState _curState{AlgoState::lie};  // 当前状态
    StateEstimatorOutputData output_, input_, outputOld_;
    JpDataOutput jpData_;

    KalmanPara vKalman;
    KalmanPara9 imuKalman;
    std::array<double, 6> paramList_;
    // 上一次的值
    Vec3<double> vComOld_;
    Vec9<double> imuOld_;
};