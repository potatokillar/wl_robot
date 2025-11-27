#include "kalmanEstimator.hpp"

#include <iostream>

#include "baseline.hpp"
#include "ctrlSendData.hpp"
#include "orientation_tools.h"
using namespace ::std;
using namespace ori;

KalmanEstimator::KalmanEstimator()
{
    Init();

    _klTask = make_unique<PeriodicMemberFunction<KalmanEstimator>>("kl", 0.002, this, &KalmanEstimator::Loop, true);
    _klSemaphore = make_unique<SimpleSemaphore>();
    _klSemaphore->init(0);
    _klTask->Start();

    debug_.Init(this);
}

void KalmanEstimator::LoadData(const StateEstimatorOutputData& data)
{
    unique_lock lock(mutex_, TimerTools::ToUs(100));
    input_ = data;
}
void KalmanEstimator::LoadData(const JpDataOutput& data)
{
    unique_lock lock(mutex_, TimerTools::ToUs(100));
    jpData_ = data;
}

void KalmanEstimator::Run(AlgoState sta)
{
    // 多线程时间，此处为空
    _curState = sta;
}

void KalmanEstimator::Loop()
{
    // 实时更改卡尔曼滤波器系数，所以这里这样写，确定以后，写到Init里面去
    // aKalman.P.setIdentity();
    // aKalman.P = paramList_[0] * aKalman.P;
    // aKalman.A.setIdentity();
    // aKalman.A = paramList_[1] * aKalman.A;
    // aKalman.B.setIdentity();
    // aKalman.B = paramList_[2] * aKalman.B;
    // aKalman.Q.setIdentity();
    // aKalman.Q = paramList_[3] * aKalman.Q;
    // aKalman.H.setIdentity();
    // aKalman.H = paramList_[4] * aKalman.H;
    // aKalman.R.setIdentity();
    // aKalman.R = paramList_[5] * aKalman.R;

    Vec9<double> imuVec;
    Vec3<double> aDesire;
    Vec3<double> vCom;
    // Mat3<double> bodyR;
    double tempYaw;
    {
        unique_lock lock(mutex_);
        output_.quat = input_.quat;
        output_.fFoot = input_.fFoot;
        output_.imuVec = input_.imuVec;
        output_.pFoot = input_.pFoot;
        output_.vFoot = input_.vFoot;
        output_.bodyHeight = input_.bodyHeight;
        output_.groundR = input_.groundR;
        output_.flag.fallDownInfo = input_.flag.fallDownInfo;
        output_.realContactFlag = input_.realContactFlag;
        output_.pRef = output_.pRef;
        // output_.rpyGround2Body = input_.rpyGround2Body;
        output_.rtHeight = input_.rtHeight;

        aDesire = jpData_.aDesire;
        vCom = input_.vCom;
        // aCom = input_.aCom;
        // bodyR = input_.bodyR;
        imuVec = input_.imuVec;
        tempYaw = input_.imuVec(2);  // 记录进来时的yaw角，目的是不经过kalman滤波,实时线程保证

        output_.vCom = vComOld_;  // 若kalman未及时计算，则回复上一次计算的值
        // output_.aCom = aComOld_;
        // output_.imuVec = imuOld_;
        output_.rpy = imuOld_.block(0, 0, 3, 1);
        output_.wCom = imuOld_.block(3, 0, 3, 1);
        output_.aCom = imuOld_.block(6, 0, 3, 1);
    }
    // a
    /*
    Vec3<double> da;
    da.setZero();
    updateFilter3(&aKalman, aCom, da);
    aCom = aKalman.x;
    aCom(2) = 9.8;
    */
    // imu
    Vec9<double> u;
    u << imuVec(3), imuVec(4), imuVec(5), 0, 0, 0, 0, 0, 0;  // 姿态角的控制量近似是角速度
    updateFilter9(&imuKalman, imuVec, u);
    imuVec = imuKalman.x;
    imuVec(2) = tempYaw;
    imuVec(8) = 9.8;

    Vec3<double> tempRPY = imuVec.block(0, 0, 3, 1);
    Mat3<double> tempbodyR = rpyToRotMat(tempRPY).transpose();  // 身体转世界

    // v
    Vec3<double> aDesireBody = tempbodyR.transpose() * (aDesire - G);  // 世界转身体
    updateFilter3(&vKalman, vCom, aDesireBody);
    vCom = vKalman.x;
    vCom(2) = 0;
    /*
    //  感兴趣的数据自行填充
    Vec3<double> a, b, c, ra, rb, rc;
    a.setZero();
    b.setZero();
    c.setZero();
    ra.setZero();
    rb.setZero();
    rc.setZero();

    a = vComBasic;
    // ra = a;
    ra = vComKalmanOld;
    b = vComKalmanOld;
    rb = vComKalmanNew;

    c = vComBasic;
    rc = vComKalmanNew;
    // c = aDesireBody;
    // rc = c;

    //
    */
    PublishBodyCurrent(imuVec.block(0, 0, 3, 1), imuVec.block(3, 0, 3, 1), imuVec.block(6, 0, 3, 1), input_.quat);
    vComOld_ = vCom;
    // aComOld_ = aCom;
    imuOld_ = imuVec;
    {
        unique_lock lock(mutex_);
        output_.vCom = vCom;
        output_.bodyR = tempbodyR;
        // output_.aCom = aCom;
        // output_.imuVec = imuVec;
        output_.rpy = imuVec.block(0, 0, 3, 1);
        output_.wCom = imuVec.block(3, 0, 3, 1);
        output_.aCom = imuVec.block(6, 0, 3, 1);
    }
}

/**
 * @description: 增加互斥保护的获取数据接口
 * @param {*}
 * @return {*}
 */
const StateEstimatorOutputData& KalmanEstimator::GetResult()
{
    unique_lock lock(mutex_, TimerTools::ToUs(100));
    if (lock.owns_lock()) {
        outputOld_ = output_;
        return output_;
    } else {
        return outputOld_;
    }
}

void KalmanEstimator::updateFilter3(KalmanPara* para, Vec3<double> y, Vec3<double> u)
{
    // 预测方程
    para->x = para->A * para->x + para->B * u;
    para->P = para->A * para->P * para->A.transpose() + para->Q;

    // 更新方程
    para->K = para->P * para->H.transpose() * (para->H * para->P * para->H.transpose() + para->R).inverse();
    para->x = para->x + para->K * (y - para->H * para->x);
    para->P = (para->I - para->K * para->H) * para->P;
}

void KalmanEstimator::updateFilter9(KalmanPara9* para, Vec9<double> y, Vec9<double> u)
{
    // 预测方程
    para->x = para->A * para->x + para->B * u;
    para->P = para->A * para->P * para->A.transpose() + para->Q;

    // 更新方程
    para->K = para->P * para->H.transpose() * (para->H * para->P * para->H.transpose() + para->R).inverse();
    para->x = para->x + para->K * (y - para->H * para->x);
    para->P = (para->I - para->K * para->H) * para->P;
}

void KalmanEstimator::Init()
{
    // 接受界面更改的参数
    paramList_[0] = 1.0;    // P
    paramList_[1] = 0.99;   // A  可快速收敛
    paramList_[2] = 0.002;  // B

    paramList_[3] = 2.0;   // Q   噪声
    paramList_[4] = 1.0;   // H
    paramList_[5] = 2000;  // R   噪声，越大过滤越明显
    // 速度滤波初始化
    vKalman.x.setZero();
    vKalman.y.setZero();
    vKalman.I.setIdentity();
    vKalman.u.setZero();

    vKalman.P.setIdentity();
    vKalman.P = 1.0 * vKalman.P;
    vKalman.A.setIdentity();
    vKalman.A = 1.0 * vKalman.A;
    vKalman.B.setIdentity();
    vKalman.B = 0.002 * vKalman.B;
    vKalman.Q.setIdentity();
    vKalman.Q = 1.3 * vKalman.Q;
    vKalman.H.setIdentity();
    vKalman.H = 1.2 * vKalman.H;
    vKalman.R.setIdentity();
    vKalman.R = 100 * vKalman.R;

    // aKalman.x.setZero();
    // aKalman.y.setZero();
    // aKalman.I.setIdentity();
    // aKalman.u.setZero();

    // aKalman.P.setIdentity();
    // aKalman.P = paramList_[0] * aKalman.P;
    // aKalman.A.setIdentity();
    // aKalman.A = paramList_[1] * aKalman.A;
    // aKalman.B.setIdentity();
    // aKalman.B = paramList_[2] * aKalman.B;
    // aKalman.Q.setIdentity();
    // aKalman.Q = paramList_[3] * aKalman.Q;
    // aKalman.H.setIdentity();
    // aKalman.H = paramList_[4] * aKalman.H;
    // aKalman.R.setIdentity();
    // aKalman.R = paramList_[5] * aKalman.R;

    imuKalman.x.setZero();
    imuKalman.y.setZero();
    imuKalman.u.setZero();
    imuKalman.I.setIdentity();

    Vec9<double> vecP, vecA, vecB, vecQ, vecH, vecR;
    vecP << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    vecA << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.95, 0.95, 0.95;
    vecB.fill(0.002);
    vecQ << 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 2.0, 2.0, 2.0;
    vecH << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    vecR << 1000, 1000, 1000, 2000, 2000, 2000, 8000, 8000, 8000;
    imuKalman.P.diagonal() = vecP;  // 参数需要调整
    imuKalman.A.diagonal() = vecA;
    imuKalman.B.diagonal() = vecB;
    imuKalman.Q.diagonal() = vecQ;
    imuKalman.H.diagonal() = vecH;
    imuKalman.R.diagonal() = vecR;
    // imuKalman.R.diagonal()(2) = 20.0;
    //  cout << "R = " << imuKalman.R << endl;
}