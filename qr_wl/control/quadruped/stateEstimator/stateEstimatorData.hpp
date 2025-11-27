
#pragma once
#include "baseline.hpp"
#include "quadrupedParam.hpp"

// 输出的数据
class StateEstimatorOutputData
{
public:
    Vec9<double> imuVec;
    Mat34<double> pRef;  // 参考触地概率
    Vec3<double> vCom;
    Vec3<double> wCom;
    Vec3<double> aCom;
    Vec4<bool> realContactFlag;
    Mat34<double> pFoot;
    Mat34<double> vFoot;
    Mat34<double> fFoot;
    Mat3<double> bodyR;
    Mat3<double> groundR;
    // Vec3<double> rpyGround2Body;
    Vec3<double> rpy;
    // Vec3<double> rpySourceData;
    Vec4<double> rtHeight;
    Vec4<double> quat;
    double bodyHeight;
    struct Flag
    {
        int fallDownInfo = 0.0;  // 表示摔倒方向, 0表示不摔，1表示绕x轴正摔倒，-1表示绕x轴负摔倒
                                 // bool holdWalk = false;
    } flag;

    StateEstimatorOutputData()
    {
        auto &qrParam = GetQrParam();
        pRef.fill(1);
        vCom.setZero();
        wCom.setZero();
        aCom.setZero();
        realContactFlag.fill(false);
        pFoot = qrParam.pFoot;
        vFoot.setZero();
        fFoot.setZero();
        bodyR.setIdentity();
        groundR.setIdentity();
        // rpyGround2Body.setZero();
        rpy.setZero();
        // rpySourceData.setZero();
        bodyHeight = qrParam.bodyHeight;
        rtHeight.fill(qrParam.bodyHeight);
        vCom.setZero();
    }
};

using StateEstimatorInputData = StateEstimatorOutputData;