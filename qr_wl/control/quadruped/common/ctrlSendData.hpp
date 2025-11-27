
#pragma once

#include "Transformer.hpp"
#include "debugJpos.hpp"
#include "quadrupedParam.hpp"

// 广播相关数据
void PublishBodyDesire(const Vec3<double> &rpyDesire, const Vec3<double> &wDesire, const Vec3<double> &aDesire, const Vec4<double> &quat);
void PublishBodyCurrent(const Vec3<double> &rpyFilter, const Vec3<double> &wFilter, const Vec3<double> &aFilter, const Vec4<double> &quat);
void PublishInterestData(const Vec3<double> &a, const Vec3<double> &ra, const Vec3<double> &b, const Vec3<double> &rb, const Vec3<double> &c, const Vec3<double> &rc);
void PublishInterestDataA(const Vec3<double> &a, const Vec3<double> &ra);
void PublishInterestDataB(const Vec3<double> &b, const Vec3<double> &rb);
void PublishInterestDataC(const Vec3<double> &c, const Vec3<double> &rc);

class CtrlSendData
{
    friend class DebugCtrlSendData;
    DebugCtrlSendData debug_;

public:
    CtrlSendData(bool fromSim);

    void SendMotorCmd(const DataCmd &motorCmd);
    void SendMotorCmd(CmdVal2 motorCmd);
    void SendMotorCmd2(const CmdVal2 &motorCmd);

    void SendMotorCmd(DataValue2 motorCmd);
    void SendMotorCmd(const CmdVal4 &motorCmd);
    void SendMotorCmd2(DataValue2 motorCmd);
    void SendMotorCmdFixed(DataValue2 motorCmd);

    void SetCxdV3(const Mat34<double> &data);
    void SetCxdV3(const Mat44<double> &data);

    void SetCurState(const AlgoState &sta) { _curState = sta; }
    void SetFromSim(bool set) { fromSim_ = set; }

private:
    CmdVal2 MotorCheck(const CmdVal2 &data);
    CmdVal2 MotorCheck2(const CmdVal2 &data);
    void CheckCmd(CmdVal2 *motorCmd);
    DataCmd CheckCmd2(const DataCmd &motorCmd, const DataCmd &motorCmdOld);
    void CheckCmd3(DataValue2 *motorCmd);
    void CheckCmd4(DataValue2 *motorCmd);

private:
    bool fromSim_{true};

    DataCmd motorCmdOld_;          // 上一次命令的值，算法坐标系
    DataCmd motorCmdMitOld_;       // 上一次命令的值，MIT坐标系
    DataCmd motorCmdStandard_;     // 标准值，算法坐标系
    DataValue2 motorCmdWheelOld_;  // 轮足式上一次命令

    TransformCoordinate transform;
    AlgoState _curState{AlgoState::lie};
    MotorCtrlPara motorPid[STATE_TOTALNUM];
    const QuadrupedParam &qrParam{GetQrParam()};
    Mat34<double> qLimitUnitreeMarkerMin_, qLimitUnitreeMarkerMax_;
    DataCmd motorCmdUnitreeOld_;

    ValueRange<Mat34<double>> qLimitMit_;
    ValueRange<Mat34<double>> qdLimitMit_;
    ValueRange<Mat34<double>> tauLimitMit_;
};