
#pragma once
#include <list>

#include "armCore.hpp"
#include "armMathTools.hpp"
#include "coreData.hpp"
// #include "cppSqlite.hpp"

class ArmCore6 : public ArmCore
{
public:
    ArmCore6(const ArmBootArgs &armArgs);
    void Run() override;

private:
    Vec6<double> qRecv_;
    Vec6<double> qOld_;

    bool Reset_ = false;
    double PY_ = 0.0;
    double QY_ = 0.0;

    std::vector<msg::arm_cmd> planlist_;

private:
    // void ArmParam();
    MsgType RxMoveSearch(const MsgType &in);
    MsgType RxTxRxCallback(const MsgType &in);
    MsgType RxInfoCallback(const MsgType &in);

    void PubInfo();

    // 由电机侧回调的指令
    std::optional<msg::arm_cmd> TxRxCallback(const msg::arm_data &ret);
    void InfoCallback(const msg::arm_motor_info &info);

private:
    void SaveAngle(Vec6<double> q, Vec6<double> qd);
    double ComputeT(Vec6<double> S, Vec6<double> V, Vec6<double> Amax, MoveFlag flag);
    Result<double> MoveJ_t(Vec6<double> Targetjoint, double userV, double userA, MoveMode moveMode, MoveFlag flag);
    Result<double> MoveL_t(Pose Targetpose, double userV, double userA, std::string ToolName, MoveMode moveMode, MoveFlag flag);
    void CalculateVelocity();
    RetState RxSetPositionLimit(const MsgType &in);
    RetState RxSetJointSpeedLimit(const MsgType &in);
    RetState RxSetJointAccLimit(const MsgType &in);
};
