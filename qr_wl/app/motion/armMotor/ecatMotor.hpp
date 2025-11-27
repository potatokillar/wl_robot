
#pragma once
#include <functional>
#include <future>
#include <thread>

#include "baseline.hpp"

// 驱动器支持的模式，并未全部实现
enum class ServoMode
{
    PP_ProfilePosition = 1,
    PV_ProfileVelocity = 3,
    PT_ProfileTorque = 4,
    // HM_Homing = 6,
    CSP_CyclicSynchronousPosition = 8,
    CSV_CyclicSynchronousVelocity = 9,
    CST_CyclicSynchronousTorque = 10,
};

class EcatMotor
{
public:
    EcatMotor(int slc);
    void OpInit();
    void PreOpInit();

    void SetPosition(double q);
    double GetPosition();
    s32 GetTorque();
    s32 GetVelocity();
    void SetCtrlWord(u32 set);
    u32 GetStatusWord();
    void SetAccVelocity(s32 pulse);
    void SetDecVelocity(s32 pulse);

    u32 GetErrorCode();
    void ClearErrorCode();
    void SetIoOut(u32 io);
    u32 GetIoIn();
    void SetMode(ServoMode mode);

private:
    int slc_;
    s32 circleCnt_;  // 一圈cnt
    s32 biasCnt_;    // 启动时的cnt
    u16 errCode_{0};
};