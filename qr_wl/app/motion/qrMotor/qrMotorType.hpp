
#pragma once
#include "baseline.hpp"
#include "transMotorMit.hpp"

struct LegWheelMotorCfg
{
    std::array<std::array<MitMotorCfg, 4>, 4> motorCfg;
    BootArgs boot;
};

struct QrMotorInfo
{
    MitMotorCfg config;
    u64 lastRecv{TimerTools::GetNowTickMs()};
    u32 sendCnt{0};
    u32 recvCnt{0};
    double qBias{0};
    std::unique_ptr<TransformMitMotor> trans;
    std::string name;
    MotorState state{MotorState::disable};  //
};
