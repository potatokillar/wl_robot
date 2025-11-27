
#pragma once
#include <onnx/onnxruntime_cxx_api.h>

#include "baseline.hpp"
#include "contrType.hpp"
#include "fsmState.hpp"
#include "imuData.hpp"
#include "innerType.hpp"
#include "jpCtrlData.hpp"
#include "mathTools.hpp"

class FsmUpright2Walk : public FsmState
{
public:
    FsmUpright2Walk(ContrContainer contr, std::shared_ptr<JpData> jpData);
    void OnEnter() override;
    void Run() override;
    void OnExit() override;

    void Middle();
    void MiddleWheel();
    void Small();
    void SmallWheel();

private:
    u64 interval_{0};
    u64 duration_{80};
    ImuData imu_;
};