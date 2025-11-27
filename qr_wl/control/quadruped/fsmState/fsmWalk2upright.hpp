
#pragma once
#include <onnx/onnxruntime_cxx_api.h>

#include "baseline.hpp"
#include "contrType.hpp"
#include "fsmState.hpp"
#include "imuData.hpp"
#include "innerType.hpp"
#include "jpCtrlData.hpp"
#include "mathTools.hpp"

class FsmWalk2Upright : public FsmState
{
public:
    FsmWalk2Upright(ContrContainer contr, std::shared_ptr<JpData> jpData);
    void OnEnter() override;
    void Run() override;
    void OnExit() override;

    void Middle();
    void MiddleWheel();
    void Small();
    void SmallWheel();

private:
    int ti{0};
    Vec6<int> Ti, deltaTi;
    Mat34<double> tmpCmd0, tmpCmd1, tmpCmd2, tmpCmd3, tmpCmd4, tmpCmd5, tmpCmd6;
};