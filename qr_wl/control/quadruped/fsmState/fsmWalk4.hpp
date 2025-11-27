
#pragma once
#include "fsmState.hpp"

class FsmWalk4 : public FsmState
{
public:
    FsmWalk4(ContrContainer contr, std::shared_ptr<JpData> jpData);
    void OnEnter() override;
    void Run() override;
    void OnExit() override;

    void Middle();
    void MiddleWheel();
    void Small();
    void SmallWheel();

private:
    u64 interval_{0};
    u64 durations_{100};
    // CmdVal2 motorCmd_;
};