
#pragma once

#include <chrono>
#include <deque>

#include "aiWalk.hpp"
#include "fsmState.hpp"
#include "qpSolver.hpp"
#include "strategy.hpp"

// 因qpSolver会创建新的线程，因此禁止派生
class FsmWalk final : public FsmState
{
public:
    FsmWalk(ContrContainer contr, std::shared_ptr<JpData> jpData);
    void OnEnter() override;
    void Run() override;
    void OnExit() override;

private:
    AiWalk aiWalk_;
};
