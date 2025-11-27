
#pragma once
#include "fsmState.hpp"
class FsmLie : public FsmState
{
public:
    FsmLie(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::lie, contr, jpData) {}

    void OnEnter() override;
    void Run() override;
    void OnExit() override;
};