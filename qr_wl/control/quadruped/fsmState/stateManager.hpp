
#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include <queue>
#include <vector>

#include "eventMsg.hpp"
#include "fsmState.hpp"
#include "jpCtrlData.hpp"

struct StateSwitch
{
    AlgoState from{AlgoState::null};
    AlgoState to{AlgoState::null};
    AlgoState trans{AlgoState::null};

    StateSwitch(AlgoState src, AlgoState dst, AlgoState mid) : from(src), to(dst), trans(mid) {}
};

class StateManager
{
public:
    StateManager(const BootArgs& args, const QrBootArgs& qrArgs);
    ~StateManager();
    void Start();
    void Run();
    void Stop();

    bool Register(AlgoState from, AlgoState to, AlgoState trans = AlgoState::null);

private:
    void CreateState(ContrContainer contr, std::shared_ptr<JpData> jpData);
    void SwitchNext();
    void GetNextState();
    void RxQrEvent(QrEventType type);

    MsgType RxSetRunState(const MsgType& set);
    std::vector<RpcRecv> rpcRxDeal_;

    // 硬件健康检查函数
    bool CheckMotorsHealthy();
    bool CheckRobotStateOk();

    AlgoState query_{AlgoState::lie};
    AlgoState queryForce_{AlgoState::null};

    AlgoState curState_{AlgoState::lie};
    std::vector<StateSwitch> canSwitch_;
    std::queue<AlgoState> execList_;

    std::shared_ptr<JpData> jpData_;
    std::map<AlgoState, std::unique_ptr<FsmState>> staMap_;

    std::unique_ptr<PeriodicMemberFunction<StateManager>> task_;
};