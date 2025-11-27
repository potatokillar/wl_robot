
#pragma once
#include <memory>

#include "checker.hpp"
#include "cmdManager.hpp"
#include "ctrlRecvData.hpp"
#include "ctrlSendData.hpp"
#include "jpCtrlData.hpp"
#include "stateEstimatorContainer.hpp"

// 所有控制需要算法类的集合
struct ContrContainer
{
    std::shared_ptr<CtrlSendData> txData;
    std::shared_ptr<CtrlRecvData> rxData;
    std::shared_ptr<StateEstimatorContainer> staEstimator;
    std::shared_ptr<CmdManager> cmdManager;
    AlgoModel model{AlgoModel::jp};
};

class FsmState
{
public:
    FsmState(AlgoState sta, ContrContainer contr, std::shared_ptr<JpData> jpData);
    virtual ~FsmState() {};
    virtual void PrevRun();      // 每个状态都会先运行，每次都运行
    virtual void OnEnter() = 0;  // 进入该状态的函数，进入状态运行一次
    virtual void Run() = 0;      // 状态运行时的函数，每次都运行，处理状态的特定算法
    virtual void OnExit() = 0;   // 离开该状态的函数，离开状态运行一次
    virtual void AfterRun();     // 每个状态后都会运行，每次都运行

    bool IsDone() const { return done_; }
    AlgoState GetName() const { return state_; }
    // bool IsFall() const { return fall_; }

protected:
    void SetCurData(CurPara* cur);

    AlgoState state_;

    std::shared_ptr<JpData> jpData_;
    std::shared_ptr<CtrlSendData> txData_;  // 计算完成返回出去的输出数据
    std::shared_ptr<CtrlRecvData> rxData_;  // 外部获取的输入数据
    std::shared_ptr<StateEstimatorContainer> staEstimator_;
    std::shared_ptr<CmdManager> cmd_;

    bool done_{false};
    MathFun mathFun_;
    const QuadrupedParam& qrParam{GetQrParam()};

private:
};