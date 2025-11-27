
#include "stateManager.hpp"

#include "ctrlRecvData.hpp"
#include "ctrlSendData.hpp"
#include "fsmFall.hpp"
#include "fsmLie.hpp"
#include "fsmLieDown.hpp"
#include "fsmRecover.hpp"
// #include "fsmSlowDown.hpp"
#include "baseline.hpp"
#include "debugParam.hpp"
#include "fsmHandStand.hpp"
#include "fsmStand.hpp"
#include "fsmStandUp.hpp"
#include "fsmUpright.hpp"
#include "fsmUpright2walk.hpp"
#include "fsmWalk.hpp"
#include "fsmWalk2upright.hpp"
#include "fsmWalk3.hpp"
#include "fsmWalk4.hpp"
#include "robotState.hpp"
#include "mitMotorType.hpp"

using namespace ::std;

StateManager::StateManager(const BootArgs &args, const QrBootArgs &qrArgs)
{
    curState_ = AlgoState::lie;

    // 构造算法数据模块
    jpData_ = make_shared<JpData>();
    jpData_->fromSim_ = args.isSim;
    jpData_->noMotor = args.noMotor;

    ContrContainer contr;
    contr.rxData = make_shared<CtrlRecvData>(args.isSim);
    contr.txData = make_shared<CtrlSendData>(args.isSim);
    contr.cmdManager = make_shared<CmdManager>();
    contr.staEstimator = make_shared<StateEstimatorContainer>();
    contr.staEstimator->Init();
    contr.model = qrArgs.algoModel;

    CreateState(contr, jpData_);

    task_ = make_unique<PeriodicMemberFunction<StateManager>>("control", 0.002, this, &StateManager::Run, true);
    GetRemoteParam(); // TODO：获取调试参数
    SetQrEventCallback(QrEventType::fall, [this](QrEventType type)
                       { this->RxQrEvent(type); });
    SetQrEventCallback(QrEventType::qd_to_big, [this](QrEventType type)
                       { this->RxQrEvent(type); });
    SetQrEventCallback(QrEventType::unstable, [this](QrEventType type)
                       { this->RxQrEvent(type); });

    rpcRxDeal_.emplace_back("qr::SetRunState", [this](MsgType set)
                            { return this->RxSetRunState(set); });
    for (auto &rpc : rpcRxDeal_)
    {
        rpc.Connect();
    }
}

StateManager::~StateManager()
{
    for (auto &rpc : rpcRxDeal_)
    {
        rpc.DisConnect();
    }
    Stop();
}

void StateManager::CreateState(ContrContainer contr, std::shared_ptr<JpData> jpData)
{
    staMap_[AlgoState::lie] = make_unique<FsmLie>(contr, jpData);
    staMap_[AlgoState::lie2] = make_unique<FsmLie2>(contr, jpData);
    // staMap_[AlgoState::slowDown] = make_unique<FsmSlowDown>(contr, jpData);
    staMap_[AlgoState::stand] = make_unique<FsmStand>(contr, jpData);
    staMap_[AlgoState::stand2] = make_unique<FsmStand2>(contr, jpData);
    staMap_[AlgoState::walk] = make_unique<FsmWalk>(contr, jpData);
    staMap_[AlgoState::recover] = make_unique<FsmRecover>(contr, jpData);
    staMap_[AlgoState::fall] = make_unique<FsmFall>(contr, jpData);
    staMap_[AlgoState::handStand] = make_unique<FsmHandStand>(contr, jpData);
    staMap_[AlgoState::walk4] = make_unique<FsmWalk4>(contr, jpData);
    staMap_[AlgoState::walk3] = make_unique<FsmWalk3>(contr, jpData);
    staMap_[AlgoState::upright] = make_unique<FsmUpright>(contr, jpData);
    staMap_[AlgoState::upright2walk] = make_unique<FsmUpright2Walk>(contr, jpData);
    staMap_[AlgoState::walk5] = make_unique<FsmWalk2Upright>(contr, jpData);

    Register(AlgoState::lie, AlgoState::stand, AlgoState::stand2);
    Register(AlgoState::stand, AlgoState::walk);
    Register(AlgoState::walk, AlgoState::stand);
    Register(AlgoState::stand, AlgoState::lie, AlgoState::lie2);
    Register(AlgoState::walk, AlgoState::fall);
    Register(AlgoState::fall, AlgoState::stand, AlgoState::recover);
    Register(AlgoState::stand, AlgoState::fall);
    Register(AlgoState::walk, AlgoState::handStand, AlgoState::walk3);
    Register(AlgoState::handStand, AlgoState::walk, AlgoState::walk4);
    Register(AlgoState::walk, AlgoState::upright, AlgoState::walk5);
    Register(AlgoState::upright, AlgoState::walk, AlgoState::upright2walk);
    Register(AlgoState::handStand, AlgoState::fall);
    Register(AlgoState::upright, AlgoState::fall);

#if 0
    Register(AlgoState::lie, AlgoState::walk, AlgoState::stand2);
    Register(AlgoState::stand, AlgoState::walk);
    Register(AlgoState::walk, AlgoState::stand);
    Register(AlgoState::stand, AlgoState::lie, AlgoState::lie2);
    Register(AlgoState::walk, AlgoState::lie, AlgoState::lie2);
    Register(AlgoState::walk, AlgoState::fall);
    Register(AlgoState::fall, AlgoState::walk, AlgoState::recover);
    Register(AlgoState::stand, AlgoState::fall);
#endif
    // 初始化
    staMap_[AlgoState::lie]->OnEnter();
}

bool StateManager::Register(AlgoState from, AlgoState to, AlgoState trans)
{
    for (auto var : canSwitch_)
    {
        // 已经被注册过，直接退出
        if ((var.from == from) && (var.to == to))
        {
            LOG_ERROR("too much 1");
            return false;
        }

        if ((var.from == trans) && (var.to == to))
        {
            LOG_ERROR("too much 2");
            return false;
        }
    }

    canSwitch_.emplace_back(from, to, trans);
    if (trans != AlgoState::null)
    {
        canSwitch_.emplace_back(trans, to, AlgoState::null);
    }

    return true;
}
void StateManager::Start() { task_->Start(); }
void StateManager::Stop() { task_->Stop(); }
void StateManager::Run()
{
    // 安全检查：系统错误状态时立即切换到 lie 状态
    if (GetRobotCurState() == RobotState::error)
    {
        static bool errorReported = false;
        if (!errorReported)
        {
            LOG_ERROR("SAFETY: Robot in error state, forcing transition to lie state");
            errorReported = true;
        }

        // 如果不在 lie 状态，强制切换到 lie
        if (curState_ != AlgoState::lie)
        {
            // 清空执行队列，强制进入 lie 状态
            while (!execList_.empty())
            {
                execList_.pop();
            }
            queryForce_ = RunState::lie;
        }

        // 即使在 lie 状态，也不允许切换到其他状态
        query_ = AlgoState::null;
        queryForce_ = RunState::null;
    }

    if (queryForce_ != RunState::null)
    {
        query_ = queryForce_;
        queryForce_ = RunState::null;
        SwitchNext();
    }
    else if (staMap_[curState_]->IsDone() == true)
    { // 判断当前状态是否执行完成，
        // cout << "!!!!" << endl;
        SwitchNext();
    }

    staMap_[curState_]->PrevRun();
    staMap_[curState_]->Run();
    staMap_[curState_]->AfterRun();

    GetNextState();
}

void StateManager::SwitchNext()
{
    if (execList_.empty() == true)
    {
        if ((query_ != AlgoState::null) && (query_ != curState_))
        {
            for (auto var : canSwitch_)
            {
                if ((var.from == curState_) && (var.to == query_))
                {
                    if (var.trans != AlgoState::null)
                    {
                        execList_.push(var.trans);
                    }
                    execList_.push(var.to);
                    break;
                }
            }
        }

        query_ = AlgoState::null;
    }
    else
    {
        staMap_[curState_]->OnExit();
        curState_ = execList_.front();
        execList_.pop();

        staMap_[curState_]->OnEnter();
    }
}

void StateManager::GetNextState()
{
    for (auto &rpc : rpcRxDeal_)
    {
        rpc.Run();
    }
}

MsgType StateManager::RxSetRunState(const MsgType &in)
{
    auto set = in.GetType<AlgoState>();
    query_ = set;
    RetState resp = RetState::noSupport;

    // 添加日志：RPC 消息已到达
    LOG_INFO("[StateManager] RPC received: SetRunState to state={}, current={}",
             static_cast<int>(set), static_cast<int>(curState_));

    if (set == curState_)
    {
        LOG_INFO("[StateManager] Already in target state, returning ok");
        return RetState::ok;
    }

    if (set == AlgoState::fall)
    {
        LOG_INFO("[StateManager] Requesting fall state, setting queryForce (FORCE mode, bypass health check)");
        queryForce_ = RunState::fall;
        return RetState::ok;
    }

    // 🔥 检查1: 检查状态机转换规则
    bool transitionAllowed = false;
    for (auto var : canSwitch_)
    {
        if ((var.from == curState_) && (var.to == query_))
        {
            transitionAllowed = true;
            break;
        }
    }

    if (!transitionAllowed)
    {
        LOG_WARN("[StateManager] State transition NOT allowed by state machine: {} -> {}",
                 static_cast<int>(curState_), static_cast<int>(set));
        resp = RetState::noSupport;
        LOG_INFO("[StateManager] RxSetRunState returns:{}", static_cast<int>(resp));
        return resp;
    }

    LOG_INFO("[StateManager] State machine allows transition: {} -> {}",
             static_cast<int>(curState_), static_cast<int>(query_));

    // 🔥 检查2: 检查机器人全局状态（新增）
    if (!CheckRobotStateOk())
    {
        LOG_ERROR("[StateManager] Robot state check FAILED, refusing transition");
        resp = RetState::error;
        LOG_INFO("[StateManager] RxSetRunState returns:{}", static_cast<int>(resp));
        return resp;
    }

    // 🔥 检查3: 检查电机健康状态（新增）
    if (!CheckMotorsHealthy())
    {
        LOG_ERROR("[StateManager] Motor health check FAILED, refusing transition");
        resp = RetState::error;
        LOG_INFO("[StateManager] RxSetRunState returns: {}", static_cast<int>(resp));
        return resp;
    }

    // 所有检查通过
    LOG_INFO("[StateManager] ✅ All checks passed, accepting state transition request");
    resp = RetState::ok;
    LOG_INFO("[StateManager] RxSetRunState returns: {}", static_cast<int>(resp));
    return resp;
}

void StateManager::RxQrEvent(QrEventType type)
{
    switch (type)
    {
    case QrEventType::fall:
        queryForce_ = RunState::fall;
        break;
    case QrEventType::unstable:
        jpData_->desire.holdflag.UpdateTrueTime();
        jpData_->desire.holdflag.Set(false);
        queryForce_ = RunState::walk;
        break;
    default:
        queryForce_ = RunState::fall;
        break;
    }
}

bool StateManager::CheckRobotStateOk()
{
    auto currentState = GetRobotCurState();

    if (currentState == RobotState::error)
    {
        LOG_ERROR("[StateManager] Robot in ERROR state, cannot switch state");
        return false;
    }

    if (currentState == RobotState::emgstop)
    {
        LOG_ERROR("[StateManager] Robot in EMERGENCY STOP state, cannot switch state");
        return false;
    }

    return true;
}

bool StateManager::CheckMotorsHealthy()
{
    // 1. 尝试接收电机数据
    auto motorRet = MsgTryRecv<msg::qr::motor_ret>("qr::motor_ret", this);
    if (!motorRet)
    {
        LOG_ERROR("[StateManager] Cannot get motor data, refusing state transition");
        return false;
    }

    // 2. 检查每个电机的状态
    int errorCount = 0;
    int timeoutCount = 0;

    for (int leg = 0; leg < 4; ++leg)
    {
        for (int joint = 0; joint < 3; ++joint)
        {
            auto state = static_cast<MotorState>(motorRet->leg[leg][joint].sta);

            if (state == MotorState::timeout)
            {
                timeoutCount++;
                LOG_WARN("[StateManager] Motor{}{} is TIMEOUT", leg, joint);
            }
            else if (state == MotorState::error)
            {
                errorCount++;
                LOG_WARN("[StateManager] Motor{}{}} is ERROR", leg, joint);
            }
        }
    }

    // 3. 判断是否健康（发现任何超时或错误就拒绝）
    if (timeoutCount > 0 || errorCount > 0)
    {
        LOG_ERROR("[StateManager] Motors not healthy:{} timeout,{} error (total 12)",
                  timeoutCount, errorCount);
        return false;
    }

    return true;
}