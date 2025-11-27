
#include "fsmState.hpp"

#include <cmath>

#include "controller.hpp"
#include "eventMsg.hpp"
#include "orientation_tools.h"

using namespace std;

/**
 * @description:
 * @param sta 状态名
 * @param contr 算法基类
 * @param jpData 数据基类
 * @return {}
 */
FsmState::FsmState(AlgoState sta, ContrContainer contr, std::shared_ptr<JpData> jpData)
    : state_(sta), jpData_(jpData), txData_(contr.txData), rxData_(contr.rxData), staEstimator_(contr.staEstimator), cmd_(contr.cmdManager)
{
    // 初始化的时候，删除旧的内容
    MsgSend msgTx("qr::ControllerData");
    msgTx.Clear();
}

/**
 * @description: 状态前处理，通用处理
 * @return {}
 */
void FsmState::PrevRun()
{
    contrParamHotUpdate.TryGetRequest();

    // 获取外部输入
    rxData_->LoadData();
    // 进行状态估计
    staEstimator_->LoadData(rxData_->GetImuData());
    staEstimator_->LoadData(rxData_->GetData3());
    staEstimator_->LoadData(jpData_->output);
    staEstimator_->Run(state_);
    // 配置一些数据
    txData_->SetCurState(state_);

    // 接收控制指令
    cmd_->Run();
    SetCurData(&jpData_->current);
}

void FsmState::AfterRun()
{
    // PublishBodyDesire(jpData_->desire.rpy, jpData_->desire.w, jpData_->desire.a);
    PublishBodyDesire(jpData_->current.rpy, jpData_->current.w, jpData_->current.a, jpData_->current.quat);

    // 算法模块发布的数据分为两种，一种是给其它大模块的，例如sdk。一种是给状态估计等小模块的。
    // 这里并没有做区分，由对应模块自行选择必要的数据

    ControllerData curData;
    curData.linearV = jpData_->current.v;
    curData.angularV = jpData_->current.w;
    curData.pose = jpData_->current.a;
    curData.state = state_;
    curData.gait = jpData_->current.gait;
    curData.mode = jpData_->current.mode;
    curData.height = jpData_->current.h;
    curData.holdFlag = jpData_->desire.holdflag.Get();
    curData.loadMass = cmd_->GetLoadMass();

    MsgTrySend("qr::ControllerData", curData);

    // jp算法输出的数据
    jpData_->output.aDesire = jpData_->desire.a;
    jpData_->output.contactFlag = jpData_->contactFlag;
    jpData_->output.sPhi = jpData_->sPhi;
    jpData_->output.phi = jpData_->phi;
    jpData_->output.holdFlag = jpData_->desire.holdflag.Get();
    jpData_->output.desireHeight = jpData_->desire.h;

    // 内部状态传出
    if (staEstimator_->GetResult().flag.fallDownInfo != 0) {
        LOG_WARN("robot falling");
        SetQrEvent(QrEventType::fall);
    }
}

void FsmState::SetCurData(CurPara* cur)
{
    cur->a = staEstimator_->GetResult().aCom;
    cur->h = staEstimator_->GetResult().bodyHeight;
    cur->rpy = staEstimator_->GetResult().rpy;
    cur->R = staEstimator_->GetResult().bodyR;  // 身体转世界的R
    // cur->Ryaw0 = ori::rpyToRotMat(Vec3<double>(cur->rpy(0), cur->rpy(1), 0.0));
    cur->v = staEstimator_->GetResult().vCom;  // 身体坐标系下的,在卡尔曼滤波里面计算
    cur->w = staEstimator_->GetResult().wCom;
    cur->quat = staEstimator_->GetResult().quat;
}