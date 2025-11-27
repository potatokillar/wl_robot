
#include "quadCmd.hpp"

#include "offlineVoiceModule.hpp"
#include "robotState.hpp"

QuadCmd::QuadCmd(std::shared_ptr<ApiQuadruped> ctrl, std::shared_ptr<ApiDevice> devApi) : api_(ctrl), devApi_(devApi) {}

void QuadCmd::Run()
{
    voiceNode::voiceMessage rbuf;

    auto ret = MsgTryRecv<voiceNode::voiceMessage>("bs::OfflineVoice", this);
    if (ret) {
        rbuf = ret.value();
        switch (rbuf) {
            case voiceNode::voiceMessage::CmdNameCall:
                // 关键字“小黑小黑”，即离线语音模块收到有人喊机器人名字
                LOG_INFO("RxCmdNameCall ");
                break;
            case voiceNode::voiceMessage::CmdStandUp:
                // 起立，站立
                LOG_INFO("RxCmdStandUp ");
                api_->SetRunState(AlgoState::stand);
                api_->SetRunState(AlgoState::walk);
                api_->SetLinearVelocityRatio(0, 0, 0);
                api_->SetAngularVelocityRatio(0, 0, 0);
                api_->SetHoldFlag(true);
                break;
            case voiceNode::voiceMessage::CmdLieDown:
                // 趴下
                LOG_INFO("RxCmdLieDown ");
                api_->SetRunState(AlgoState::stand);
                api_->SetRunState(AlgoState::lie);
                api_->SetLinearVelocityRatio(0, 0, 0);
                api_->SetAngularVelocityRatio(0, 0, 0);
                break;
            case voiceNode::voiceMessage::CmdMarchInPlace:
                // 原地踏步
                LOG_INFO("RxCmdMarchInPlace ");
                api_->SetRunState(AlgoState::walk);
                api_->SetHoldFlag(false);
                api_->SetLinearVelocityRatio(0, 0, 0);
                api_->SetAngularVelocityRatio(0, 0, 0);
                break;
            case voiceNode::voiceMessage::CmdDance:
                // 跳舞
                LOG_INFO("RxCmdDance ");
                break;
            case voiceNode::voiceMessage::CmdTurnRight:
                // 右转，小黑向右转
                LOG_INFO("RxCmdTurnRight ");
                api_->SetRunState(AlgoState::walk);
                api_->SetAngularVelocityRatio(0, 0, -0.4);
                api_->SetLinearVelocityRatio(0.1, 0, 0);
                api_->SetHoldFlag(true);
                break;
            case voiceNode::voiceMessage::CmdTurnLeft:
                // 左转，小黑向左转
                LOG_INFO("RxCmdTurnLeft ");
                api_->SetRunState(AlgoState::walk);
                api_->SetAngularVelocityRatio(0, 0, 0.4);
                api_->SetLinearVelocityRatio(0.1, 0, 0);
                api_->SetHoldFlag(true);
                break;
            case voiceNode::voiceMessage::CmdMoveBackward:
                // 倒退，小黑向后走
                LOG_INFO("RxCmdMoveBackward ");
                api_->SetRunState(AlgoState::walk);
                api_->SetLinearVelocityRatio(-0.2, 0, 0);
                api_->SetAngularVelocityRatio(0, 0, 0);
                api_->SetHoldFlag(true);
                break;
            case voiceNode::voiceMessage::CmdMoveForward:
                // 前进、行走、小黑向前走
                api_->SetRunState(AlgoState::walk);
                api_->SetLinearVelocityRatio(0.6, 0, 0);
                api_->SetAngularVelocityRatio(0, 0, 0);
                api_->SetHoldFlag(true);
                LOG_INFO("RxCmdMoveForward ");
                break;
            case voiceNode::voiceMessage::CmdMoveLeft:
                // 小黑向左平移
                LOG_INFO("RxCmdMoveLeft ");
                api_->SetRunState(AlgoState::walk);
                api_->SetLinearVelocityRatio(0, 0.4, 0);
                api_->SetAngularVelocityRatio(0, 0, 0);
                api_->SetHoldFlag(true);
                break;
            case voiceNode::voiceMessage::CmdMoveRight:
                // 小黑向右平移
                LOG_INFO("RxCmdMoveRight");
                api_->SetRunState(AlgoState::walk);
                api_->SetLinearVelocityRatio(0, -0.4, 0);
                api_->SetAngularVelocityRatio(0, 0, 0);
                api_->SetHoldFlag(true);
                break;
            case voiceNode::voiceMessage::CmdStandTaller:
                // 站高一点
                LOG_INFO("RxCmdStandTaller ");
                api_->ModifyHeightRatio(0.2);
                api_->SetLinearVelocityRatio(0, 0, 0);
                api_->SetAngularVelocityRatio(0, 0, 0);
                api_->SetHoldFlag(true);
                break;
            case voiceNode::voiceMessage::CmdCrouchLower:
                // 趴低一点
                LOG_INFO("RxCmdCrouchLower ");
                api_->ModifyHeightRatio(-0.2);
                api_->SetLinearVelocityRatio(0, 0, 0);
                api_->SetAngularVelocityRatio(0, 0, 0);
                api_->SetHoldFlag(true);
                break;
            case voiceNode::voiceMessage::CmdStop:
                // 停止|停下|停
                LOG_INFO("RxCmdStop ");
                api_->SetLinearVelocityRatio(0, 0, 0);
                api_->SetAngularVelocityRatio(0, 0, 0);
                api_->SetHoldFlag(true);
                break;
            case voiceNode::voiceMessage::Shutdown:
                // 关机
                LOG_INFO("RxShutdown ");
                devApi_->Shutdown();
                break;
            default:
                LOG_INFO("RxNull");
                break;
        }
        SoftTimerSet[0].AddFunc(
            "audioStop" + std::to_string(TimerTools::GetNowTickMs()),
            3000,
            [this]() {
                this->api_->SetLinearVelocityRatio(0, 0, 0);
                this->api_->SetAngularVelocityRatio(0, 0, 0);
                this->api_->SetHoldFlag(true);
            },
            false);
    }

    // api_->SetRunState
}