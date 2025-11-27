
#include "keyQuadruped.hpp"

#include "deviceCustomParam.hpp"
#include "deviceParam.hpp"
#include "robotMedia.hpp"
#include "robotState.hpp"

using namespace std;

KeyQuadruped::KeyQuadruped(std::shared_ptr<ApiQuadruped> api) : api_(api)
{
    thread_ = std::thread(&KeyQuadruped::BlockRun, this);

    // 使用AddFunc()注册按键的短按、长按动作
    // AddFunc("a", KeyMode::Short, [] { LOG_DEBUG("A short press"); });
    // AddFunc("a", KeyMode::Long, [] { LOG_DEBUG("A long press"); });
    // AddFunc("b", KeyMode::Short, [] { LOG_DEBUG("B short press"); });
    // AddFunc("b", KeyMode::Long, [] { LOG_DEBUG("B long press"); });
    // AddFunc("up", KeyMode::Short, [] { LOG_DEBUG("up short press"); });
    // AddFunc("up", KeyMode::Long, [] { LOG_DEBUG("up long press"); });

    // 使用AddMultiFunc()添加多键动作
    // AddMultiFunc("a", "lb", [] { LOG_DEBUG("A and LB multi press"); });
    // AddMultiFunc("a", "left", [] { LOG_DEBUG("A and left multi press"); });
}

/**
 * @description: 阻塞运行处理，自行通过线程处理
 * @return {}
 */
void KeyQuadruped::BlockRun()
{
    while (true) {
        if (UpdateGamepadKey()) {
            if (IsShortRelease("right")) {
                LOG_DEBUG("RIGHT short press");
                api_->SetDance(tmpDance_);
            }
        }

        TimerTools::SleepForMs(50);
    }
}

/**
 * @description: 非阻塞型调用，由keyNode调用，若是阻塞调用自行开辟线程
 * @return {}
 */
void KeyQuadruped::Run()
{
    if (UpdateGamepadKey()) {
        RunMultiPress();
        QuadKey();
    }
}

/**
 * @description: 回调式接口，检测以及执行所有按键的长按动作
 * @return {}
 */
void KeyQuadruped::RunLongPress()
{
    for (auto item : func_) {
        LongPressingCallback(item.first);  // 执行长按逻辑，带回调函数
    }
}

/**
 * @description: 回调式接口，更新所有按键的多键状态，判断是否构成多键
 * 如构成多键，调整单个按键的标志位，并将结果存放在multi_act中
 * @return {}
 */
void KeyQuadruped::RunMultiPress() { MultiEnable(); }

/**
 * @description: 回调式接口，处理所有按键发生的抬起动作
 * @return {}
 */
void KeyQuadruped::ReleaseKey()
{
    for (auto item : func_) {
        if (btn_.at(item.first).IsTriggerRelease()) {
            RunRelease(item.first);
        }
    }
}

// /**
//  * @description: 四足手柄按键映射
//  * @param cmd
//  * @return {}
//  */
// void KeyQuadruped::QuadKey()
// {
//     if (btn_.at("start").IsTriggerPress()) {
//         api_->Start(QuadBootMode::qr);
//     } else if (btn_.at("back").IsTriggerPress()) {
//         api_->Stop();
//     }

//     if (btn_.at("x").IsTriggerPress()) {
//         api_->SetRunState(RunState::lie);
//     } else if (btn_.at("a").IsTriggerPress()) {
//         api_->SetRunState(RunState::stand);
//     } else if (btn_.at("b").IsTriggerPress()) {
//         api_->SetRunState(RunState::walk);
//     } else if (stick_.at("lt") > 0.8) {
//         api_->SetRunState(RunState::fall);
//     }

//     // pose  or velocity based on state and mode
//     RunState state = api_->GetRunState();
//     if (state == RunState::lie) {
//         if (btn_["up"].IsTriggerPress()) {
//             GetDevCustomParam().WriteInitFlag(1);
//             PlayMedia("paramWrite");  // 2024.10.23 - PlayMedia()接口已在当前commit中注释掉了
//         }
//     }

//     if (state == RunState::stand) {
//         double rollRatio = 0;
//         if (stick_.at("lx") < -0.2) {
//             rollRatio = -1;
//         } else if (stick_.at("lx") > 0.2) {
//             rollRatio = 1;
//         } else {
//             rollRatio = 0;
//         }
//         api_->SetPoseRatio(rollRatio, -stick_.at("ry"), -stick_.at("rx"));
//         // height
//         api_->ModifyHeightRatio(-stick_.at("ly"));
//     } else if (state == RunState::walk) {
//         bool holdFlag = api_->GetHoldFlag();
//         // 组合键，两个按键按下时，才置为按下。
//         // LB_RB_.SetValue(cmd.btnLb && cmd.btnRb);
//         if (btn_.at("lb_rb").IsTriggerPress() == true) {
//             poseControlFlag_ = !poseControlFlag_;
//         }

//         if (poseControlFlag_ && holdFlag) {
//             api_->SetPoseRatio(-stick_.at("lx"), -stick_.at("ry"), -stick_.at("rx"));
//             api_->ModifyHeightRatio(-stick_.at("ly"));  // todo
//         } else {
//             api_->SetLinearVelocityRatio(-stick_.at("ly"), -stick_.at("lx"), 0);
//             api_->SetAngularVelocityRatio(0, 0, -stick_.at("rx"));
//             api_->SetPoseRatio(0.0, -stick_.at("ry"), 0.0);
//             if (btn_.at("down").IsTriggerPress()) {
//                 api_->ModifyHeight(-0.01);
//             } else if (btn_.at("up").IsTriggerPress()) {
//                 api_->ModifyHeight(0.01);
//             }
//         }
//     }

//     // walkmode
//     if (btn_["b_rb"].IsTriggerPress() == true) {
//         api_->ModifyWalkMode();
//     }

//     // holdFlag
//     if (btn_["y"].IsTriggerPress() == true) {
//         api_->ModifyHoldFlag();
//     }

//     if (btn_.at("rt_lb").IsTriggerPress()) {
//         auto loadmass = api_->GetLoadMass();
//         auto maxValue = api_->GetLoadMassRange().max;
//         if (isLoadmassAdd_) {
//             if (api_->SetLoadMass(loadmass + maxValue * 0.05) == RetState::outRange) {
//                 isLoadmassAdd_ = false;
//             }

//         } else {
//             if (api_->SetLoadMass(loadmass - maxValue * 0.05) == RetState::outRange) {
//                 isLoadmassAdd_ = true;
//             }
//         }
//     }

//     if (btn_["left"].IsTriggerPress()) {
//         tmpDance_++;
//         if (tmpDance_ > 4) {
//             tmpDance_ = 0;
//         }
//         LOG_INFO("dance idx:{}", tmpDance_);
//     }
// }

/**
 * @description: 应用新的Is式接口，复现了原有的按键映射QuadKey()
 * @return {}
 */
void KeyQuadruped::QuadKey()
{
    // 安全检查：系统错误状态时禁用所有手柄控制
    if (GetRobotCurState() == RobotState::error) {
        static bool errorReported = false;
        if (!errorReported) {
            LOG_ERROR("SAFETY: Gamepad control disabled - robot in error state");
            errorReported = true;
        }
        return;  // 阻止任何手柄操作
    }

    if (IsShortRelease("start")) {
        api_->Start(QuadBootMode::qr);
        LOG_DEBUG("START short press");
    } else if (IsShortRelease("back")) {
        api_->SetRunState(RunState::fall);
        LOG_DEBUG("BACK short press");
    }

    if (IsShortRelease("x")) {
        api_->SetRunState(RunState::lie);
        LOG_DEBUG("X short press");
    } else if (IsShortRelease("a")) {
        api_->SetRunState(RunState::stand);
        LOG_DEBUG("A short press");
    } else if (IsShortRelease("b")) {
        api_->SetRunState(RunState::walk);
        LOG_DEBUG("B short press");
    }

    // pose or velocity based on state and mode
    RunState state = api_->GetRunState();

    // 站立状态下
    if (state == RunState::stand) {
        double rollRatio = 0;
        if (stick_.at("lx") < -0.2) {
            rollRatio = -1;
        } else if (stick_.at("lx") > 0.2) {
            rollRatio = 1;
        } else {
            rollRatio = 0;
        }
        api_->SetPoseRatio(rollRatio, -stick_.at("ry"), -stick_.at("rx"));
        api_->ModifyHeightRatio(-stick_.at("ly"));

        if (IsMultiKey("lb", "rb")) {  // 负载增加1个档位
            LOG_DEBUG("LB and RB, multi key press");
            auto loadmass = api_->GetLoadMass();
            auto maxValue = api_->GetLoadMassRange().max;
            if (!load_max_reached) {
                if (api_->SetLoadMass(loadmass + maxValue * 0.05) == RetState::outRange) {
                    load_max_reached = true;
                } else {
                    load_min_reached = false;
                }
            }
        } else if (IsMultiKey("lb", "rt")) {  // 负载减少1个档位
            LOG_DEBUG("LB and RT, multi key press");
            auto loadmass = api_->GetLoadMass();
            auto maxValue = api_->GetLoadMassRange().max;
            if (!load_min_reached) {
                if (api_->SetLoadMass(loadmass - maxValue * 0.05) == RetState::outRange) {
                    load_min_reached = true;
                } else {
                    load_max_reached = false;
                }
            }
        }

        if (IsShortRelease("down")) {  // 表演动作index向下翻页1
            tmpDance_++;
            if (tmpDance_ > 6) {
                tmpDance_ = 0;
            }
            LOG_INFO("dance idx:{}", tmpDance_);
            LOG_DEBUG("UP short press");
        } else if (IsShortRelease("up")) {  // 表演动作index向上翻页1
            tmpDance_--;
            if (tmpDance_ < 0) {
                tmpDance_ = 6;
            }
            LOG_INFO("dance idx:{}", tmpDance_);
            LOG_DEBUG("DOWN short press");
        }
    }
    // 行走状态下
    else if (state == RunState::walk || state == RunState::handStand || state == RunState::upright) {
        bool holdFlag = api_->GetHoldFlag();
        if (IsMultiKey("rb", "lb")) {
            LOG_DEBUG("RB and LB, multi key press");
            poseControlFlag_ = !poseControlFlag_;
        }

        if (poseControlFlag_ && holdFlag) {
            api_->SetPoseRatio(-stick_.at("lx"), -stick_.at("ry"), -stick_.at("rx"));
            api_->ModifyHeightRatio(-stick_.at("ly"));  // todo
        } else {
            api_->SetLinearVelocityRatio(-stick_.at("ly"), -stick_.at("lx"), 0);
            api_->SetAngularVelocityRatio(0, 0, -stick_.at("rx"));
            api_->SetPoseRatio(0.0, -stick_.at("ry"), 0.0);
            if (IsShortRelease("down")) {
                api_->ModifyHeight(-0.01);
                LOG_DEBUG("DOWN short press");
            } else if (IsShortRelease("up")) {
                api_->ModifyHeight(0.01);
                LOG_DEBUG("UP short press");
            }
        }

        // 切换行走模式
        if (IsMultiKey("b", "rb")) {
            LOG_DEBUG("B and RB, multi key press");
            api_->ModifyWalkMode();
        } else if (IsMultiKey("b", "rt")) {
            LOG_DEBUG("B and RT, multi key press");
            api_->ModifyWalkModeReverse();
        }

        if (IsMultiKey("b", "lb")) {
            LOG_DEBUG("B and LB, multi key press");
            api_->SetRunState(RunState::handStand);
        } else if (IsMultiKey("b", "lt")) {
            LOG_DEBUG("B and LT, multi key press");
            api_->SetRunState(RunState::upright);
        }

        // holdFlag，原地踏步功能，仅在传统运控状态下起作用
        if (IsShortRelease("y")) {
            api_->ModifyHoldFlag();
            LOG_DEBUG("Y short press");
        }
    }
    // 趴着状态下
    else if (state == RunState::lie) {
        if (IsMultiKey("lt", "rt")) {  // 初始化关节角，以当前摆放位置为零位
            GetDevCustomParam().WriteInitFlag(1);
            AddMediaPackage(MediaTaskPackage("paramWrite"));
            LOG_DEBUG("LT and RT, multi key press");
        }
    }
};