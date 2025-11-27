#include "qrMotorMitV3.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

#include "baseline.hpp"
#include "deviceCustomParam.hpp"
#include "robotState.hpp"

using namespace std;

QrMotorMitV3::QrMotorMitV3(const QrMotorCfgV3 &cfg, std::unique_ptr<SpiToCanV3> can) : MotorMitV3(4, 3, std::move(can))
{
    vector<string> legName{
        "left_front",
        "right_front",
        "right_back",
        "left_back",
    };
    vector<string> motorName{"abad", "hip", "knee"};

    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            infos_[leg][motor].config = cfg.motorCfg[leg][motor];
            infos_[leg][motor].trans = make_unique<TransformMitMotor>(infos_[leg][motor].config.model);
            infos_[leg][motor].name = legName[leg] + "_" + motorName[motor];
        }
    }
}

void QrMotorMitV3::Start()
{
    Enable();
    // todo 改成10ms
    task_ = make_unique<PeriodicMemberFunction<QrMotorMitV3>>("wheelMotor", 0.002, this, &QrMotorMitV3::Run, true);
    task_->Start();
}

void QrMotorMitV3::Stop()
{
    task_->Stop();
    Disable();
}

void QrMotorMitV3::Run()
{
    if ((GetRobotCurState() == RobotState::running) || (GetRobotCurState() == RobotState::jointCtrl)) {
        SendRecvDeal();
    } else {
        msg::qr::motor_cmd motorCmd;
        for (int i = 0; i < LEG_NUM; i++) {
            for (int j = 0; j < MOTOR_NUM; j++) {
                motorCmd.leg[i][j].k1 = 3;
            }
        }

        BlockSend(motorCmd);
    }

    MsgTrySend("qr::motor_ret", BlockRecv());
}

bool QrMotorMitV3::BlockSend(msg::qr::motor_cmd cmd)
{
    for (auto leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            if (infos_[leg][motor].config.chiral == false) {
                cmd.leg[leg][motor].alpha *= -1;
                cmd.leg[leg][motor].torq *= -1;
                cmd.leg[leg][motor].blta *= -1;
            }
        }
    }

    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            cmd.leg[leg][motor].alpha += infos_[leg][motor].qBias;
        }
    }

    Transfer(cmd);
    return true;
}

msg::qr::motor_ret QrMotorMitV3 ::BlockRecv()
{
    msg::qr::motor_ret data = motorRawPrevData_;

    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            data.leg[leg][motor].alpha -= infos_[leg][motor].qBias;
            data.leg[leg][motor].sta = static_cast<uint8_t>(infos_[leg][motor].state);
        }
    }

    for (auto leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            if (infos_[leg][motor].config.chiral == false) {
                data.leg[leg][motor].alpha *= -1;
                data.leg[leg][motor].torq *= -1;
                data.leg[leg][motor].blta *= -1;
            }
        }
    }

    return data;
}

void QrMotorMitV3::Enable()
{
    LOG_INFO("Starting motor enable process...");
    msg::qr::motor_ret motorData;
    msg::qr::motor_cmd motorCmd;
    ClearSpiData();

    // 步骤1：清空状态，确保从干净状态开始
    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            infos_[leg][motor].lastRecv = 0;
            infos_[leg][motor].state = MotorState::disable;
        }
    }

    // 步骤2：发送使能命令
    SendEnable();
    TimerTools::SleepForMs(20);  // 增加等待时间

    // 步骤3：验证响应（零容错）
    const int MAX_ATTEMPTS = 15;  // 增加尝试次数
    int enabledCount = 0;

    for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
        BlockSend(motorCmd);
        TimerTools::SleepForMs(10);

        // 检查每个电机的响应状态
        enabledCount = 0;
        for (int leg = 0; leg < LEG_NUM; leg++) {
            for (int motor = 0; motor < MOTOR_NUM; motor++) {
                auto& info = infos_[leg][motor];
                auto nowTime = TimerTools::GetNowTickMs();

                // 电机在最近100ms内有响应，认为在线
                if (info.lastRecv > 0 && (nowTime - info.lastRecv) < 100) {
                    if (info.state != MotorState::enable) {
                        info.state = MotorState::enable;
                        LOG_INFO("Motor {} enabled at attempt {}", info.name, attempt + 1);
                    }
                    enabledCount++;
                }
            }
        }

        // 所有电机都已响应，提前退出
        if (enabledCount == LEG_NUM * MOTOR_NUM) {
            LOG_INFO("All {} motors enabled successfully", enabledCount);
            break;
        }
    }

    // 步骤4：失败处理 - 零容错策略
    if (enabledCount < LEG_NUM * MOTOR_NUM) {
        LOG_CRITICAL("SAFETY: Motor enable failed, only {}/{} motors responded", enabledCount, LEG_NUM * MOTOR_NUM);

        // 标记失败的电机
        for (int leg = 0; leg < LEG_NUM; leg++) {
            for (int motor = 0; motor < MOTOR_NUM; motor++) {
                if (infos_[leg][motor].state != MotorState::enable) {
                    infos_[leg][motor].state = MotorState::error;
                    LOG_ERROR("Motor {} FAILED to enable", infos_[leg][motor].name);
                }
            }
        }

        // 设置系统错误状态
        SetRobotCurState(RobotState::error, "Motor enable failed");

        // 发送阻尼命令确保安全
        msg::qr::motor_cmd safeCmd;
        for (int i = 0; i < LEG_NUM; i++) {
            for (int j = 0; j < MOTOR_NUM; j++) {
                safeCmd.leg[i][j].k1 = 5;  // 较高阻尼
            }
        }
        BlockSend(safeCmd);
        return;
    }

    // 步骤5：成功后的初始化（只在所有电机都成功后执行）
    vector<vector<double>> nowRet;
    nowRet.resize(LEG_NUM);
    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            nowRet[leg].push_back(motorRawPrevData_.leg[leg][motor].alpha);
        }
    }
    CheckInit("qrMotorInitPos", nowRet);

    auto firstData = BlockRecv();
    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            LOG_INFO("motor:{}, angle:{:.3f}", infos_[leg][motor].name, firstData.leg[leg][motor].alpha);
        }
        LOG_INFO("---------------------------------------------------");
    }
}

void QrMotorMitV3::Disable()
{
    msg::qr::motor_cmd motorCmd;
    BlockSend(motorCmd);

    SendDisable();
    TimerTools::SleepForMs(2);
    SendDisable();

    ClearSpiData();

    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            if (infos_[leg][motor].state == MotorState::enable) {
                infos_[leg][motor].state = MotorState::disable;
            }
        }
    }
}

void QrMotorMitV3::SendRecvDeal()
{
    // 实时检查电机状态（零容错）
    int timeoutCount = 0;
    int errorCount = 0;

    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            if (infos_[leg][motor].state == MotorState::timeout) {
                timeoutCount++;
            } else if (infos_[leg][motor].state == MotorState::error) {
                errorCount++;
            }
        }
    }

    // 任何电机异常立即处理
    if (timeoutCount > 0 || errorCount > 0) {
        static uint32_t lastWarnTime = 0;
        auto nowTime = TimerTools::GetNowTickMs();

        // 每秒报告一次
        if (nowTime - lastWarnTime > 1000) {
            LOG_ERROR("SAFETY: Motor fault detected - {} timeout, {} error", timeoutCount, errorCount);
            lastWarnTime = nowTime;
        }

        // 立即设置错误状态
        if (GetRobotCurState() == RobotState::running || GetRobotCurState() == RobotState::jointCtrl) {
            SetRobotCurState(RobotState::error, "Motor fault during operation");
        }

        // 发送安全命令（零扭矩+阻尼）
        msg::qr::motor_cmd stopCmd;
        for (int i = 0; i < LEG_NUM; i++) {
            for (int j = 0; j < MOTOR_NUM; j++) {
                stopCmd.leg[i][j].torq = 0;
                stopCmd.leg[i][j].k1 = 5;  // 阻尼
            }
        }
        BlockSend(stopCmd);
        return;  // 不执行正常命令
    }

    // 正常处理
    auto cmd = MsgTryRecv<msg::qr::motor_cmd>("qr::motor_cmd", this);
    if (cmd) {
        BlockSend(cmd.value());
    }
}

void QrMotorMitV3::Transfer(const msg::qr::motor_cmd &cmd)
{
    array<std::vector<CanData>, 4> canMsg;
    for (int motor = 0; motor < MOTOR_NUM; motor++) {
        canMsg[0].push_back(infos_[0][motor].trans->CmdToCanV3(cmd.leg[0][motor], motor + 1));
        canMsg[1].push_back(infos_[1][motor].trans->CmdToCanV3(cmd.leg[1][motor], motor + 1));
        canMsg[2].push_back(infos_[2][motor].trans->CmdToCanV3(cmd.leg[2][motor], motor + 1));
        canMsg[3].push_back(infos_[3][motor].trans->CmdToCanV3(cmd.leg[3][motor], motor + 1));
    }

    SpiTransfer(canMsg);

    std::array<std::vector<CanData>, 4> msg;  // 4路can
    msg[0] = spi2can->GetMessage(0, 0);
    msg[1] = spi2can->GetMessage(0, 1);
    msg[2] = spi2can->GetMessage(1, 0);
    msg[3] = spi2can->GetMessage(1, 1);

    // 检查数据完整性
    bool dataValid = true;
    for (int i = 0; i < 4; i++) {
        if (msg[i].size() != (size_t)MOTOR_NUM) {
            dataValid = false;
            break;
        }
    }

    vector<vector<bool>> checks;
    for (int leg = 0; leg < LEG_NUM; leg++) {
        vector<bool> check;
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            bool ret = false;
            if (dataValid) {
                ret = infos_[leg][motor].trans->CanToDataV3(msg[leg][motor], &motorRawPrevData_.leg[leg][motor]);
            }
            check.push_back(ret);
        }
        checks.push_back(check);
    }

    // 即使数据无效，也要调用CheckMotorState让超时检测工作
    CheckMotorState(checks);
}