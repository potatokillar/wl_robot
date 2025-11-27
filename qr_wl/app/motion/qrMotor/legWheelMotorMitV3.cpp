#include "legWheelMotorMitV3.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

#include "baseline.hpp"
#include "deviceCustomParam.hpp"
#include "robotState.hpp"

using namespace std;

LegWheelMotorMitV3::LegWheelMotorMitV3(const LegWheelMotorCfg &cfg, std::unique_ptr<SpiToCanV3> can) : MotorMitV3(4, 4, std::move(can))
{
    vector<string> legName{"right_front", "left_front", "right_back", "left_back"};
    vector<string> motorName{"abad", "hip", "wheel", "knee"};

    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            infos_[leg][motor].config = cfg.motorCfg[leg][motor];
            infos_[leg][motor].trans = make_unique<TransformMitMotor>(infos_[leg][motor].config.model);
            infos_[leg][motor].name = legName[leg] + "_" + motorName[motor];
        }
    }
}

void LegWheelMotorMitV3::Start()
{
    Enable();
    task_ = make_unique<PeriodicMemberFunction<LegWheelMotorMitV3>>("wheelMotor", 0.01, this, &LegWheelMotorMitV3::Run, true);
    task_->Start();
}

void LegWheelMotorMitV3::Stop()
{
    task_->Stop();
    Disable();
}

void LegWheelMotorMitV3::Run()
{
    if ((GetRobotCurState() == RobotState::running) || (GetRobotCurState() == RobotState::jointCtrl)) {
        SendRecvDeal();
    } else {
        msg::wheel::motor_cmd motorCmd;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                motorCmd.leg[i][j].k1 = 3;
            }
        }

        BlockSend(motorCmd);
    }

    MsgTrySend("wheel::motor_ret", BlockRecv());
}

bool LegWheelMotorMitV3::BlockSend(msg::wheel::motor_cmd cmd)
{
    for (auto leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            if (infos_[leg][motor].config.chiral == false) {
                cmd.leg[leg][motor].alpha *= -1;
                cmd.leg[leg][motor].torq *= -1;
                cmd.leg[leg][motor].blta *= -1;
            }
        }
    }

    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            cmd.leg[leg][motor].alpha += infos_[leg][motor].qBias;
        }
    }

    Transfer(cmd);
    return true;
}

msg::wheel::motor_ret LegWheelMotorMitV3 ::BlockRecv()
{
    msg::wheel::motor_ret data = motorRawPrevData_;

    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            data.leg[leg][motor].alpha -= infos_[leg][motor].qBias;
        }
    }

    for (auto leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            if (infos_[leg][motor].config.chiral == false) {
                data.leg[leg][motor].alpha *= -1;
                data.leg[leg][motor].torq *= -1;
                data.leg[leg][motor].blta *= -1;
            }
        }
    }

    return data;
}

void LegWheelMotorMitV3::Enable()
{
    msg::wheel::motor_ret motorData;
    msg::wheel::motor_cmd motorCmd;
    ClearSpiData();

    SendEnable();
    TimerTools::SleepForMs(2);

    for (int i = 0; i < 5; i++) {
        BlockSend(motorCmd);
        TimerTools::SleepForMs(2);
    }

    vector<vector<double>> nowRet;
    nowRet.resize(LEG_NUM);
    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            nowRet[leg].push_back(motorRawPrevData_.leg[leg][motor].alpha);
        }
    }
    CheckInit("legWheelMotorInitPos", nowRet);

    auto firstData = BlockRecv();

    LOG_INFO("----------------motor init angle--------------------");
    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            LOG_INFO("motor:{}, angle:{:.3f}", infos_[leg][motor].name, firstData.leg[leg][motor].alpha);

            infos_[leg][motor].state = MotorState::enable;
        }
        LOG_INFO("---------------------------------------------------");
    }
}

void LegWheelMotorMitV3::Disable()
{
    msg::wheel::motor_cmd motorCmd;
    BlockSend(motorCmd);

    SendDisable();
    TimerTools::SleepForMs(2);
    SendDisable();

    ClearSpiData();

    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            if (infos_[leg][motor].state == MotorState::enable) {
                infos_[leg][motor].state = MotorState::disable;
            }
        }
    }
}

void LegWheelMotorMitV3::SendRecvDeal()
{
    auto cmd = MsgTryRecv<msg::wheel::motor_cmd>("wheel::motor_cmd", this);
    if (cmd) {
        BlockSend(cmd.value());
    }
}

void LegWheelMotorMitV3::Transfer(const msg::wheel::motor_cmd &cmd)
{
    array<std::vector<CanData>, 4> canMsg;
    for (int motor = 0; motor < MOTOR_NUM; motor++) {
        canMsg[0].push_back(infos_[0][motor].trans->CmdToCanV3(cmd.leg[0][motor], motor + 1));
        canMsg[1].push_back(infos_[1][motor].trans->CmdToCanV3(cmd.leg[1][motor], motor + 1));
        canMsg[2].push_back(infos_[2][motor].trans->CmdToCanV3(cmd.leg[2][motor], motor + 1));
        canMsg[3].push_back(infos_[3][motor].trans->CmdToCanV3(cmd.leg[3][motor], motor + 1));
    }

    SpiTransfer(canMsg);

    std::array<std::vector<CanData>, 4> msg;
    msg[0] = spi2can->GetMessage(0, 0);
    msg[1] = spi2can->GetMessage(0, 1);
    msg[2] = spi2can->GetMessage(1, 0);
    msg[3] = spi2can->GetMessage(1, 1);

    for (int i = 0; i < 4; i++) {
        if (msg[i].size() != (size_t)MOTOR_NUM) {
            return;
        }
    }

    vector<vector<bool>> checks;
    for (int leg = 0; leg < LEG_NUM; leg++) {
        vector<bool> check;
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            auto ret = infos_[leg][motor].trans->CanToDataV3(msg[leg][motor], &motorRawPrevData_.leg[leg][motor]);
            check.push_back(ret);
        }
        checks.push_back(check);
    }

    CheckMotorState(checks);
}
