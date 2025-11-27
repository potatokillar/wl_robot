#include "legWheelMotorMit.hpp"

#include <chrono>
#include <iostream>
#include <vector>

#include "baseline.hpp"
#include "deviceCustomParam.hpp"
#include "robotState.hpp"

using namespace std;

LegWheelMotorMit::LegWheelMotorMit(const LegWheelMotorCfg &cfg, std::unique_ptr<SpiToCanV2> can) : spi2can(std::move(can))
{
    vector<string> legName{"right_front", "left_front", "right_back", "left_back"};
    vector<string> motorName{"abad", "hip", "knee", "wheel"};

    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            infos_[leg][motor].config = cfg.motorCfg[leg][motor];
            infos_[leg][motor].trans = make_unique<TransformMitMotor>(infos_[leg][motor].config.model);
            infos_[leg][motor].name = legName[leg] + "_" + motorName[motor];
        }
    }
}

void LegWheelMotorMit::ClearAllData() { spi2can->ClearBuf(); }

void LegWheelMotorMit::Start()
{
    Enable();
    task_ = make_unique<PeriodicMemberFunction<LegWheelMotorMit>>("wheelMotor", 0.01, this, &LegWheelMotorMit::Run, true);
    task_->Start();
}

void LegWheelMotorMit::Stop()
{
    Disable();
    task_->Stop();
}

void LegWheelMotorMit::Run()
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

bool LegWheelMotorMit::BlockSend(msg::wheel::motor_cmd cmd)
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

    SPI_TO_CAN_V2 msgBoard[2];

    for (int motor = 0; motor < 4; motor++) {
        infos_[0][motor].trans->CmdToCanV2(msgBoard[0].payload.can0[motor], cmd.leg[0][motor], motor + 1);
        infos_[1][motor].trans->CmdToCanV2(msgBoard[0].payload.can1[motor], cmd.leg[1][motor], motor + 1);
        infos_[2][motor].trans->CmdToCanV2(msgBoard[1].payload.can0[motor], cmd.leg[2][motor], motor + 1);
        infos_[3][motor].trans->CmdToCanV2(msgBoard[1].payload.can1[motor], cmd.leg[3][motor], motor + 1);

        for (int leg = 0; leg < 4; leg++) {
            infos_[leg][motor].sendCnt++;
        }
    }

    SpiTransfer(msgBoard[0], msgBoard[1]);
    return true;
}

msg::wheel::motor_ret LegWheelMotorMit ::BlockRecv()
{
    msg::wheel::motor_ret data = RecvRaw();

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

msg::wheel::motor_ret LegWheelMotorMit::RecvRaw()
{
    SPI_TO_CAN_V2 msgBoard[2];
    msgBoard[0] = spi2can->GetMessage(0);
    msgBoard[1] = spi2can->GetMessage(1);

    bool ret[4];
    for (int motor = 0; motor < 4; motor++) {
        ret[0] = infos_[0][motor].trans->CanToDataV2(msgBoard[0].payload.can0[motor], &motorRawPrevData_.leg[0][motor]);
        ret[1] = infos_[1][motor].trans->CanToDataV2(msgBoard[0].payload.can1[motor], &motorRawPrevData_.leg[1][motor]);
        ret[2] = infos_[2][motor].trans->CanToDataV2(msgBoard[1].payload.can0[motor], &motorRawPrevData_.leg[2][motor]);
        ret[3] = infos_[3][motor].trans->CanToDataV2(msgBoard[1].payload.can1[motor], &motorRawPrevData_.leg[3][motor]);

        for (int j = 0; j < 4; j++) {
            if (ret[j]) {
                infos_[j][motor].lastRecv = TimerTools::GetNowTickMs();
                infos_[j][motor].recvCnt++;
            }
        }
    }

    return motorRawPrevData_;
}

std::optional<std::array<std::array<double, 4>, 4>> LegWheelMotorMit::GetZeroPosFromDatabase() { return GetDevCustomParam().ReadWheelZeroPos(); }

void LegWheelMotorMit::SaveZeroPos2Database(const msg::wheel::motor_ret &data)
{
    auto [ret, init] = GetDevCustomParam().ReadInitFlag();
    if ((ret == false) || (init == 0)) {
        LOG_INFO("SaveZeroPos2Database: init flag is 0 or noexist, exit");
        return;
    }

    bool write = true;

    if (boot_.noMotor == false) {
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 4; motor++) {
                if (data.leg[leg][motor].alpha == 0) {
                    write = false;
                }
            }
        }
    }

    if (write == true) {
        std::array<std::array<double, 4>, 4> qBias;
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 4; motor++) {
                qBias[leg][motor] = data.leg[leg][motor].alpha;
            }
        }

        GetDevCustomParam().WriteWheelZeroPos(qBias);
        GetDevCustomParam().WriteInitFlag(0);
        LOG_INFO("save zeropos to database");
    }
}

void LegWheelMotorMit::CheckInit(const msg::wheel::motor_ret &data)
{
    if (biasIsUpdate_ == true) {
        return;
    }

    SaveZeroPos2Database(data);

    bool success = true;

    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            auto &one_motor_info = infos_[leg][motor];
            auto &one_motor_data = data.leg[leg][motor];
            if (one_motor_data.alpha == 0.0) {
                one_motor_info.qBias = 0.0;
                success = false;
            } else {
                one_motor_info.qBias = one_motor_data.alpha;
            }
        }
    }
    biasIsUpdate_ = success;
}

void LegWheelMotorMit::Enable()
{
    msg::wheel::motor_ret motorData;
    msg::wheel::motor_cmd motorCmd;
    ClearAllData();

    MotorEnableCmd();
    TimerTools::SleepForMs(2);

    for (int i = 0; i < 5; i++) {
        BlockSend(motorCmd);
        RecvRaw();
        TimerTools::SleepForMs(2);
    }

    CheckInit(RecvRaw());

    auto firstData = BlockRecv();
    bool isBigNum = false;
    int isZero = 0;
    LOG_INFO("----------------motor init angle--------------------");
    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            LOG_INFO("motor:{}, angle:{:.3f}", infos_[leg][motor].name, firstData.leg[leg][motor].alpha);

            infos_[leg][motor].state = MotorState::enable;
        }
        LOG_INFO("---------------------------------------------------");
    }
}

void LegWheelMotorMit::Disable()
{
    msg::wheel::motor_cmd motorCmd;
    BlockSend(motorCmd);

    MotorDisableCmd();
    TimerTools::SleepForMs(2);
    MotorDisableCmd();

    ClearAllData();

    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            if (infos_[leg][motor].state == MotorState::enable) {
                infos_[leg][motor].state = MotorState::disable;
            }
        }
    }
}

void LegWheelMotorMit::MotorZeroCmd()
{
    SPI_TO_CAN_V2 msgBoard[2];
    for (int board = 0; board < 2; board++) {
        for (int i = 0; i < 8; i++) {
            msgBoard[board].payload.data[i][9] = 0xFE;
        }

        msgBoard[board].payload.data[0][0] = 0x01;
        msgBoard[board].payload.data[0][1] = 0x00;

        msgBoard[board].payload.data[1][0] = 0x02;
        msgBoard[board].payload.data[1][1] = 0x00;

        msgBoard[board].payload.data[2][0] = 0x03;
        msgBoard[board].payload.data[2][1] = 0x00;

        msgBoard[board].payload.data[3][0] = 0x04;
        msgBoard[board].payload.data[3][1] = 0x00;

        msgBoard[board].payload.data[4][0] = 0x01;
        msgBoard[board].payload.data[4][1] = 0x00;

        msgBoard[board].payload.data[5][0] = 0x02;
        msgBoard[board].payload.data[5][1] = 0x00;

        msgBoard[board].payload.data[6][0] = 0x03;
        msgBoard[board].payload.data[6][1] = 0x00;

        msgBoard[board].payload.data[7][0] = 0x04;
        msgBoard[board].payload.data[7][1] = 0x00;
    }

    SpiTransfer(msgBoard[0], msgBoard[1]);
}

void LegWheelMotorMit::MotorEnableCmd()
{
    SPI_TO_CAN_V2 msgBoard[2];
    // LOG_INFO("Enable motor");
    for (int board = 0; board < 2; board++) {
        for (int i = 0; i < 8; i++) {
            msgBoard[board].payload.data[i][9] = 0xFC;
        }
        msgBoard[board].payload.data[0][0] = 0x01;
        msgBoard[board].payload.data[0][1] = 0x00;

        msgBoard[board].payload.data[1][0] = 0x02;
        msgBoard[board].payload.data[1][1] = 0x00;

        msgBoard[board].payload.data[2][0] = 0x03;
        msgBoard[board].payload.data[2][1] = 0x00;

        msgBoard[board].payload.data[3][0] = 0x04;
        msgBoard[board].payload.data[3][1] = 0x00;

        msgBoard[board].payload.data[4][0] = 0x01;
        msgBoard[board].payload.data[4][1] = 0x00;

        msgBoard[board].payload.data[5][0] = 0x02;
        msgBoard[board].payload.data[5][1] = 0x00;

        msgBoard[board].payload.data[6][0] = 0x03;
        msgBoard[board].payload.data[6][1] = 0x00;

        msgBoard[board].payload.data[7][0] = 0x04;
        msgBoard[board].payload.data[7][1] = 0x00;
    }

    SpiTransfer(msgBoard[0], msgBoard[1]);
}

void LegWheelMotorMit::MotorDisableCmd()
{
    SPI_TO_CAN_V2 msgBoard[2];
    // LOG_INFO("Disable motor");
    for (int board = 0; board < 2; board++) {
        for (int i = 0; i < 8; i++) {
            msgBoard[board].payload.data[i][9] = 0xFD;
        }

        msgBoard[board].payload.data[0][0] = 0x01;
        msgBoard[board].payload.data[0][1] = 0x00;

        msgBoard[board].payload.data[1][0] = 0x02;
        msgBoard[board].payload.data[1][1] = 0x00;

        msgBoard[board].payload.data[2][0] = 0x03;
        msgBoard[board].payload.data[2][1] = 0x00;

        msgBoard[board].payload.data[3][0] = 0x04;
        msgBoard[board].payload.data[3][1] = 0x00;

        msgBoard[board].payload.data[4][0] = 0x01;
        msgBoard[board].payload.data[4][1] = 0x00;

        msgBoard[board].payload.data[5][0] = 0x02;
        msgBoard[board].payload.data[5][1] = 0x00;

        msgBoard[board].payload.data[6][0] = 0x03;
        msgBoard[board].payload.data[6][1] = 0x00;

        msgBoard[board].payload.data[7][0] = 0x04;
        msgBoard[board].payload.data[7][1] = 0x00;
    }

    SpiTransfer(msgBoard[0], msgBoard[1]);
}

void LegWheelMotorMit::SendRecvDeal()
{
    auto cmd = MsgTryRecv<msg::wheel::motor_cmd>("wheel::motor_cmd", this);
    if (cmd) {
        BlockSend(cmd.value());
        CheckMotorState();
    }
}

void LegWheelMotorMit::SpiTransfer(const SPI_TO_CAN_V2 &msg0, const SPI_TO_CAN_V2 &msg1)
{
    spi2can->AddMessage(0, msg0);
    spi2can->AddMessage(1, msg1);
    spi2can->Transfer();
}

void LegWheelMotorMit::CheckMotorState()
{
    auto nowTime = TimerTools::GetNowTickMs();
    for (int leg = 0; leg < 4; leg++) {
        for (int motor = 0; motor < 4; motor++) {
            auto &info = infos_[leg][motor];
            // 检查未收到数据
            if (auto dura = nowTime - info.lastRecv; dura > 300) {
                if (info.state != MotorState::timeout) {
                    info.state = MotorState::timeout;
                }
            }
        }
    }
}
