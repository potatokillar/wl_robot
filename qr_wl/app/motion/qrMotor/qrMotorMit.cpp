#include "qrMotorMit.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

#include "baseline.hpp"
#include "robotState.hpp"

using namespace std;

MotorBias QrMotorMit::bias;  // 类中静态变量类外初始化

/**
 * @description: 构造函数
 * @param &cfg 电机配置
 * @param can can接口实例
 * @return {}
 */
QrMotorMit::QrMotorMit(const QrMotorCfg &cfg, std::unique_ptr<SpiToCan> can) : motorCfg_(cfg), spi2can(std::move(can))
{
    spi2can->ResetTimer();

    for (auto i = 0; i < 3; i++) {
        for (auto j = 0; j < 4; j++) {
            trans_(i, j) = make_unique<TransformMitMotor>(motorCfg_.motorModel(i, j));
        }
    }
    legNameMap_[0] = "front  left";
    legNameMap_[1] = "front right";
    legNameMap_[2] = "rear  right";
    legNameMap_[3] = "rear   left";

    jointNameMap_[0] = "abad";
    jointNameMap_[1] = "hip ";
    jointNameMap_[2] = "knee";
}

/**
 * @description: 清除SPI发送接收缓存中的数据
 * @return {}
 */
void QrMotorMit::ClearAllData()
{
    spi2can->ResetTimer();
    spi2can->ClearBuf();
}

/**
 * @description: 使能，并启动四足电机循环
 * @return {}
 */
void QrMotorMit::Start()
{
    Enable();
    task_ = make_unique<PeriodicMemberFunction<QrMotorMit>>("qrMotor", 0.002, this, &QrMotorMit::Run, true);
    task_->Start();
}

void QrMotorMit::Stop()
{
    task_->Stop();
    Disable();
}

/**
 * @description: 电机发送接收处理
 * @return {}
 */
void QrMotorMit::Run()
{
    if ((GetRobotCurState() == RobotState::running) || (GetRobotCurState() == RobotState::jointCtrl)) {
        SendRecvDeal();
    } else {
        // 算法未启动，发阻尼模式指令

        msg::qr::motor_cmd motorCmd;
#if 0
        for (int i = 0; i < 4; i++) {
            motorCmd.kd_abad[i] = 3;
            motorCmd.kd_hip[i] = 3;
            motorCmd.kd_knee[i] = 3;
        }
#endif
        for (int leg = 0; leg < 4; leg++) {
            for (int motor = 0; motor < 3; motor++) {
                motorCmd.leg[leg][motor].k1 = 3;
            }
        }

        BlockSend(motorCmd);
    }
    motorRet_ = BlockRecv();
    MsgTrySend("qr::motor_ret", motorRet_);
}

bool QrMotorMit::BlockSend(const msg::qr::motor_cmd &_cmd)
{
    auto cmd = _cmd;

    // 先转极性
    for (auto j = 0; j < 4; j++) {
        if (motorCfg_.chiral(0, j) == false) {
            // cmd.q_abad[j] *= -1;
            // cmd.qd_abad[j] *= -1;
            // cmd.t_abad[j] *= -1;
            cmd.leg[j][0].alpha *= -1;
            cmd.leg[j][0].torq *= -1;
            cmd.leg[j][0].blta *= -1;
        }

        if (motorCfg_.chiral(1, j) == false) {
            // cmd.q_hip[j] *= -1;
            // cmd.qd_hip[j] *= -1;
            // cmd.t_hip[j] *= -1;
            cmd.leg[j][1].alpha *= -1;
            cmd.leg[j][1].torq *= -1;
            cmd.leg[j][1].blta *= -1;
        }

        if (motorCfg_.chiral(2, j) == false) {
            // cmd.q_knee[j] *= -1;
            // cmd.qd_knee[j] *= -1;
            // cmd.t_knee[j] *= -1;
            cmd.leg[j][2].alpha *= -1;
            cmd.leg[j][2].torq *= -1;
            cmd.leg[j][2].blta *= -1;
        }
    }

    // 增加电机固有偏移值
    for (int i = 0; i < 4; i++) {
        // cmd.q_abad[i] += bias.q_abad[i];
        // cmd.q_hip[i] += bias.q_hip[i];
        // cmd.q_knee[i] += bias.q_knee[i];
        cmd.leg[i][0].alpha += bias.q_abad[i];
        cmd.leg[i][1].alpha += bias.q_hip[i];
        cmd.leg[i][2].alpha += bias.q_knee[i];
    }

    SPI_TO_CAN_S msgBoard[2];                 // 将数据转换成两个板子
    auto *board0 = msgBoard[0].payload.data;  // 发送给第一个板子的6串数据
#if 0
    trans_(0, 0)->CmdToCan(board0[0], cmd.q_abad[0], cmd.qd_abad[0], cmd.t_abad[0], cmd.kp_abad[0], cmd.kd_abad[0]);
    trans_(1, 0)->CmdToCan(board0[1], cmd.q_hip[0], cmd.qd_hip[0], cmd.t_hip[0], cmd.kp_hip[0], cmd.kd_hip[0]);
    trans_(2, 0)->CmdToCan(board0[2], cmd.q_knee[0], cmd.qd_knee[0], cmd.t_knee[0], cmd.kp_knee[0], cmd.kd_knee[0]);

    // leg2
    trans_(0, 1)->CmdToCan(board0[3], cmd.q_abad[1], cmd.qd_abad[1], cmd.t_abad[1], cmd.kp_abad[1], cmd.kd_abad[1]);
    trans_(1, 1)->CmdToCan(board0[4], cmd.q_hip[1], cmd.qd_hip[1], cmd.t_hip[1], cmd.kp_hip[1], cmd.kd_hip[1]);
    trans_(2, 1)->CmdToCan(board0[5], cmd.q_knee[1], cmd.qd_knee[1], cmd.t_knee[1], cmd.kp_knee[1], cmd.kd_knee[1]);

    auto *board1 = msgBoard[1].payload.data;
    // leg3
    trans_(0, 2)->CmdToCan(board1[0], cmd.q_abad[2], cmd.qd_abad[2], cmd.t_abad[2], cmd.kp_abad[2], cmd.kd_abad[2]);
    trans_(1, 2)->CmdToCan(board1[1], cmd.q_hip[2], cmd.qd_hip[2], cmd.t_hip[2], cmd.kp_hip[2], cmd.kd_hip[2]);
    trans_(2, 2)->CmdToCan(board1[2], cmd.q_knee[2], cmd.qd_knee[2], cmd.t_knee[2], cmd.kp_knee[2], cmd.kd_knee[2]);

    // leg4
    trans_(0, 3)->CmdToCan(board1[3], cmd.q_abad[3], cmd.qd_abad[3], cmd.t_abad[3], cmd.kp_abad[3], cmd.kd_abad[3]);
    trans_(1, 3)->CmdToCan(board1[4], cmd.q_hip[3], cmd.qd_hip[3], cmd.t_hip[3], cmd.kp_hip[3], cmd.kd_hip[3]);
    trans_(2, 3)->CmdToCan(board1[5], cmd.q_knee[3], cmd.qd_knee[3], cmd.t_knee[3], cmd.kp_knee[3], cmd.kd_knee[3]);
#endif
    trans_(0, 0)->CmdToCan(board0[0], cmd.leg[0][0].alpha, cmd.leg[0][0].torq, cmd.leg[0][0].blta, cmd.leg[0][0].k2, cmd.leg[0][0].k1);
    trans_(1, 0)->CmdToCan(board0[1], cmd.leg[0][1].alpha, cmd.leg[0][1].torq, cmd.leg[0][1].blta, cmd.leg[0][1].k2, cmd.leg[0][1].k1);
    trans_(2, 0)->CmdToCan(board0[2], cmd.leg[0][2].alpha, cmd.leg[0][2].torq, cmd.leg[0][2].blta, cmd.leg[0][2].k2, cmd.leg[0][2].k1);

    // leg2
    trans_(0, 1)->CmdToCan(board0[3], cmd.leg[1][0].alpha, cmd.leg[1][0].torq, cmd.leg[1][0].blta, cmd.leg[1][0].k2, cmd.leg[1][0].k1);
    trans_(1, 1)->CmdToCan(board0[4], cmd.leg[1][1].alpha, cmd.leg[1][1].torq, cmd.leg[1][1].blta, cmd.leg[1][1].k2, cmd.leg[1][1].k1);
    trans_(2, 1)->CmdToCan(board0[5], cmd.leg[1][2].alpha, cmd.leg[1][2].torq, cmd.leg[1][2].blta, cmd.leg[1][2].k2, cmd.leg[1][2].k1);

    auto *board1 = msgBoard[1].payload.data;
    // leg3
    trans_(0, 2)->CmdToCan(board1[0], cmd.leg[2][0].alpha, cmd.leg[2][0].torq, cmd.leg[2][0].blta, cmd.leg[2][0].k2, cmd.leg[2][0].k1);
    trans_(1, 2)->CmdToCan(board1[1], cmd.leg[2][1].alpha, cmd.leg[2][1].torq, cmd.leg[2][1].blta, cmd.leg[2][1].k2, cmd.leg[2][1].k1);
    trans_(2, 2)->CmdToCan(board1[2], cmd.leg[2][2].alpha, cmd.leg[2][2].torq, cmd.leg[2][2].blta, cmd.leg[2][2].k2, cmd.leg[2][2].k1);

    // leg4
    trans_(0, 3)->CmdToCan(board1[3], cmd.leg[3][0].alpha, cmd.leg[3][0].torq, cmd.leg[3][0].blta, cmd.leg[3][0].k2, cmd.leg[3][0].k1);
    trans_(1, 3)->CmdToCan(board1[4], cmd.leg[3][1].alpha, cmd.leg[3][1].torq, cmd.leg[3][1].blta, cmd.leg[3][1].k2, cmd.leg[3][1].k1);
    trans_(2, 3)->CmdToCan(board1[5], cmd.leg[3][2].alpha, cmd.leg[3][2].torq, cmd.leg[3][2].blta, cmd.leg[3][2].k2, cmd.leg[3][2].k1);

    SpiTransfer(msgBoard[0], msgBoard[1]);
    return true;
}

/**
 * @description: 协议接收，每次调用获取一次，可能重复数据
 * @return {} 数据，若无新数据，则为历史值
 */
msg::qr::motor_ret QrMotorMit ::BlockRecv()
{
    msg::qr::motor_ret data = RecvRaw();

    // 减去固有偏移值
    for (int i = 0; i < 4; i++) {
        // data.q_abad[i] -= bias.q_abad[i];
        // data.q_hip[i] -= bias.q_hip[i];
        // data.q_knee[i] -= bias.q_knee[i];
        data.leg[i][0].alpha -= bias.q_abad[i];
        data.leg[i][1].alpha -= bias.q_hip[i];
        data.leg[i][2].alpha -= bias.q_knee[i];
    }

    for (auto j = 0; j < 4; j++) {
        if (motorCfg_.chiral(0, j) == false) {
            // data.q_abad[j] *= -1;
            // data.qd_abad[j] *= -1;
            // data.t_abad[j] *= -1;
            data.leg[j][0].alpha *= -1;
            data.leg[j][0].torq *= -1;
            data.leg[j][0].blta *= -1;
        }

        if (motorCfg_.chiral(1, j) == false) {
            // data.q_hip[j] *= -1;
            // data.qd_hip[j] *= -1;
            // data.t_hip[j] *= -1;
            data.leg[j][1].alpha *= -1;
            data.leg[j][1].torq *= -1;
            data.leg[j][1].blta *= -1;
        }

        if (motorCfg_.chiral(2, j) == false) {
            // data.q_knee[j] *= -1;
            // data.qd_knee[j] *= -1;
            // data.t_knee[j] *= -1;
            data.leg[j][2].alpha *= -1;
            data.leg[j][2].torq *= -1;
            data.leg[j][2].blta *= -1;
        }
    }

    return data;
}

msg::qr::motor_ret QrMotorMit::RecvRaw()
{
    SPI_TO_CAN_S msgBoard[2];
    msgBoard[0] = spi2can->GetMessage(0);
    msgBoard[1] = spi2can->GetMessage(1);
    u8 t;

#if 0
    // leg1
    trans_(0, 0)->CanToData(msgBoard[0].payload.data[0], &motorRawPrevData_.q_abad[0], &motorRawPrevData_.qd_abad[0], &motorRawPrevData_.t_abad[0], &t, &state_(0, 0));
    trans_(1, 0)->CanToData(msgBoard[0].payload.data[1], &motorRawPrevData_.q_hip[0], &motorRawPrevData_.qd_hip[0], &motorRawPrevData_.t_hip[0], &t, &state_(1, 0));
    trans_(2, 0)->CanToData(msgBoard[0].payload.data[2], &motorRawPrevData_.q_knee[0], &motorRawPrevData_.qd_knee[0], &motorRawPrevData_.t_knee[0], &t, &state_(2, 0));
    // leg2
    trans_(0, 1)->CanToData(msgBoard[0].payload.data[3], &motorRawPrevData_.q_abad[1], &motorRawPrevData_.qd_abad[1], &motorRawPrevData_.t_abad[1], &t, &state_(0, 1));
    trans_(1, 1)->CanToData(msgBoard[0].payload.data[4], &motorRawPrevData_.q_hip[1], &motorRawPrevData_.qd_hip[1], &motorRawPrevData_.t_hip[1], &t, &state_(1, 1));
    trans_(2, 1)->CanToData(msgBoard[0].payload.data[5], &motorRawPrevData_.q_knee[1], &motorRawPrevData_.qd_knee[1], &motorRawPrevData_.t_knee[1], &t, &state_(2, 1));
    // leg3
    trans_(0, 2)->CanToData(msgBoard[1].payload.data[0], &motorRawPrevData_.q_abad[2], &motorRawPrevData_.qd_abad[2], &motorRawPrevData_.t_abad[2], &t, &state_(0, 2));
    trans_(1, 2)->CanToData(msgBoard[1].payload.data[1], &motorRawPrevData_.q_hip[2], &motorRawPrevData_.qd_hip[2], &motorRawPrevData_.t_hip[2], &t, &state_(1, 2));
    trans_(2, 2)->CanToData(msgBoard[1].payload.data[2], &motorRawPrevData_.q_knee[2], &motorRawPrevData_.qd_knee[2], &motorRawPrevData_.t_knee[2], &t, &state_(2, 2));
    // leg4
    trans_(0, 3)->CanToData(msgBoard[1].payload.data[3], &motorRawPrevData_.q_abad[3], &motorRawPrevData_.qd_abad[3], &motorRawPrevData_.t_abad[3], &t, &state_(0, 3));
    trans_(1, 3)->CanToData(msgBoard[1].payload.data[4], &motorRawPrevData_.q_hip[3], &motorRawPrevData_.qd_hip[3], &motorRawPrevData_.t_hip[3], &t, &state_(1, 3));
    trans_(2, 3)->CanToData(msgBoard[1].payload.data[5], &motorRawPrevData_.q_knee[3], &motorRawPrevData_.qd_knee[3], &motorRawPrevData_.t_knee[3], &t, &state_(2, 3));
#endif
    // leg1
    trans_(0, 0)
        ->CanToData(msgBoard[0].payload.data[0], &motorRawPrevData_.leg[0][0].alpha, &motorRawPrevData_.leg[0][0].torq, &motorRawPrevData_.leg[0][0].blta, &t, &state_(0, 0));
    trans_(1, 0)
        ->CanToData(msgBoard[0].payload.data[1], &motorRawPrevData_.leg[0][1].alpha, &motorRawPrevData_.leg[0][1].torq, &motorRawPrevData_.leg[0][1].blta, &t, &state_(1, 0));
    trans_(2, 0)
        ->CanToData(msgBoard[0].payload.data[2], &motorRawPrevData_.leg[0][2].alpha, &motorRawPrevData_.leg[0][2].torq, &motorRawPrevData_.leg[0][2].blta, &t, &state_(2, 0));
    // leg2
    trans_(0, 1)
        ->CanToData(msgBoard[0].payload.data[3], &motorRawPrevData_.leg[1][0].alpha, &motorRawPrevData_.leg[1][0].torq, &motorRawPrevData_.leg[1][0].blta, &t, &state_(0, 1));
    trans_(1, 1)
        ->CanToData(msgBoard[0].payload.data[4], &motorRawPrevData_.leg[1][1].alpha, &motorRawPrevData_.leg[1][1].torq, &motorRawPrevData_.leg[1][1].blta, &t, &state_(1, 1));
    trans_(2, 1)
        ->CanToData(msgBoard[0].payload.data[5], &motorRawPrevData_.leg[1][2].alpha, &motorRawPrevData_.leg[1][2].torq, &motorRawPrevData_.leg[1][2].blta, &t, &state_(2, 1));
    // leg3
    trans_(0, 2)
        ->CanToData(msgBoard[1].payload.data[0], &motorRawPrevData_.leg[2][0].alpha, &motorRawPrevData_.leg[2][0].torq, &motorRawPrevData_.leg[2][0].blta, &t, &state_(0, 2));
    trans_(1, 2)
        ->CanToData(msgBoard[1].payload.data[1], &motorRawPrevData_.leg[2][1].alpha, &motorRawPrevData_.leg[2][1].torq, &motorRawPrevData_.leg[2][1].blta, &t, &state_(1, 2));
    trans_(2, 2)
        ->CanToData(msgBoard[1].payload.data[2], &motorRawPrevData_.leg[2][2].alpha, &motorRawPrevData_.leg[2][2].torq, &motorRawPrevData_.leg[2][2].blta, &t, &state_(2, 2));
    // leg4
    trans_(0, 3)
        ->CanToData(msgBoard[1].payload.data[3], &motorRawPrevData_.leg[3][0].alpha, &motorRawPrevData_.leg[3][0].torq, &motorRawPrevData_.leg[3][0].blta, &t, &state_(0, 3));
    trans_(1, 3)
        ->CanToData(msgBoard[1].payload.data[4], &motorRawPrevData_.leg[3][1].alpha, &motorRawPrevData_.leg[3][1].torq, &motorRawPrevData_.leg[3][1].blta, &t, &state_(1, 3));
    trans_(2, 3)
        ->CanToData(msgBoard[1].payload.data[5], &motorRawPrevData_.leg[3][2].alpha, &motorRawPrevData_.leg[3][2].torq, &motorRawPrevData_.leg[3][2].blta, &t, &state_(2, 3));

    UNUSED(t);
    UNUSED(state_);
    msg::qr::motor_ret data = motorRawPrevData_;

    return data;
}

std::optional<Mat34<double>> QrMotorMit::SaveZeroPos(const msg::qr::motor_ret &data)
{
    if (reqModifyZeroPos_ == false) {
        return nullopt;
    }

    bool write = true;

    if (motorCfg_.boot.noMotor == false) {
        for (int i = 0; i < 4; i++) {
            if (data.leg[i][0].alpha == 0) {
                write = false;
            }
            if (data.leg[i][1].alpha == 0) {
                write = false;
            }
            if (data.leg[i][2].alpha == 0) {
                write = false;
            }
        }
    }

    if (write == true) {
        Mat34<double> pos;
        pos << data.leg[0][0].alpha, data.leg[1][0].alpha, data.leg[2][0].alpha, data.leg[3][0].alpha, data.leg[0][1].alpha, data.leg[1][1].alpha, data.leg[2][1].alpha,
            data.leg[3][1].alpha, data.leg[0][2].alpha, data.leg[1][2].alpha, data.leg[2][2].alpha, data.leg[3][2].alpha;

        if (callbackFunc_.count("joint_write_complete")) {
            callbackFunc_.at("joint_write_complete")(pos);
            reqModifyZeroPos_ = false;
        }
        return pos;
    }
    return nullopt;
}

bool QrMotorMit::BiasSave(const msg::qr::motor_ret &data)
{
    bool ret = true;
    Mat34<double> zeroPos;
    auto update = SaveZeroPos(data);
    if (update) {
        zeroPos = update.value();
    } else {
        zeroPos = motorCfg_.zeroPos;
    }

    constexpr double PI = 3.1415926;
    for (int i = 0; i < 4; i++) {
        if (data.leg[i][0].alpha == 0.0) {
            bias.q_abad[i] = 0;
            ret = false;
        } else {
            bias.q_abad[i] = data.leg[i][0].alpha;
        }

        if (data.leg[i][1].alpha == 0.0) {
            bias.q_hip[i] = 0;
            ret = false;
        } else {
            bias.q_hip[i] = data.leg[i][1].alpha;
        }

        if (data.leg[i][2].alpha == 0.0) {
            bias.q_knee[i] = 0;
            ret = false;
        } else {
            bias.q_knee[i] = data.leg[i][2].alpha;
        }
    }

    return ret;
}

MotorState QrMotorMit ::GetState()
{
    auto motorSta = spi2can->GetMotorState();

    for (int i = 0; i < motorSta.rows(); i++) {
        for (int j = 0; j < motorSta.cols(); j++) {
            if (motorSta(i, j) == false) {
                return MotorState::error;
            }
        }
    }
    return MotorState::ok;
}

void QrMotorMit::Enable()
{
    msg::qr::motor_ret motorData;
    msg::qr::motor_cmd motorCmd;
    ClearAllData();

    if (motorCfg_.motorModel(0, 0) == MotorModel::tMotor_12_8) {
        for (int i = 0; i < 4; i++) {
            MotorEnableCmd();
            TimerTools::SleepForMs(5);
        }

        for (int check = 0; check < 2; check++) {
            if (CheckEnable() == false) {
                for (int i = 0; i < 4; i++) {
                    MotorEnableCmd();
                    TimerTools::SleepForMs(5);
                }
            } else {
                break;
            }
        }

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 3; j++) {
                if (((state_(j, i) & 0x01) != 0x01) || (state_(j, i) == 0xff)) {
                    LOG_ERROR("motor enable err: {} {}", legNameMap_[i], j);
                }
            }
        }
    } else {
        MotorEnableCmd();
        TimerTools::SleepForMs(2);
    }

    for (int i = 0; i < 4; i++) {
        BlockSend(motorCmd);
        RecvRaw();
        TimerTools::SleepForMs(2);
    }

    if (bias.isSave == false) {
        bias.isSave = BiasSave(RecvRaw());
    }
    auto firstData = BlockRecv();
    bool isBigNum = false;
    int isZero = 0;
    for (int i = 0; i < 4; i++) {
        LOG_INFO("current joint angle, leg:{}, abad:{:.3f}, hip:{:.3f}, knee:{:.3f}",
                 legNameMap_[i],
                 firstData.leg[i][0].alpha,
                 firstData.leg[i][1].alpha,
                 firstData.leg[i][2].alpha);
    }
}

void QrMotorMit::Disable()
{
    msg::qr::motor_cmd motorCmd;
    BlockSend(motorCmd);

    MotorDisableCmd();
    TimerTools::SleepForMs(2);
    MotorDisableCmd();

    ClearAllData();
}

void QrMotorMit::MotorZeroCmd()
{
    SPI_TO_CAN_S msgBoard[2];
    // LOG_INFO("Zero motor");
    for (int board = 0; board < 2; board++) {
        for (int i = 0; i < 6; i++) {
            msgBoard[board].payload.data[i][7] = 0xFE;
        }
    }

    SpiTransfer(msgBoard[0], msgBoard[1]);
}

void QrMotorMit::MotorEnableCmd()
{
    SPI_TO_CAN_S msgBoard[2];
    // LOG_INFO("Enable motor");
    for (int board = 0; board < 2; board++) {
        for (int i = 0; i < 6; i++) {
            msgBoard[board].payload.data[i][7] = 0xFC;
        }
    }

    SpiTransfer(msgBoard[0], msgBoard[1]);
}

void QrMotorMit::MotorDisableCmd()
{
    SPI_TO_CAN_S msgBoard[2];
    // LOG_INFO("Disable motor");
    for (int board = 0; board < 2; board++) {
        for (int i = 0; i < 6; i++) {
            msgBoard[board].payload.data[i][7] = 0xFD;
        }
    }

    SpiTransfer(msgBoard[0], msgBoard[1]);
}

void QrMotorMit::SendRecvDeal()
{
    // 足部通讯处理
    auto cmd = MsgTryRecv<msg::qr::motor_cmd>("qr::motor_cmd", this);
    if (cmd) {
        BlockSend(cmd.value());

        if (GetState() == MotorState::ok) {
            if (errorIsPrint_ == true) {
                LOG_DEBUG("qr motor recover");
                errorIsPrint_ = false;
            }
        } else {
            if (errorIsPrint_ == false) {
                errorIsPrint_ = true;

                if (callbackFunc_.count("joint_error")) {
                    callbackFunc_.at("joint_error")(true);
                    reqModifyZeroPos_ = true;
                }
            }
        }
    }
}

void QrMotorMit::SpiTransfer(const SPI_TO_CAN_S &msg0, const SPI_TO_CAN_S &msg1)
{
    lock_guard<mutex> lock(mutex_);
    spi2can->AddMessage(0, msg0);
    spi2can->AddMessage(1, msg1);
    spi2can->Transfer();
}

bool QrMotorMit::CheckEnable()
{
    RecvRaw();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            if (((state_(i, j) & 0x01) != 0x01) || (state_(j, i) == 0xff)) {
                return false;
            }
        }
    }
    return true;
}

RetState QrMotorMit::SetParam(const std::string &type, std::any param)
{
    if (type == "req_zeropos_write") {
        reqModifyZeroPos_ = std::any_cast<bool>(param);
        return RetState::ok;
    }

    return RetState::noSupport;
}
RetState QrMotorMit::SetCallback(const std::string &type, QrCallbackFunc func)
{
    if (type == "joint_write_complete") {
        callbackFunc_["joint_write_complete"] = func;
        return RetState::ok;
    }

    if (type == "joint_error") {
        callbackFunc_["joint_error"] = func;
        return RetState::ok;
    }

    return RetState::noSupport;
}
