#include "armMotorMit.hpp"

#include <chrono>
#include <iostream>
#include <mutex>
#include <vector>

#include "baseline.hpp"
#include "robotState.hpp"

using namespace std;

/**
 * @description: 构造函数
 * @param &cfg 电机配置
 * @param can can接口实例
 * @return {}
 */
ArmMotorMit::ArmMotorMit(const std::vector<MotorCfg> &cfg, std::unique_ptr<SpiToCan> can) : motorCfg_(cfg), spi2can(std::move(can))
{
    spi2can->ResetTimer();

    for (size_t i = 0; i < cfg.size(); i++) {
        trans_.push_back(make_unique<TransformMitMotor>(cfg[i].model));
    }
}

/**
 * @description: 清除SPI发送接收缓存中的数据
 * @return {}
 */
void ArmMotorMit::ClearAllData()
{
    spi2can->ResetTimer();
    spi2can->ClearBuf();
}

/**
 * @description: 使能，并启动四足电机循环
 * @return {}
 */
void ArmMotorMit::Start()
{
    task_ = make_unique<PeriodicMemberFunction<ArmMotorMit>>("armMotor", 0.004, this, &ArmMotorMit::Run, true);
    task_->Start();
}

/**
 * @description: 失能，并关闭四足电机循环
 * @return {}
 */
void ArmMotorMit::Stop() { task_->Stop(); }

/**
 * @description: 电机发送接收处理
 * @return {}
 */
void ArmMotorMit::Run()
{
    UpdateBasicData();
    RxArmCmd();

    SendMotorCmd();
    RecvMotorRet();

    UpdateStatusWord();
}

void ArmMotorMit::SendMotorCmd()
{
    if (motorInfos_.state != msg::arm_motor_state::enable) {
        return;
    }

    for (int i = 0; i < 3; i++) {
        motorCmd_.motor[i].k2 = 100;
        motorCmd_.motor[i].k1 = 4;
    }
    for (int i = 3; i < 6; i++) {
        motorCmd_.motor[i].k2 = 10;
        motorCmd_.motor[i].k1 = 1.5;
    }

    bool isErr = false;
    for (int i = 0; i < ARM_MOTOR_SIZE; i++) {
        if (fabs(motorCmd_.motor[i].alpha - motorCmdOld_.motor[i].alpha) > 0.1) {
            isErr = true;
            LOG_ERROR("motor cmd step error: {}, {}", motorCmd_.motor[i].alpha, motorCmdOld_.motor[i].alpha);
            break;
        }
    }

    if (isErr == false) {
        BlockSend(motorCmd_);
        motorCmdOld_ = motorCmd_;
    }

    MsgTrySend("arm::arm_cmd", motorCmd_);  // 上报给qrChart
}

void ArmMotorMit::RecvMotorRet()
{
    motorRet_ = BlockRecv();

    MsgTrySend("arm::arm_data", motorRet_);
}
void ArmMotorMit::RxArmCmd()
{
    if (auto ret = MsgTryRecv<bool>("arm::enable", this)) {
        if (ret.value() == true) {
            Enable();
        } else {
            //  Disable();  // MIT电机没有抱闸，为了安全，关闭下使能功能
        }
    }
}
/**
 * @description: 更新状态字
 * @return {}
 */
void ArmMotorMit::UpdateStatusWord()
{
    // std::array<u16, ARM_MOTOR_SIZE> errors;
    auto state = msg::arm_motor_state::enable;  // 默认为全使能
    if (isEnable_ == false) {
        state = msg::arm_motor_state::disable;
    }

    if (state != motorInfos_.state) {
        motorInfos_.state = state;
        cout << "arm motor state change " << endl;
        infoIsUpdate_ = true;
    }
#if 0
    // 无错误码
    if (errors != motorInfos_.errCode) {
        motorInfos_.errCode = errors;
        cout << "arm motor errCode change " << endl;
        infoIsUpdate_ = true;
    }
#endif
}

/**
 * @description: 电机命令发送，该接口直接调用can接口，会阻塞
 * @param cmd 该值会被修改，所以不用const &
 * @return {}
 */
bool ArmMotorMit::BlockSend(msg::arm_cmd cmd)
{
    // 先转极性
    // 算法real部分始终以右手定制为正方向
    // 实物电机若同样以右手为正方形(true)，则保持不变
    // 若实物电机以左手为正方向(false)，则进行取反
    for (size_t i = 0; i < motorCfg_.size(); i++) {
        if (motorCfg_[i].chiral == false) {
            cmd.motor[i].alpha *= -1;
            cmd.motor[i].torq *= -1;
            cmd.motor[i].blta *= -1;
        }
    }

    // 增加电机固有偏移值
    for (size_t i = 0; i < bias.size(); i++) {
        cmd.motor[i].alpha += bias[i];
    }

    SPI_TO_CAN_S msgBoard[2];                 // 将数据转换成两个板子，但机械臂只用到了第一个spi
    auto *board0 = msgBoard[0].payload.data;  // 发送给第一个板子的6串数据

    // 暂时只支持6轴
    for (int i = 0; i < 6; i++) {
        trans_[i]->CmdToCan(board0[i], cmd.motor[i].alpha, cmd.motor[i].torq, cmd.motor[i].blta, cmd.motor[i].k2, cmd.motor[i].k1);
    }

    SpiTransfer(msgBoard[0], msgBoard[1]);
    return true;
}

/**
 * @description: 协议接收，每次调用获取一次，可能重复数据
 * @return {} 数据，若无新数据，则为历史值
 */
msg::arm_data ArmMotorMit ::BlockRecv()
{
    msg::arm_data data = RecvRaw();

    // 减去固有偏移值
    for (size_t i = 0; i < bias.size(); i++) {
        data.motor[i].alpha -= bias[i];
    }

    for (size_t i = 0; i < motorCfg_.size(); i++) {
        if (motorCfg_[i].chiral == false) {
            data.motor[i].alpha *= -1;
            data.motor[i].torq *= -1;
            data.motor[i].blta *= -1;
        }
    }

    return data;
}

/**
 * @description: 接收原始值，不经过偏移值处理和极性转化
 * @return {}
 */
msg::arm_data ArmMotorMit::RecvRaw()
{
    SPI_TO_CAN_S msgBoard[2];
    msgBoard[0] = spi2can->GetMessage(0);
    msgBoard[1] = spi2can->GetMessage(1);
    u8 t;
    // 如果转换失败，则不会存放到motorData中，因此返回的会是历史值
    for (int i = 0; i < 6; i++) {
        trans_[i]->CanToData(msgBoard[0].payload.data[i], &motorRawPrevData_.motor[i].alpha, &motorRawPrevData_.motor[i].torq, &motorRawPrevData_.motor[i].blta, &t, &state_[i]);
    }

    UNUSED(t);
    UNUSED(state_);

    return motorRawPrevData_;
}

/**
 * @description: 保存电机的偏移值，上电后保存一次
 * @param &data
 * @return {}
 */
bool ArmMotorMit::BiasSave(const msg::arm_data &data)
{
    bool ret = true;  // 只要有一个为0，就表示通讯失败，不保存此次结果

    for (int i = 0; i < 6; i++) {
        bias[i] = data.motor[i].alpha;
        if (bias[i] == 0.0) {
            ret = false;
        }
    }

    return ret;
}

MotorState ArmMotorMit ::GetState()
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

void ArmMotorMit::Enable()
{
    if (motorInfos_.state != msg::arm_motor_state::enable) {
        msg::arm_data motorData;
        msg::arm_cmd motorCmd;
        ClearAllData();
        // todo 目前上下使能是共用同一个逻辑，即上下使能不支持不同协议的电机
        // 需要各种配置单独化
        // tMotor_12_8是我们自定义的电机，支持使能状态上报
        if (motorCfg_[0].model == MotorModel::tMotor_12_8) {
            // 基于spi全双工的特性，第一次下发的数据，在第二次下发spi时才能获取到第一次的返回
            // 通过抓取数据，电机第二次返回使能完成信号，那么在spi层，就是第三次下发才能获取
            // 这里连续发四次，保证数据获取正确
            for (int i = 0; i < 4; i++) {
                MotorEnableCmd();
                TimerTools::SleepForMs(5);
            }

            // 连续检测两次是否使能成功，两次均失败则不再检测
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

            // 打印使能情况
            for (int i = 0; i < 6; i++) {
                if (((state_[i] & 0x01) != 0x01) || (state_[i] == 0xff)) {
                    LOG_ERROR("motor enable err: {}", i);
                }
            }
        } else {
            MotorEnableCmd();
            TimerTools::SleepForMs(2);
        }

        // 由于spi全双工的机制，当前下发指令的返回值是电机上一次的值。
        // 因此需要多发几帧，确保返回值不能是归零之前的值。
        // 实测AK电机第1帧(0开始)开始即为归零后的值。海泰电机第2帧开始即为归零后的值。
        // 这里取第3帧为bias的值
        for (int i = 0; i < 5; i++) {
            BlockSend(motorCmd);
            RecvRaw();
            TimerTools::SleepForMs(2);
        }

        if (isBias_ == false) {
            // bias未初始化时默认是0，因此此时Recv()的值就是实际值
            isBias_ = BiasSave(RecvRaw());
        }
        isEnable_ = true;
        LOG_INFO("motor enable ok");

        // 接收值放入发送值，阻止速度超限
        for (int i = 0; i < ARM_MOTOR_SIZE; i++) {
            motorCmd_.motor[i].alpha = motorRet_.motor[i].alpha;
            motorCmd_.motor[i].torq = motorRet_.motor[i].torq;
            motorCmd_.motor[i].blta = motorRet_.motor[i].blta;
        }
    }
}

/**
 * @description: 下使能
 * @return {}
 */
void ArmMotorMit::Disable()
{
    msg::arm_cmd motorCmd;
    BlockSend(motorCmd);  // 掉使能前发0，清除缓存

    MotorDisableCmd();
    TimerTools::SleepForMs(2);
    MotorDisableCmd();

    ClearAllData();
    isEnable_ = false;
    LOG_INFO("motor disable ok");
}

/**
 * @description: 归零指令
 * @return {}
 */
void ArmMotorMit::MotorZeroCmd()
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

/**
 * @description: 使能指令
 * @return {}
 */
void ArmMotorMit::MotorEnableCmd()
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

/**
 * @description: 失能指令
 * @return {}
 */
void ArmMotorMit::MotorDisableCmd()
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

/**
 * @description: 根据规定的溢出值计算新值和旧值之间的差值(间隔值，必为正)
 * @param oldV 旧值
 * @param newV 新值
 * @param downOver 下边界
 * @param upOver 上边界
 * @return {} 下溢出，上溢出，不溢出
 */
std::vector<double> ArmMotorMit::CalOverflow(double oldV, double newV, double overFlow)
{
    double upOver = fabs(overFlow);  // 防止传入下溢出值
    double downOver = -fabs(overFlow);
    // 部分情况下，旧的海泰电机，其newV可能超过溢出值，但不需要做任何处理也没问题，见测试用例

    // 上溢出，新值数值上 < 旧值
    double up1 = upOver - oldV;  // 旧值增加多少值会到达上溢出
    // cout << "up1 " << up1 << endl;
    double up2 = newV - downOver;  // 下溢出增加多少值到达新值
    // cout << "up2 " << up2 << endl;
    double up = up1 + up2;  // 两者之和就是旧值到新值需要增加的值，恒为正值
    // double up = newV - oldV + upOver - downOver;

    // 下溢出，新值数值上 > 旧值
    double down1 = oldV - downOver;  // 旧值减少多少值到达下溢出
                                     // cout << "do1 " << down1 << endl;
    double down2 = upOver - newV;    // 上溢出减少多少值到达新值
                                     // cout << "do2 " << down2 << endl;
    double down = down1 + down2;     // 两者之和就是旧值到新值需要减少的值，恒为正值
                                     // double down = oldV-newV + upOver - downOver;

    // 不溢出
    double no = fabs(newV - oldV);  // 旧值需要增加多少值到达新值，有正有负，但需取正值
    return {down, up, no};
}

/**
 * @description: 调用SPI发送命令
 * @return {}
 */
void ArmMotorMit::SpiTransfer(const SPI_TO_CAN_S &msg0, const SPI_TO_CAN_S &msg1)
{
    lock_guard<mutex> lock(mutex_);
    spi2can->AddMessage(0, msg0);
    spi2can->AddMessage(1, msg1);
    spi2can->Transfer();
}

/**
 * @description: 检测是否电机为使能
 * @return {}
 */
bool ArmMotorMit::CheckEnable()
{
    RecvRaw();
    for (int i = 0; i < 3; i++) {
        if (((state_[i] & 0x01) != 0x01) || (state_[i] == 0xff)) {
            return false;
        }
    }
    return true;
}
