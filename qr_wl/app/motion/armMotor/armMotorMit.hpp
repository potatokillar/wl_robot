

#pragma once

#include "armMotor.hpp"
#include "baseline.hpp"
#include "deviceParam.hpp"
#include "spi2can.hpp"
#include "transMotorMit.hpp"

class ArmMotorMit : public ArmMotor
{
public:
    ArmMotorMit(const std::vector<MotorCfg> &cfg, std::unique_ptr<SpiToCan> can);
    // 共享所有权，本类支持多个实例操作
    // 类复制，但由于本类管理着设备，设备是唯一的，因此不支持复制
    ArmMotorMit(const ArmMotorMit &) = delete;
    ArmMotorMit &operator=(const ArmMotorMit &) = delete;

    void Start() override;
    void Stop() override;
    void Enable() override;
    void Disable() override;

private:
    void Run();
    bool BlockSend(msg::arm_cmd cmd);
    msg::arm_data BlockRecv();

    msg::arm_data RecvRaw();
    void MotorEnableCmd();
    void MotorDisableCmd();
    void MotorZeroCmd();
    bool BiasSave(const msg::arm_data &data);
    std::vector<double> CalOverflow(double oldV, double newV, double overFlow);

    void ClearAllData();
    MotorState GetState();
    void SpiTransfer(const SPI_TO_CAN_S &msg0, const SPI_TO_CAN_S &msg1);
    bool CheckEnable();
    void UpdateStatusWord();
    void RxArmCmd();
    void SendMotorCmd();
    void RecvMotorRet();

private:
    msg::arm_cmd motorCmdOld_;
    msg::arm_data motorRawPrevData_;  // 历史原始数据，当数据未及时更新时，使用旧数据

    std::array<uint8_t, 6> state_;
    std::vector<MotorCfg> motorCfg_;
    std::unique_ptr<SpiToCan> spi2can;                       // 不建议使用shared_ptr，因为该设备是唯一的，且只能由本类管理
    std::vector<std::unique_ptr<TransformMitMotor>> trans_;  // 6或7轴
    bool isBias_{false};
    std::array<double, 6> bias;  // 仅在电机上电后刷新一次
    std::unique_ptr<PeriodicMemberFunction<ArmMotorMit>> task_;
    bool errorIsPrint_;

    std::mutex mutex_;

    bool isEnable_{false};
};
