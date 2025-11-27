
#pragma once

#include <optional>

#include "baseline.hpp"
#include "qrMotor.hpp"
#include "spi2canV2.hpp"
#include "transMotorMit.hpp"

class LegWheelMotorMit : public QrMotor
{
public:
    LegWheelMotorMit(const LegWheelMotorCfg &cfg, std::unique_ptr<SpiToCanV2> can);

    LegWheelMotorMit(const LegWheelMotorMit &) = delete;
    LegWheelMotorMit &operator=(const LegWheelMotorMit &) = delete;

    void Start() override;
    void Stop() override;
    void Run() override;
    void Enable() override;
    void Disable() override;

private:
    bool BlockSend(msg::wheel::motor_cmd cmd);
    msg::wheel::motor_ret BlockRecv();
    msg::wheel::motor_ret RecvRaw();

    void MotorEnableCmd();
    void MotorDisableCmd();
    void MotorZeroCmd();

    void CheckInit(const msg::wheel::motor_ret &data);

    void ClearAllData();
    void SendRecvDeal();
    void SaveZeroPos2Database(const msg::wheel::motor_ret &data);
    std::optional<std::array<std::array<double, 4>, 4>> GetZeroPosFromDatabase();
    void SpiTransfer(const SPI_TO_CAN_V2 &msg0, const SPI_TO_CAN_V2 &msg1);

    void CheckMotorState();

private:
    msg::wheel::motor_ret motorRawPrevData_;

    std::array<std::array<QrMotorInfo, 4>, 4> infos_;
    std::unique_ptr<SpiToCanV2> spi2can;

    std::unique_ptr<PeriodicMemberFunction<LegWheelMotorMit>> task_;

    BootArgs boot_;
    bool biasIsUpdate_{false};
};
