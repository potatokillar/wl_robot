
#pragma once

#include <optional>

#include "baseline.hpp"
#include "motorMitV3.hpp"
#include "spi2canV3.hpp"
#include "transMotorMit.hpp"

class LegWheelMotorMitV3 : public MotorMitV3
{
public:
    LegWheelMotorMitV3(const LegWheelMotorCfg &cfg, std::unique_ptr<SpiToCanV3> can);

    LegWheelMotorMitV3(const LegWheelMotorMitV3 &) = delete;
    LegWheelMotorMitV3 &operator=(const LegWheelMotorMitV3 &) = delete;

    void Start() override;
    void Stop() override;
    void Run() override;
    void Enable() override;
    void Disable() override;

private:
    bool BlockSend(msg::wheel::motor_cmd cmd);
    msg::wheel::motor_ret BlockRecv();
    void SendRecvDeal();
    void Transfer(const msg::wheel::motor_cmd &cmd);

private:
    msg::wheel::motor_ret motorRawPrevData_;  // 历史原始数据，当数据未及时更新时，使用旧数据
    std::unique_ptr<PeriodicMemberFunction<LegWheelMotorMitV3>> task_;
};
