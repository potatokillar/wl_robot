
#pragma once

#include <optional>

#include "baseline.hpp"
#include "motorMitV3.hpp"
#include "spi2canV3.hpp"
#include "transMotorMit.hpp"

struct QrMotorCfgV3
{
    std::array<std::array<MitMotorCfg, 3>, 4> motorCfg;
    BootArgs boot;  // 启动参数
};

class QrMotorMitV3 : public MotorMitV3
{
public:
    QrMotorMitV3(const QrMotorCfgV3 &cfg, std::unique_ptr<SpiToCanV3> can);
    // 共享所有权，本类支持多个实例操作
    // 类复制，但由于本类管理着设备，设备是唯一的，因此不支持复制
    QrMotorMitV3(const QrMotorMitV3 &) = delete;
    QrMotorMitV3 &operator=(const QrMotorMitV3 &) = delete;

    void Start() override;
    void Stop() override;
    void Run() override;
    void Enable() override;
    void Disable() override;

private:
    bool BlockSend(msg::qr::motor_cmd cmd);
    msg::qr::motor_ret BlockRecv();
    void SendRecvDeal();
    void Transfer(const msg::qr::motor_cmd &cmd);

private:
    msg::qr::motor_ret motorRawPrevData_;  // 历史原始数据，当数据未及时更新时，使用旧数据
    std::unique_ptr<PeriodicMemberFunction<QrMotorMitV3>> task_;

    BootArgs boot_;
    bool biasIsUpdate_{false};
};
