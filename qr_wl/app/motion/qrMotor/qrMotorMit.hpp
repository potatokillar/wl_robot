
#pragma once

#include "baseline.hpp"
#include "qrMotor.hpp"
#include "qrMotorType.hpp"
#include "spi2can.hpp"
#include "transMotorMit.hpp"

struct MotorBias
{
    bool isSave{false};
    double q_abad[4]{0};
    double q_hip[4]{0};
    double q_knee[4]{0};
};

struct QrMotorCfg
{
    Mat34<MotorModel> motorModel;
    Mat34<bool> chiral;
    Mat34<double> ratio;
    Mat34<double> zeroPos;
    BootArgs boot;  // 启动参数

    QrMotorCfg()
    {
        motorModel.fill(MotorModel::haitai);
        chiral.fill(false);
        ratio.fill(6);
    }
};

class QrMotorMit : public QrMotor
{
public:
    QrMotorMit(const QrMotorCfg &cfg, std::unique_ptr<MitCanInterface> can);

    QrMotorMit(const QrMotorMit &) = delete;
    QrMotorMit &operator=(const QrMotorMit &) = delete;

    void Start() override;
    void Stop() override;
    void Run() override;
    void Enable() override;
    void Disable() override;

    RetState SetParam(const std::string &type, std::any param) override;
    RetState SetCallback(const std::string &type, QrCallbackFunc func) override;

private:
    bool BlockSend(const msg::qr::motor_cmd &_cmd);
    msg::qr::motor_ret BlockRecv();

    msg::qr::motor_ret RecvRaw();
    void MotorEnableCmd();
    void MotorDisableCmd();
    void MotorZeroCmd();
    bool BiasSave(const msg::qr::motor_ret &data);

    void ClearAllData();
    MotorState GetState();
    void SendRecvDeal();
    std::optional<Mat34<double>> SaveZeroPos(const msg::qr::motor_ret &set);
    void SpiTransfer(const SPI_TO_CAN_S &msg0, const SPI_TO_CAN_S &msg1);
    bool CheckEnable();

private:
    msg::qr::motor_ret motorRet_;
    msg::qr::motor_ret motorRawPrevData_;

    Mat34<uint8_t> state_;
    QrMotorCfg motorCfg_;
    std::unique_ptr<SpiToCan> spi2can;
    Mat34<std::unique_ptr<TransformMitMotor>> trans_;
    static MotorBias bias;
    std::unique_ptr<PeriodicMemberFunction<QrMotorMit>> task_;
    bool errorIsPrint_;
    std::map<int, std::string> legNameMap_;
    std::map<int, std::string> jointNameMap_;
    std::mutex mutex_;
    bool reqModifyZeroPos_{false};
    bool reqJointZeroWrite_{reqModifyZeroPos_};
    std::map<std::string, QrCallbackFunc> callbackFunc_;
};
