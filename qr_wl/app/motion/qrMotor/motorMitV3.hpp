
#pragma once
#include "qrMotor.hpp"

class MotorMitV3 : public QrMotor
{
public:
    MotorMitV3(int leg_num, int motor_num, std::unique_ptr<SpiToCanV3> can);

protected:
    // 定义腿和关节的数量
    const int LEG_NUM;
    const int MOTOR_NUM;

    void SendEnable();
    void SendDisable();
    void SendZero();

    void CheckInit(const std::string& dbTag, const std::vector<std::vector<double>>& data);

    void SaveInitPos2Database(const std::string& tag, const std::vector<std::vector<double>>& pos);
    std::optional<std::vector<std::vector<double>>> GetInitPosFromDatabase(const std::string& tag);

    void ClearSpiData();
    void SpiTransfer(const std::array<std::vector<CanData>, 4>& can);
    void CheckMotorState(const std::vector<std::vector<bool>>& ret);

    std::vector<std::vector<QrMotorInfo>> infos_;
    std::unique_ptr<SpiToCanV3> spi2can;  // 不建议使用shared_ptr，因为该设备是唯一的，且只能由本类管理

private:
    bool biasIsUpdate_{false};
};
