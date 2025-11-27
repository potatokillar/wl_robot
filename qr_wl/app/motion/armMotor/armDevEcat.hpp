
#pragma once

#include <future>
#include <variant>

#include "armMotor.hpp"
#include "baseline.hpp"
#include "ecatIO.hpp"
#include "ecatMotor.hpp"

using EcatDevType = std::variant<EcatMotor, EcatIO>;
class ArmDevEcat : public ArmMotor
{
public:
    ArmDevEcat(const std::string& ethName);
    void Start() override;
    void Stop() override;
    void Enable() override;
    void Disable() override;
    void SetIoOut(const std::string& name, std::bitset<32> set) override;
    std::bitset<32> GetIoOut(const std::string& name) override;
    std::bitset<32> GetIoIn(const std::string& name) override;

private:
    void PreOpInit();
    void OpInit();
    void PdoSync();
    void ToOperational();
    void ToPreOperational();
    void ToSafeOperational();
    void Init();
    void Run();

    void AsyncEnable();
    void AsyncDisable();
    void SendMotorCmd();
    void RecvMotorRet();

    void UpdateStatusWord();
    void UpdateIoInState();
    void RxArmCmd();

private:
    std::vector<EcatDevType> devs_;
    std::unique_ptr<PeriodicMemberFunction<ArmDevEcat>> task_;

    std::future<void> asyncFunc_;
    std::vector<RpcRecv> rpcRxDeal_;  // rpc接收处理
};