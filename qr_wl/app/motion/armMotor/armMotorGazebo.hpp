
#pragma once
#include "armMotor.hpp"

class ArmMotorGazebo : public ArmMotor
{
public:
    ArmMotorGazebo();
    ~ArmMotorGazebo() override;
    void Start() override;
    void Stop() override;
    void Enable() override;
    void Disable() override;
    void SetIoOut(const std::string& name, std::bitset<32> set) override;
    std::bitset<32> GetIoOut(const std::string& name) override;

private:
    void Run();
    void SendMotorCmd();
    void RxArmCmd();
    void RecvMotorRet();

private:
    std::unique_ptr<PeriodicMemberFunction<ArmMotorGazebo>> task_;
};