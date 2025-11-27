
#pragma once
#include "apiArm.hpp"
#include "gripper.hpp"
#include "keyBase.hpp"

class KeyArm : public KeyBase
{
public:
    KeyArm(std::shared_ptr<ApiArm> api);
    void Run();
    void BlockRun();

private:
    void ArmKey();
    void GripperKey();
    void ArmEnableKey();
    void ArmDisableKey();
    void SetArmResetKey();
    void ChangerateKey();

private:
    std::thread thread_;
    std::shared_ptr<ApiArm> api_;
    bool running_{true};
    bool Reset_ = false;
    // ControlGripperAPI gripper_;
};