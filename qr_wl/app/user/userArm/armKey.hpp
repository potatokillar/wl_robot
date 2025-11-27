
#pragma once
#include "apiArm.hpp"
#include "apiArmCtrl.hpp"
#include "keyArm.hpp"
#include "keyDevice.hpp"

class ArmKey
{
public:
    ArmKey(std::shared_ptr<ApiArm> api, std::shared_ptr<ApiDevice> devApi);
    void Run();

private:
    std::unique_ptr<KeyArm> keyArm_;
    std::unique_ptr<KeyDevice> keyDev_;
};