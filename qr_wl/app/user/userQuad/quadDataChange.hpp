

#pragma once
#include <memory>

#include "apiDevice.hpp"
#include "apiQuadruped.hpp"

class QuadDataChange
{
public:
    QuadDataChange(std::shared_ptr<ApiQuadruped> ctrl, std::shared_ptr<ApiDevice> devApi);
    void Run();
    void LowBatteryWarn();

private:
    std::shared_ptr<ApiQuadruped> api_;
    std::shared_ptr<ApiDevice> devApi_;
};