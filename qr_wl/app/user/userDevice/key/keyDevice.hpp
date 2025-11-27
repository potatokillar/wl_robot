
#pragma once
#include "apiDevice.hpp"
#include "keyBase.hpp"

class KeyDevice : public KeyBase
{
public:
    KeyDevice(std::shared_ptr<ApiDevice> api);
    void Run() override;

private:
    std::shared_ptr<ApiDevice> api_;
    u64 shutdownStartTime_ = 0;  // 0表示未启动计时，非0表示启动关机检测
};