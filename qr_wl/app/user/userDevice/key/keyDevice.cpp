
#include "keyDevice.hpp"

KeyDevice::KeyDevice(std::shared_ptr<ApiDevice> api) : api_(api) {}

void KeyDevice::Run()
{
    UpdateGamepadKey();

    if (btn_.at("back").IsTriggerPress()) {
        if (shutdownStartTime_ == 0) {
            shutdownStartTime_ = TimerTools::GetNowTickMs();
        }
    }

    if ((shutdownStartTime_ != 0) && (btn_.at("back").IsPress() == false)) {
        shutdownStartTime_ = 0;
    }

    // 关机计时
    if (shutdownStartTime_ != 0) {
        if (TimerTools::GetNowTickMs() - shutdownStartTime_ > 3000) {
            api_->Shutdown();
        }
    }
}
