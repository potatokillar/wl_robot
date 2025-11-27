
#include "quadDataChange.hpp"

#include "baseline.hpp"
#include "canBridge.hpp"
#include "robotMedia.hpp"

QuadDataChange::QuadDataChange(std::shared_ptr<ApiQuadruped> api, std::shared_ptr<ApiDevice> devApi) : api_(api), devApi_(devApi)
{
    // 1 min上报一次
    SoftTimerSet[0].AddFunc("lowBattery", 60000, [this]() { this->LowBatteryWarn(); });
}

/**
 * @description: 具有实时性的监控要求，写在此处
 * @return {}
 */
void QuadDataChange::Run()
{
    {
        auto ret = MsgTryRecv<bool>("qr::imu_error", this);
        if (ret && (ret.value() == true)) {
            api_->SetRunState(RunState::fall);  // 进入阻尼模式
        }
    }
}

void QuadDataChange::LowBatteryWarn()
{
    u16 msg;
    msg = devApi_->GetQuantity();
    MediaTaskPackage media_pkg;

    if (msg <= 25) {
        media_pkg.Add("lowBattery");
    } else if (msg <= 10) {
        media_pkg.Add("lowBatteryToShutDown");
    }
    AddMediaPackage(media_pkg);
}