
#include "armKey.hpp"

using namespace std;

ArmKey::ArmKey(std::shared_ptr<ApiArm> api, std::shared_ptr<ApiDevice> devApi)
{
    keyArm_ = make_unique<KeyArm>(api);
    keyDev_ = make_unique<KeyDevice>(devApi);
}

/**
 * @description: 按键逻辑处理函数
 * @return {}
 */
void ArmKey::Run()
{
    keyArm_->Run();
    keyDev_->Run();
}
