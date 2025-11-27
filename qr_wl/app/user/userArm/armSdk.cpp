
#include "armSdk.hpp"

#include "deviceCustomParam.hpp"

using namespace std;

ArmSdk::ArmSdk(std::shared_ptr<ApiArm> armApi, std::shared_ptr<ApiDevice> devApi)
{
    sdkParse_ = std::make_shared<SdkProtocolServer>();
    arm_ = make_unique<SdkArm>(armApi, sdkParse_);
    dev_ = make_unique<SdkDevice>(devApi, sdkParse_);
    ptr_dbg_ = make_unique<SdkDebug>(sdkParse_, std::move(make_unique<ArmDebugServer>()));
}

void ArmSdk::Run() { GetDevCustomParam().sdkInput_.TryGetRequest(); }