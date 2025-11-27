
#pragma once

#include "apiArm.hpp"
#include "armDebugServer.hpp"
#include "sdkArm.hpp"
#include "sdkDebug.hpp"
#include "sdkDevice.hpp"

class ArmSdk
{
public:
    ArmSdk(std::shared_ptr<ApiArm> armApi, std::shared_ptr<ApiDevice> devApi);

    void Run();

private:
    std::shared_ptr<SdkProtocolServer> sdkParse_;

    std::unique_ptr<SdkDevice> dev_;
    std::unique_ptr<SdkDebug> ptr_dbg_;
    std::unique_ptr<SdkArm> arm_;
};
