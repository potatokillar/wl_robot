
#pragma once
#include "apiDevice.hpp"
#include "apiQuadruped.hpp"
#include "quadDebugServer.hpp"
#include "sdkDebug.hpp"
#include "sdkDevice.hpp"
#include "sdkProtocolServer.hpp"
#include "sdkQuadruped.hpp"

class QuadSdk
{
public:
    QuadSdk(std::shared_ptr<ApiQuadruped> apiQuad, std::shared_ptr<ApiDevice> devApi);

    void Run();

private:
    std::shared_ptr<SdkProtocolServer> sdkParse_;

    std::unique_ptr<SdkQuadruped> quad_;
    std::unique_ptr<SdkDevice> dev_;
    std::unique_ptr<SdkDebug> ptr_dbg_;
};
