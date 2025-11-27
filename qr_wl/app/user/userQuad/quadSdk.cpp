
#include "quadSdk.hpp"

#include "deviceCustomParam.hpp"

using namespace std;

/**
 * @description: 四足的sdk业务包含了四足，设备，调试三种sdk接口
 * @param api
 * @return {}
 */
QuadSdk::QuadSdk(std::shared_ptr<ApiQuadruped> apiQuad, std::shared_ptr<ApiDevice> devApi)
{
    sdkParse_ = std::make_shared<SdkProtocolServer>();
    quad_ = make_unique<SdkQuadruped>(apiQuad, sdkParse_);
    dev_ = make_unique<SdkDevice>(devApi, sdkParse_);
    // ptr_dbg_初始化
    ptr_dbg_ = make_unique<SdkDebug>(sdkParse_, std::move(make_unique<QuadDebugServer>()));
}

void QuadSdk::Run()
{
    GetDevCustomParam().sdkInput_.TryGetRequest();

    auto ret = MsgTryRecv<bool>("bs::gamepadFind", this);
    if (ret) {
        quad_->SetKeyIsExist(ret.value());  // 手柄连接仅会导致sdkQuad中的部分API无效
    }
}