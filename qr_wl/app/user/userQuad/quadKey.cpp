
#include "quadKey.hpp"

using namespace std;

QuadKey::QuadKey(std::shared_ptr<ApiQuadruped> api, std::shared_ptr<ApiDevice> devApi)
{
    keyQuad_ = make_unique<KeyQuadruped>(api);
    keyDev_ = make_unique<KeyDevice>(devApi);
}

/**
 * @description: 按键逻辑处理函数
 * @return {}
 */
void QuadKey::Run()
{
    keyQuad_->Run();
    keyDev_->Run();
}
