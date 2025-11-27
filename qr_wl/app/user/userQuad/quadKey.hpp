
#pragma once

#include <memory>

#include "apiQuadruped.hpp"
#include "keyDevice.hpp"
#include "keyQuadruped.hpp"

class QuadKey
{
public:
    QuadKey(std::shared_ptr<ApiQuadruped> api, std::shared_ptr<ApiDevice> devApi);
    void Run();

private:
    // 四足按键业务只支持四足按键接口和设备按键接口
    std::unique_ptr<KeyQuadruped> keyQuad_;
    std::unique_ptr<KeyDevice> keyDev_;
};