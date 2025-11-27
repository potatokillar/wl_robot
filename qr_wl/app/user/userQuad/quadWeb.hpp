
#pragma once
#include <memory>
#include <thread>

#include "apiDevice.hpp"
#include "apiQuadruped.hpp"

class QuadWeb
{
public:
    QuadWeb(std::shared_ptr<ApiQuadruped> apiQuad, std::shared_ptr<ApiDevice> devApi);

private:
    std::thread thread_;
    void Run();
    std::shared_ptr<ApiQuadruped> apiQuad_;
    std::shared_ptr<ApiDevice> apiDev_;
};