
#pragma once
#include "apiQuadruped.hpp"
#include "baseline.hpp"
#include "keyBase.hpp"

class KeyQuadruped : public KeyBase
{
public:
    KeyQuadruped(std::shared_ptr<ApiQuadruped> api);
    void Run() override;
    void BlockRun();

private:
    void QuadKey();
    void RunLongPress();
    void RunMultiPress();
    void ReleaseKey();
    bool poseControlFlag_ = false;
    std::thread thread_;
    int tmpDance_ = 0;
    std::shared_ptr<ApiQuadruped> api_;
    bool load_min_reached = false;
    bool load_max_reached = false;
};