
#pragma once

#if 0
#include <librealsense2/rs.hpp>

#include "baseline.hpp"

class RealSenseNode : public Singleton<RealSenseNode>
{
public:
    RealSenseNode();
    ~RealSenseNode() override;

private:
    void Loop() override;
    rs2::pipeline pipe;
};

inline RealSenseNode& GetRealSenseNode() { return RealSenseNode::GetInstance(); }
#endif