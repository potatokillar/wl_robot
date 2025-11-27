
#include "realsense.hpp"

#include "baseline.hpp"

#if 0
RealSenseNode::RealSenseNode()
{
    thread_ = std::thread(&RealSenseNode::Loop, this);

    LOG_INFO("realsense init");
}
RealSenseNode::~RealSenseNode() { pipe.stop(); }

void RealSenseNode::Loop()
{
    try {
        pipe.start();
        MsgSend msgRealSense{"realsense"};
        while (1) {
            rs2::frameset frames = pipe.wait_for_frames();  // 阻塞等待下一个流到来，实测就是下一个帧，设备是30帧。阻塞约30ms
            rs2::depth_frame depth = frames.get_depth_frame();  // 获取一个深度信息
            double width = depth.get_width();                    // 获取深度信息的长宽
            double height = depth.get_height();

            // 获取帧中心的距离
            double dist_to_center = depth.get_distance(width / 2, height / 2);
            msgRealSense.Send(dist_to_center);
            std::cout << "The camera is facing an object " << dist_to_center << " meters away" << std::endl;
        }
    } catch (std::exception& e) {
        LOG_ERROR("realsense error");
    }
}

#endif