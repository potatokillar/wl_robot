/*
 * @Author: 唐文浩
 * @Date: 2025-02-11
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-04
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <vector>
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswresample/swresample.h>
}

#include "dhav2wav.hpp"
#include "interface/msg/camera_ptz_move.hpp"
#include "interface/srv/my_info.hpp"
#include "realsense.hpp"
#include "robot_base/ws_client.hpp"
#include "sdk_dahua.hpp"

class Camera : public rclcpp::Node
{
public:
    Camera();
    virtual ~Camera();
    void login();
    void save_h264_frame(const std::vector<uint8_t> &frame);
    void save_audio_frame(const std::vector<uint8_t> &frame);
    void save_user_audio_frame(const std::vector<uint8_t> &frame);

private:
    void rtsp_loop();
    void realsense_loop();
    void get_ip_list();
    void rx_ptz(const std::string &payload);
    void rx_set_talk(const std::string &payload);

    std::thread thread_; // 用于音视频发送
    bool running_{false};
    std::string camera_addr_;
    std::string rtsp_addr_;
    std::vector<std::string> ip_list_;
    rclcpp::TimerBase::SharedPtr timer_login_, timer_search_; // 用于登录和扫描相机

    std::queue<std::vector<uint8_t>> h264_queue, audio_queue_, user_audio_queue_; // 数据队列
    std::mutex queue_mutex;
    std::unique_ptr<SdkDahua> sdk_dahua_;

    rclcpp::Subscription<interface::msg::CameraPTZMove>::SharedPtr sub_ptz_;
    rclcpp::Client<interface::srv::MyInfo>::SharedPtr cli_my_info_;
    std::string uuid_;
    std::string rtsp_ip_;

    std::shared_ptr<WebSocketClient> ws_client_;
    std::vector<std::string> paths_{"/robot/video", "/robot/audio", "/user/audio", "/device/set_talk", "/device/set_camera_ptz"};
    std::thread ws_thread_;

    Dhav2Wav dhav2wav_;
    u64 lastRxAudio_; // 上一次收到用户音频的时间
};