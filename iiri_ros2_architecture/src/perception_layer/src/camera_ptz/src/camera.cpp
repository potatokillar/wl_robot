/*
 * @Author: 唐文浩
 * @Date: 2025-02-11
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-10
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "camera.hpp"

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <boost/process.hpp>
#include <fstream>
#include <std_srvs/srv/empty.hpp>
#include <string>

#include "robot_base/timer_tools.hpp"

using namespace std;

Camera::Camera() : Node("camera_node")
{
    RCLCPP_INFO(this->get_logger(), "camera node init");

    this->declare_parameter<std::string>("pwd", "");
    this->declare_parameter<std::string>("ip", "");
    this->declare_parameter<std::string>("user", "");
    this->declare_parameter<std::string>("camera_type", "");

    std::string ip = this->get_parameter("ip").as_string();
    std::string pwd = this->get_parameter("pwd").as_string();
    std::string user = this->get_parameter("user").as_string();
    std::string camera_type = this->get_parameter("camera_type").as_string();

    if (camera_type == "realsense")
    {
        running_ = true;
        RCLCPP_INFO(this->get_logger(), "Starting realsense camera");
        thread_ = std::thread([this]()
                              { this->realsense_loop(); });
    }
    else if (camera_type == "dahua")
    {
        RCLCPP_INFO(this->get_logger(), "Starting dahua camera");
        if (ip.empty())
        {
            sdk_dahua_ = std::make_unique<SdkDahua>(user, pwd);

            // 扫描ip，每3秒取一次结果
            RCLCPP_INFO(this->get_logger(), "no camera ip, start search");
            sdk_dahua_->start_search();
            timer_search_ = this->create_wall_timer(std::chrono::seconds(3), [this]()
                                                    { this->get_ip_list(); });
        }
        else
        {
            sdk_dahua_ = std::make_unique<SdkDahua>(ip, user, pwd);

            // 当本节点比摄像机启动早时，可能登录失败，因此需要循环登录
            timer_login_ = this->create_wall_timer(std::chrono::seconds(10), [this]()
                                                   { this->login(); });
            login(); // 立刻回调一次，ros2的定时器并不会在启动时立刻回调
        }
    }

    // 必须等待websocket 服务器启动后才能启动
    auto cli_ws_ready = this->create_client<std_srvs::srv::Empty>("/dev_server/ready");
    cli_ws_ready->wait_for_service();

    cli_my_info_ = create_client<interface::srv::MyInfo>("/robot_base/my_info");
    cli_my_info_->wait_for_service();
    cli_my_info_->async_send_request(std::make_shared<interface::srv::MyInfo::Request>(), [this](rclcpp::Client<interface::srv::MyInfo>::SharedFuture future)
                                     {
        auto response = future.get();
        this->rtsp_ip_ = response->addr[3];  // 只取第一个ip
        uuid_ = response->uuid;
        RCLCPP_INFO(this->get_logger(), "rtsp addr: %s uuid:%s", response->addr[0].c_str(), uuid_.c_str()); });

    RCLCPP_INFO(this->get_logger(), "camera_node init complete");
}

Camera::~Camera()
{
    running_ = false;
    timer_search_->cancel();
    timer_login_->cancel();
    if (thread_.joinable())
    {
        thread_.join();
    }
}

/**
 * @description: 搜索到IP后立刻停止搜索
 * @return {}
 */
void Camera::get_ip_list()
{
    auto ipset = sdk_dahua_->get_search_result();
    if (ipset.empty() == false)
    {
        sdk_dahua_->stop_search();
        // 这里只取第一个，暂不支持多摄像头
        sdk_dahua_->set_ip(*ipset.cbegin());
        timer_login_ = this->create_wall_timer(std::chrono::seconds(10), [this]()
                                               { this->login(); });
        login();

        timer_search_->cancel();
    }
}

/**
 * @description: 推流
 * @return {}
 */
void Camera::rtsp_loop()
{
    // 等待ip和uuid获取到
    while (running_)
    {
        if (rtsp_ip_.empty() || uuid_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "rtsp_ip or uuid is empty, wait next scan. ip: %d, uuid: %d", rtsp_ip_.empty(), uuid_.empty());
            TimerTools::SleepForS(1);
        }
        else
        {
            break;
        }
    }

    ws_client_ = std::make_shared<WebSocketClient>("127.0.0.1", uuid_, paths_);
    ws_client_->async_recv(paths_[2], [this](const std::vector<uint8_t> &frame)
                           { this->save_user_audio_frame(frame); });
    ws_client_->async_recv(paths_[3], [this](const std::string &payload)
                           { this->rx_set_talk(payload); });
    ws_client_->async_recv(paths_[4], [this](const std::string &payload)
                           { this->rx_ptz(payload); });
    ws_thread_ = std::thread([this]()
                             { ws_client_->loop(); });

    int count = 0;
    std::vector<uint8_t> data;
    while (running_)
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (!h264_queue.empty())
        {
            data = h264_queue.front();
            h264_queue.pop();
            RCLCPP_INFO_ONCE(this->get_logger(), "send video frame");
            ws_client_->send(paths_[0], data); // 第一个是视频流

            if (h264_queue.size() > 50)
            {
                RCLCPP_WARN(this->get_logger(), "h264 queue is too long, size: %d", h264_queue.size());
            }
        }

        if (!audio_queue_.empty())
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "send audio frame");
            data = audio_queue_.front();
            audio_queue_.pop();
            ws_client_->send(paths_[1], data);

            if (audio_queue_.size() > 50)
            {
                RCLCPP_WARN(this->get_logger(), "audio queue is too long, size: %d", audio_queue_.size());
            }
        }

        // 一次处理多个数据，否则会卡卡的
        if (user_audio_queue_.size() > 20)
        {
#if 0
            // 保存到文件
            static std::ofstream aacFile("output.pcm", std::ios::binary | std::ios::app | ios::out);
            if (!aacFile.is_open()) {
                std::cerr << "Error: Unable to open file for writing." << std::endl;
                return;
            }
#endif

            RCLCPP_INFO_ONCE(this->get_logger(), "recv audio frame");
            while (!user_audio_queue_.empty())
            {
                data = user_audio_queue_.front();
                user_audio_queue_.pop();
                sdk_dahua_->start_talk(); // 同时也支持手动开启
                sdk_dahua_->send_audio(data);
                // 写入音频帧
                // aacFile.write(reinterpret_cast<const char *>(data.data()), data.size());
            }
            // ffplay -f s16le -ar 8000 -ac 1 -i audio.pcm
            //  aacFile.close();
        }
#if 1
        // 较长时间内没有收到音频，停止音频，似乎未成功关闭会导致新的对讲失效？
        if (TimerTools::GetNowTickS() - lastRxAudio_ > 10)
        {
            sdk_dahua_->stop_talk();
        }
#endif
    }

    dhav2wav_.deinit();
}

/**
 * @description: 登录成功才开启相关功能
 * @return {}
 */
void Camera::login()
{
    if (sdk_dahua_->login())
    {
        sdk_dahua_->set_video_callback([this](const std::vector<uint8_t> &frame)
                                       { this->save_h264_frame(frame); });
        sdk_dahua_->set_audio_callback([this](const std::vector<uint8_t> &frame)
                                       { this->save_audio_frame(frame); });
        running_ = true;
        thread_ = std::thread([this]()
                              { this->rtsp_loop(); });
        timer_login_->cancel();
    }
}

void Camera::save_h264_frame(const std::vector<uint8_t> &frame)
{
    std::lock_guard<std::mutex> lock(queue_mutex);
    h264_queue.push(frame);
}

void Camera::save_audio_frame(const std::vector<uint8_t> &frame)
{
    std::lock_guard<std::mutex> lock(queue_mutex);
    audio_queue_.push(frame);
}

void Camera::rx_ptz(const string &payload)
{
    try
    {
        nlohmann::json json = nlohmann::json::parse(payload);
        double vx = Json2Double(json.at("param").at("vx"));
        double vy = Json2Double(json.at("param").at("vy"));
        RCLCPP_INFO(this->get_logger(), "ptz: %f, %f", vx, vy);
        sdk_dahua_->set_ptz(vx, vy);
    }
    catch (nlohmann::json::exception &e)
    {
        RCLCPP_WARN(this->get_logger(), "ptz json error: %s", e.what());
    }
};

void Camera::save_user_audio_frame(const std::vector<uint8_t> &frame)
{
    std::lock_guard<std::mutex> lock(queue_mutex);
    user_audio_queue_.push(frame);
    lastRxAudio_ = TimerTools::GetNowTickS();
}

void Camera::rx_set_talk(const std::string &payload)
{
    try
    {
        nlohmann::json json = nlohmann::json::parse(payload);
        bool set = json.at("param").at("open");
        RCLCPP_INFO(this->get_logger(), "on_set_talk: %d", set);
        lastRxAudio_ = TimerTools::GetNowTickS();
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (set)
        {
            sdk_dahua_->start_talk();
        }
        else
        {
            sdk_dahua_->stop_talk();
        }
    }
    catch (std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "on_set_talk error: %s", e.what());
    }
}

void Camera::realsense_loop()
{
    // 等待ip和uuid获取到
    while (running_)
    {
        if (rtsp_ip_.empty() || uuid_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "rtsp_ip or uuid is empty, wait next scan. ip: %d, uuid: %d", rtsp_ip_.empty(), uuid_.empty());
            TimerTools::SleepForS(1);
        }
        else
        {
            break;
        }
    }

    ws_client_ = std::make_shared<WebSocketClient>("127.0.0.1", uuid_, paths_);
    ws_thread_ = std::thread([this]()
                             { ws_client_->loop(); });

    RealsenseStart();
    while (running_)
    {
        auto data = GetRealsenseRGB();
        RCLCPP_INFO_ONCE(this->get_logger(), "send video frame");
        ws_client_->send(paths_[0], data); // 第一个是视频流
    }

    RealsenseStop();
}