/*
 * @Author: 唐文浩
 * @Date: 2025-02-26
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-05-09
 * @Description: 大华sdk，单个摄像头，仅和摄像头有关
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include "dahua/dhnetsdk.h"
#include "robot_base/cpp_types.hpp"

class SdkDahuaImpl;
class SdkDahua
{
public:
    SdkDahua(const std::string &ip, const std::string &user, const std::string &pwd);
    SdkDahua(const std::string &user, const std::string &pwd);
    virtual ~SdkDahua();

    void set_ip(const std::string &ip);
    void set_ptz(double vx, double vy);
    void set_video_callback(std::function<void(const std::vector<uint8_t> &)> func);
    void set_audio_callback(std::function<void(const std::vector<uint8_t> &)> func);
    void send_audio(const std::vector<uint8_t> &data);

    bool login();
    void start_search();
    void stop_search();
    std::set<std::string> get_search_result();

    bool start_talk();
    bool stop_talk();

private:
    std::unique_ptr<SdkDahuaImpl> impl_;
};