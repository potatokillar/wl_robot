/*
 * @Author: 唐文浩
 * @Date: 2025-04-21
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-24
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <dirent.h>
#include <opus/opus.h>
#include <speex/speex_resampler.h> // 重采样头文件

#include <algorithm>
#include <atomic>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_client.hpp>

#include "atomicops.h"
#include "global.hpp"
#include "http.h"
#include "interface/msg/audio_data_package.hpp"
#include "json.hpp"
#include "opus.h"
#include "rclcpp/rclcpp.hpp"
#include "readerwriterqueue.h"
#include "std_msgs/msg/u_int16.hpp"
#include "websocket_client.h"

typedef enum ListeningMode
{
    kListeningModeAutoStop,
    kListeningModeManualStop,
    kListeningModeAlwaysOn // 需要 AEC 支持
} ListeningMode;

// 定义设备状态枚举类型
typedef enum DeviceState
{
    kDeviceStateUnknown,
    kDeviceStateStarting,
    kDeviceStateWifiConfiguring,
    kDeviceStateIdle,
    kDeviceStateConnecting,
    kDeviceStateListening,
    kDeviceStateSpeaking,
    kDeviceStateUpgrading,
    kDeviceStateActivating,
    kDeviceStateFatalError
} DeviceState;

typedef struct opus_encoder
{
    unsigned int inputSampleRate;
    unsigned int inputChannels;
    unsigned int outputSampleRate;
    unsigned int outputChannels;
    unsigned int duration_ms;
    SpeexResamplerState *resampler;
    OpusEncoder *encoder;
} opus_encoder;

typedef struct opus_decoder
{
    int inputSampleRate;
    int inputChannels;
    int outputSampleRate;
    int outputChannels;
    int duration_ms;
    SpeexResamplerState *resampler;
    OpusDecoder *decoder;
} opus_decoder;

class AudioDataDismissTimer
{
public:
    AudioDataDismissTimer();
    ~AudioDataDismissTimer();
    void Open();
    void Close();
    void TimerThread();
    void Activate();

    size_t duration_ = 30;                  // 计时器上限，单位为 s
    std::atomic<size_t> now_ = {duration_}; // 当前剩余时长，单位为 s
    std::thread timer_;                     // 计时器线程
    std::atomic<bool> valid_ = {false};     // 工作标志位
    std::mutex mtx_;                        // 互斥量
};

class XiaoZhiNode : public rclcpp::Node
{
public:
    XiaoZhiNode();
    void LoopXiaoZhiWork();
    void SendSpeakerWavFile(std::string str);
    void SendIotCmd(std::string str);

    void CallbackRecord(const interface::msg::AudioDataPackage::SharedPtr msg);
    // void CallbackXiaoZhiCmd(const std_msgs::msg::String::SharedPtr msg);
    void CallbackCurrentVolume(const std_msgs::msg::UInt16::SharedPtr msg);
    void LoopSendToSpeaker();
    void LoopSendToASR();

    void ParseIotJson(nlohmann::json j);

private:
    // topic通信接口
    // 发布端
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _xiaozhi_stt_pub_ptr;           // xiaozhi服务器的语音识别结果
    rclcpp::Publisher<interface::msg::AudioDataPackage>::SharedPtr _audio_data_pub_ptr; // 发布音频数据
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _wav_file_pub_ptr;              // 发送要播放的wav文件的文件名
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _iot_cmd_pub_ptr;               // 大模型给出的设备控制指令的内容
    // 接收端
    rclcpp::Subscription<interface::msg::AudioDataPackage>::SharedPtr _audio_data_sub_ptr; // 接收record节点音频数据
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _xiaozhi_cmd_sub_ptr;            // 接收控制xiaozhi大模型开启关闭的指令
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr _current_volume_sub_ptr; // 接收当前音量

    std::thread _thread_xiaozhi;
    std::thread _thread_publish_audio_data;
    std::thread _thread_publish_stt;
};

// 声明全局节点智能指针
extern std::shared_ptr<XiaoZhiNode> g_node_ptr;
