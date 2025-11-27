/*
 * @Author: 唐文浩
 * @Date: 2024-10-17
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-19
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <std_msgs/msg/string.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "interface/msg/audio_data_package.hpp"
#include "interface/msg/gamepad_cmd.hpp"
#include "interface/msg/run_state.hpp"
#include "interface/srv/dance.hpp"
#include "interface/srv/dog_move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"

// 自定义哈希函数对象
// 这个hash是用于为CommandText类型生成哈希值的
struct SetHash
{
    std::size_t operator()(const std::set<std::string> &set) const noexcept
    {
        std::size_t hash = 0;
        for (const auto &elem : set)
        {
            // 使用标准库提供的 hash 函数处理每个字符串，并将结果合并到最终的哈希值中
            // 这里使用了一个常见的哈希组合方法，即通过 XOR 和位移来组合多个哈希值
            hash ^= std::hash<std::string>{}(elem) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};

using CommandText = std::set<std::string>;                                   // 指令的文字内容
using CommandContent = std::function<void()>;                                // 指令的具体内容，函数对象
using ASRCommand = std::unordered_map<CommandText, CommandContent, SetHash>; // 指令的映射关系
struct DogMoveStruct;                                                        // DogMove service的传参结构体

#define WAVPLAY_ENABLE 0
#if WAVPLAY_ENABLE
using MediaIndexText = std::string;                                 // 媒体文件的中文索引
using MediaFilePath = std::string;                                  // 媒体文件的相对路径
using MediaMap = std::unordered_map<MediaIndexText, MediaFilePath>; // 媒体文件映射关系

// 定义一个 constexpr 函数来返回完整的路径前缀
constexpr std::string_view get_full_media_path()
{
#ifdef __x86_64__
    return "build_x86/install/speech_recognition/share/speech_recognition/audio/";
#else
    return "install/speech_recognition/share/speech_recognition/audio/";
#endif
}

const MediaMap media_file_map = {{"happy_new_year", std::string(get_full_media_path()) + "happy_new_year.wav"},
                                 {"huan_le_tiao", std::string(get_full_media_path()) + "huan_le_tiao.wav"},
                                 {"i_am_here", std::string(get_full_media_path()) + "i_am_here.wav"},
                                 {"lie_down", std::string(get_full_media_path()) + "lie_down.wav"},
                                 {"move_backward", std::string(get_full_media_path()) + "move_backward.wav"},
                                 {"move_forward", std::string(get_full_media_path()) + "move_forward.wav"},
                                 {"move_left", std::string(get_full_media_path()) + "move_left.wav"},
                                 {"move_right", std::string(get_full_media_path()) + "move_right.wav"},
                                 {"quit", std::string(get_full_media_path()) + "quit.wav"},
                                 {"shake_hand", std::string(get_full_media_path()) + "shake_hand.wav"},
                                 {"speech_node_close", std::string(get_full_media_path()) + "speech_node_close.wav"},
                                 {"speech_node_open", std::string(get_full_media_path()) + "speech_node_open.wav"},
                                 {"stand_up", std::string(get_full_media_path()) + "stand_up.wav"},
                                 {"tai_kong_bu", std::string(get_full_media_path()) + "tai_kong_bu.wav"},
                                 {"turn_left", std::string(get_full_media_path()) + "turn_left.wav"},
                                 {"turn_right", std::string(get_full_media_path()) + "turn_right.wav"},
                                 {"please_repeat", std::string(get_full_media_path()) + "please_repeat.wav"},
                                 {"xiang_qian_tiao", std::string(get_full_media_path()) + "xiang_qian_tiao.wav"},
                                 {"attention", std::string(get_full_media_path()) + "bo.wav"},
                                 {"open_asr", std::string(get_full_media_path()) + "ding.wav"}};

// 播放1个wav文件的类
// 使用RAII形式，在构造函数中传入想要播放的文件的索引，就能阻塞式播放
class WavPlay
{
public:
    WavPlay(const std::string &text_to_search);

private:
    void Play();
    void ShellPlay();

    MediaIndexText index_text_;
    MediaFilePath file_;
};
#endif

/** @brief 计时器类 */
class AliveTimer
{
public:
    AliveTimer();
    ~AliveTimer();

    void Open();
    void Close();
    void Running();
    bool Check();
    void Reset();

private:
    std::atomic<int> now_ = {duration_};
    std::atomic<bool> valid_ = {false};
    int duration_ = 5; // 倒计时最大时长
    std::thread timer_;
};

// 负责在ROS2架构中进行通信的类，继承自ROS2节点类
class speech_recognition : public rclcpp::Node
{
public:
    speech_recognition();
    ~speech_recognition();

    std::set<std::string> GetCommandTextSet();

private:
    void AddSpeechCmd(std::initializer_list<std::string> list, CommandContent func);
    void AddWakeAndSpeechCmd(std::initializer_list<std::string> list, CommandContent func);

    void SayDogMoveTrigger(DogMoveStruct dog_move_cmd, int time);
    void SayStandUpTrigger();
    void SayLieDownTrigger();
    void SayDanceActionTrigger(const std::string &dance_name);
    void SayBeware();
    void SayEnableXiaozhi(bool cmd);
    void SaySmartFollowTrigger(bool cmd);

    void Loop();
    void TxDeepseekMsgLoop();
    void SendSpeakerWavFile(std::string str);

    // 回调函数
    void CallbackTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void CallbackRunState(const interface::msg::RunState::SharedPtr msg);
    void CallbackASRControl(const std_msgs::msg::String::SharedPtr msg);
    void CallbackRecordAudioData(const interface::msg::AudioDataPackage::SharedPtr msg);
    void CallbackXiaoZhiSTT(const std_msgs::msg::String::SharedPtr msg);
    void CallbackGamepad(const interface::msg::GamepadCmd::SharedPtr msg);

    // topic通信接口
    // 发布端
    rclcpp::Publisher<interface::msg::RunState>::SharedPtr _run_state_pub_ptr; // 发布RunState，起立，趴下
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_pub_ptr;      // 发布速度信息
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _deepseek_msg_pub_ptr; // 发布识别结果
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _tts_content_pub_ptr;  // 发送文字转语音的内容
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _wav_file_pub_ptr;     // 发送要播放的wav文件的文件名
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _xiaozhi_cmd_pub_ptr;  // 发送控制xiaozhi大模型开启关闭的指令
    // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _smart_follow_pub_ptr;    // 发布智能跟随指令
    // 接收端
    // rclcpp::Subscription<interface::msg::RunState>::SharedPtr _run_state_sub_ptr;           // 接收RunState
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _asr_control_sub_ptr;           // 接收tts_server_enable
    rclcpp::Subscription<interface::msg::AudioDataPackage>::SharedPtr _audio_data_sub_ptr; // 接收record节点音频数据
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _xiaozhi_stt_sub_ptr;           // 接收xiaozhi服务器的语音识别结果
    rclcpp::Subscription<interface::msg::GamepadCmd>::SharedPtr _gamepad_sub_ptr;          // 接收手柄数据的消息

    // 客户端回调组
    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::Client<interface::srv::DogMove>::SharedPtr _dog_move_cli_ptr;
    rclcpp::Client<interface::srv::Dance>::SharedPtr _dance_cli_ptr;

    // 服务端回调组
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _trigger_srv_ptr; // 行为树的trigger信号

    const std::string cmd_head_ = "小黑小黑";
    ASRCommand _cmd_map; // 语音指令映射表
    std::thread _thread;
    std::thread _send_message_thread;
    std::thread _cache_thread;
};