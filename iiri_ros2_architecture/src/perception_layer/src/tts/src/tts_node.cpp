/*
 * @Author: 唐文浩
 * @Date: 2025-03-04
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-17
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <alsa/asoundlib.h>

#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>

#include "SynthesizerTrn.h"
#include "header/Hanz2Piny.h"
#include "header/hanzi2phoneid.h"
#include "interface/msg/audio_data_package.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "stdio.h"
#include "string"
#include "string.h"
#include "utils.h"

// 宽字符字符串转普通字符串
std::string WideString2String(std::wstring str_wide)
{
    std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
    std::string str_narrow = converter.to_bytes(str_wide);
    return str_narrow;
};

// 普通字符串转宽字符字符串
std::wstring String2WideString(std::string str_narrow)
{
    std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
    std::wstring str_wide = converter.from_bytes(str_narrow);
    return str_wide;
};

// 将uint16_t数组转换为int16_t数组
std::vector<int16_t> convertToInt16(const std::vector<uint16_t> &audio)
{
    const auto *rawData = reinterpret_cast<const int16_t *>(audio.data());
    return std::vector<int16_t>(rawData, rawData + audio.size());
}

class TTSNode : public rclcpp::Node
{
public:
    TTSNode() : rclcpp::Node("tts_node")
    {
        _running = true;
        InitModel();

        _tts_content_sub_ptr = this->create_subscription<std_msgs::msg::String>("/tts/content", 10, [this](const std_msgs::msg::String::SharedPtr msg)
                                                                                { this->RxTTSContent(msg); });

        _asr_control_pub_ptr = this->create_publisher<std_msgs::msg::String>("/asr/control", 10);
        _audio_data_pub_ptr = this->create_publisher<interface::msg::AudioDataPackage>("/speaker/audio_data", 10);

        _thread_infer = std::thread(&TTSNode::InferThreadFunction, this);
    }

    ~TTSNode()
    {
        _running = false;
        // 唤醒所有线程
        _text_cv.notify_all();

        if (_thread_infer.joinable())
        {
            _thread_infer.join();
        }

        delete _model_handler;
    }

    void InitModel()
    {
        /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "tts node start initialize models");
        const Hanz2Piny hanz2piny;
        char model_path[] = "/usr/local/share/summertts/single_speaker_fast.bin";
        float *dataW = NULL;
        int32_t modelSize = ttsLoadModel(model_path, &dataW); // 加载模型
        _model_handler = new SynthesizerTrn(dataW, modelSize);
        /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "model init successfully. model file location: %s", model_path);
    }

    std::vector<uint16_t> Infer(const std::string &str)
    {
        int32_t retLen = 0;
        int16_t *wavData = _model_handler->infer(str, 0, 1.0, retLen);
        if (retLen <= 0 || !wavData)
        {
            return {};
        }

        std::vector<uint16_t> audio_data(wavData, wavData + retLen);
        tts_free_data(wavData);
        return audio_data;
    }

    void InferThreadFunction()
    {
        while (_running)
        {
            std::unique_lock<std::mutex> lock(_text_mutex);
            _text_cv.wait(lock, [this]()
                          { return !_local_text_queue.empty() || !_running; });

            while (!_local_text_queue.empty())
            {
                auto text = _local_text_queue.front();
                _local_text_queue.pop();
                lock.unlock();

                if (text != "LLM_RESP_END")
                {
                    auto audio = Infer(text);
                    // 组装AudioDataPackage数据包，发送给speaker节点进行播放
                    auto local_audio_pkg = interface::msg::AudioDataPackage();
                    local_audio_pkg.data = convertToInt16(audio);
                    _audio_data_pub_ptr->publish(local_audio_pkg);
                }
                else
                {
                    // 如果deepseek发言已结束，就发送1条空的AudioDataPackage消息，仅传递tts已完成的内容
                    auto local_audio_pkg = interface::msg::AudioDataPackage();
                    local_audio_pkg.additional_info = "tts finish";
                    _audio_data_pub_ptr->publish(local_audio_pkg);
                }
                /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "已发送AudioDataPackage包");

                lock.lock();
            }
        }
    }

    void RxTTSContent(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string data = msg->data;
        /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "tts receive data: %s", data.c_str());
        size_t chunk_size = 50;                           // 每个块的目标大小（以字符为单位）
        std::wstring wide_data = String2WideString(data); // 将原始字符串转换为宽字符字符串
        {
            std::lock_guard<std::mutex> lock(_text_mutex);

            int i = 1;
            if (wide_data.length() > chunk_size)
            {
                // 遍历宽字符字符串，并按字符数而非字节数分割
                for (size_t pos = 0; pos < wide_data.length(); pos += chunk_size)
                {
                    size_t end = std::min(pos + chunk_size, wide_data.length());
                    std::wstring chunk_wide = wide_data.substr(pos, end - pos);

                    // 将宽字符字符串转换回普通字符串
                    std::string chunk = WideString2String(chunk_wide);
                    _local_text_queue.push(chunk);

                    /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "the %d th part content is: %s", i, chunk.c_str());
                    i++;
                }
                _local_text_queue.push("TTS_END");
            }
            else
            {
                // 如果发来的字数少于要裁剪的长度，那么直接将之添加到队列中即可，无需任何处理
                _local_text_queue.push(data);
            }
        }
        _text_cv.notify_one();
    }

private:
    // 文本队列相关
    std::queue<std::string> _local_text_queue;
    std::mutex _text_mutex;
    std::condition_variable _text_cv;

    // 线程控制
    std::atomic<bool> _running;
    std::thread _thread_infer;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _tts_content_sub_ptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _asr_control_pub_ptr;
    rclcpp::Publisher<interface::msg::AudioDataPackage>::SharedPtr _audio_data_pub_ptr;

    SynthesizerTrn *_model_handler; // 模型句柄
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TTSNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
