/*
 * @Author: 唐文浩
 * @Date: 2025-04-28
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-29
 * @Description: 大华DHAV格式转WAV格式，大华的PCM音频是dhav格式的
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <string>

#include "robot_base/cpp_types.hpp"
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswresample/swresample.h>
}

// 通用数据结构体
struct DhavWavBuffer
{
    std::vector<uint8_t> input;  // 输入数据
    std::vector<uint8_t> output; // 输出数据
    size_t input_pos;
};

class Dhav2Wav
{
public:
    void init();
    void deinit();
    void convert();
    void set_input(const std::vector<uint8_t> &input);
    std::vector<uint8_t> get_output();
    void set_header();
    void set_tail();

private:
    void create_in_ioctx();
    void create_in_stream();
    void find_in_codec();
    void create_out_ioctx();
    void create_out_stream();
    void set_out_codec();

    std::string get_error(int errnum);

private:
    // 初始化只需要一次，但是必须在流存在时初始化
    bool is_init_ = false;
    // IO上下文，输入输出为内存时需要
    AVIOContext *in_ioctx_ = nullptr;
    AVIOContext *out_ioctx_ = nullptr;
    // 格式上下文
    AVFormatContext *ifmt_ctx = nullptr;
    AVFormatContext *ofmt_ctx = nullptr;
    // 编解码器上下文
    AVCodecContext *in_codec_ctx_ = nullptr;
    AVCodecContext *out_codec_ctx_ = nullptr;
    // 流
    AVStream *in_stream = nullptr;
    AVStream *out_stream = nullptr;

    int audio_stream_index = -1;

    uint8_t *in_buf_;
    uint8_t *out_buf_;
    DhavWavBuffer buffer_;
    u32 mute_cnt_{0}; // 静音时间
};