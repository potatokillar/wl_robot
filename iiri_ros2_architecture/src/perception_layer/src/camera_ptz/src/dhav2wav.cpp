/*
 * @Author: 唐文浩
 * @Date: 2025-04-28
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-29
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "dhav2wav.hpp"

#include <iostream>
#include <rclcpp/rclcpp.hpp>

/**
 * @description: 从输入读取数据，全部读取
 * @param *opaque
 * @param *buf
 * @param buf_size
 * @return {}
 */
int in_read(void *opaque, uint8_t *buf, int buf_size)
{
    DhavWavBuffer *buffer = (DhavWavBuffer *)opaque;
    if (buffer->input_pos >= buffer->input.size())
    {
        //  std::cout << "read frame null " << buffer->input_pos << std::endl;
        return AVERROR(EAGAIN);
    }

    int bytes_to_read = std::min(buf_size, static_cast<int>(buffer->input.size() - buffer->input_pos));
    std::copy(buffer->input.begin() + buffer->input_pos, buffer->input.begin() + buffer->input_pos + bytes_to_read, buf);
    buffer->input_pos += bytes_to_read;
    // std::cout << "read frame " << bytes_to_read << std::endl;
    return bytes_to_read;
}

int in_write(void *opaque, uint8_t *buf, int buf_size)
{
    // std::cout << "write frame:" << buf_size << std::endl;
    DhavWavBuffer *buffer = (DhavWavBuffer *)opaque;
    // 调整向量大小以容纳新数据
    buffer->output.resize(buf_size);
    // 将数据从 buf 复制到 buffer->input 中
    std::copy(buf, buf + buf_size, buffer->output.begin());
    return buf_size;
}

void Dhav2Wav::init()
{
    if (is_init_)
    {
        //  return;
    }

    try
    {
        create_in_ioctx();
        create_in_stream();
        find_in_codec();
        create_out_ioctx();
        create_out_stream();
        set_out_codec();
        // is_init_ = true;
    }
    catch (const std::exception &e)
    {
        deinit();
        RCLCPP_ERROR(rclcpp::get_logger("Dhav2Wav"), "init failed:%s", e.what());
    }
}
void Dhav2Wav::deinit()
{
    // is_init_ = false;
    av_write_trailer(ofmt_ctx);
}

/**
 * @description: 创建输入IO上下文，当输入是内存时，必须要
 * @return {}
 */
void Dhav2Wav::create_in_ioctx()
{
    in_buf_ = (uint8_t *)av_malloc(4096);
    in_ioctx_ = avio_alloc_context(in_buf_, 4096, 0, (void *)&buffer_, in_read, nullptr, nullptr);
    if (!in_ioctx_)
    {
        // 抛出异常
        throw std::runtime_error("create_in_ioctx failed");
    }
}

/**
 * @description: 创建输入流
 * @return {}
 */
void Dhav2Wav::create_in_stream()
{
    // 查找输入格式，当流不能自动检测时，需要显式指定
    AVInputFormat *input_format = av_find_input_format("dhav");
    if (!input_format)
    {
        throw std::runtime_error("av_find_input_format failed");
    }

    // 创建输入流
    ifmt_ctx = avformat_alloc_context();
    if (!ifmt_ctx)
    {
        throw std::runtime_error("avformat_alloc_context failed");
    }
    ifmt_ctx->pb = in_ioctx_; // 当stream为内存时，需要设置pb为in_ioctx_

    // 打开输入内存流，输入为内存时，第二个参数为null
    // 需要库自动猜测输入格式时，第三个参数为null
    if (auto ret = avformat_open_input(&ifmt_ctx, nullptr, nullptr, nullptr); ret < 0)
    {
        throw std::runtime_error("avformat_open_input: " + get_error(ret));
    }

    // 查找输入流信息
    if (avformat_find_stream_info(ifmt_ctx, nullptr) < 0)
    {
        throw std::runtime_error("avformat_find_stream_info failed");
    }

    // 查找音频流
    audio_stream_index = -1;
    for (unsigned int i = 0; i < ifmt_ctx->nb_streams; i++)
    {
        if (ifmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO)
        {
            audio_stream_index = i;
            break;
        }
    }

    if (audio_stream_index == -1)
    {
        throw std::runtime_error("no audio stream found");
    }
    in_stream = ifmt_ctx->streams[audio_stream_index]; // 音频流
}

/**
 * @description: 查找输入解码器
 * @return {}
 */
void Dhav2Wav::find_in_codec()
{
    AVCodecParameters *codecPara = in_stream->codecpar;
    AVCodec *codec = avcodec_find_decoder(codecPara->codec_id);
    if (codec == NULL)
    {
        throw std::runtime_error("avcodec_find_decoder failed");
    }

    // 根据解码器参数来创建解码器内容
    in_codec_ctx_ = avcodec_alloc_context3(codec);
    avcodec_parameters_to_context(in_codec_ctx_, codecPara);
    if (in_codec_ctx_ == NULL)
    {
        throw std::runtime_error("avcodec_parameters_to_context failed");
    }

    //================================  打开解码器 ===================================//
    if (avcodec_open2(in_codec_ctx_, codec, NULL) < 0)
    { // 具体采用什么解码器ffmpeg经过封装 我们无须知道
        throw std::runtime_error("avcodec_open2 failed");
    }
}

/**
 * @description: 创建输出上下文
 * @return {}
 */
void Dhav2Wav::create_out_ioctx()
{
    out_buf_ = (uint8_t *)av_malloc(4096);
    out_ioctx_ = avio_alloc_context(out_buf_, 4096, 1, (void *)&buffer_, nullptr, in_write, nullptr);
    if (!out_ioctx_)
    {
        throw std::runtime_error("avio_alloc_context out failed");
    }
    std::cout << "Output IO context allocated successfully" << std::endl;
}

/**
 * @description: 创建输出流
 * @return {}
 */
void Dhav2Wav::create_out_stream()
{
    // 分配输出格式上下文，明确指定输出格式为 WAV
    AVOutputFormat *out_format = av_guess_format("wav", nullptr, nullptr);
    if (!out_format)
    {
        throw std::runtime_error("av_guess_format failed");
    }
    // 分配输出格式上下文
    avformat_alloc_output_context2(&ofmt_ctx, out_format, "wav", nullptr);
    if (!ofmt_ctx)
    {
        throw std::runtime_error("avformat_alloc_output_context2 failed");
    }
    ofmt_ctx->pb = out_ioctx_;
    ofmt_ctx->oformat->flags |= AVFMT_NOFILE;

    // 创建输出流
    out_stream = avformat_new_stream(ofmt_ctx, nullptr);
    if (!out_stream)
    {
        throw std::runtime_error("avformat_new_stream failed");
    }
}

/**
 * @description: 设置输出编码器参数
 * @return {}
 */
void Dhav2Wav::set_out_codec()
{
    // 复制编解码参数，因为内部编解码部分不做修改
    if (avcodec_parameters_copy(out_stream->codecpar, in_stream->codecpar) < 0)
    {
        throw std::runtime_error("avcodec_parameters_copy failed");
    }

    std::cout << "采样率 " << out_stream->codecpar->sample_rate << std::endl;
    std::cout << "通道数 " << out_stream->codecpar->channels << std::endl;
    std::cout << "采样位数 " << out_stream->codecpar->bits_per_coded_sample << std::endl;
    // out_stream->codecpar->codec_id = AV_CODEC_ID_PCM_S16LE;
    std::cout << "比特率" << out_stream->codecpar->bit_rate << std::endl;
    std::cout << "输出格式" << out_stream->codecpar->codec_id << std::endl;
}

/**
 * @description: dhav转成wav
 * @return {}
 */
void Dhav2Wav::convert()
{
    if (!is_init_)
    {
        std::cout << "failed" << std::endl;
        return;
    }
    /*     std::cout << "failed e" << std::endl;
        // 打开流
        if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE)) {
            std::cout << "failed m" << &ofmt_ctx->pb << std::endl;
            if (auto ret = avio_open(&ofmt_ctx->pb, nullptr, AVIO_FLAG_WRITE); ret < 0) {
                std::cout << "failed y" << std::endl;
                throw std::runtime_error("avio_open failed" + get_error(ret));
            }
            std::cout << "failed 55555555555" << std::endl;
        }
        std::cout << "failed uuuuuuuuuuu" << std::endl; */

    if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE))
    {
        // std::cout << "Output format requires file operations" << std::endl;
    }
    else
    {
        // std::cout << "Output format supports memory stream operations" << std::endl;
    }

    // 写入文件头
    if (avformat_write_header(ofmt_ctx, nullptr) < 0)
    {
        if (!(ofmt_ctx->oformat->flags & AVFMT_NOFILE))
        {
            avio_closep(&ofmt_ctx->pb);
        }

        throw std::runtime_error("avformat_write_header failed");
    }
    //
    // 读取和写入数据包
    AVPacket packet;
    while (av_read_frame(ifmt_ctx, &packet) >= 0)
    {
        if (packet.stream_index == audio_stream_index)
        {
            packet.stream_index = out_stream->index;
            if (av_interleaved_write_frame(ofmt_ctx, &packet) < 0)
            {
                fprintf(stderr, "Error writing packet\n");
                break;
            }
        }

        av_packet_unref(&packet);
    }

    // 写入文件尾
    av_write_trailer(ofmt_ctx);
    // double rmsVolume = calculateRMSVolume(packet->data, packet->size, sampleSize);
}

void Dhav2Wav::set_input(const std::vector<uint8_t> &input)
{
    // RCLCPP_INFO(rclcpp::get_logger("Dhav2Wav"), "Dhav2Wav::set_input:%d", input.size());
    buffer_.input = input;
    buffer_.input_pos = 0;
    init();
    convert();
}

std::vector<uint8_t> Dhav2Wav::get_output() { return buffer_.output; }

std::string Dhav2Wav::get_error(int errnum)
{
    char errStr[AV_ERROR_MAX_STRING_SIZE];
    av_make_error_string(errStr, AV_ERROR_MAX_STRING_SIZE, errnum);
    return std::string(errStr);
}
#if 0
// 计算 RMS 音量
double Dhav2Wav::calculate_vol(const uint8_t *data, int size, int sampleSize)
{
    double sumSquared = 0.0;
    int numSamples = size / sampleSize;

    for (int i = 0; i < numSamples; ++i) {
        int16_t sample = *(reinterpret_cast<const int16_t *>(data + i * sampleSize));
        sumSquared += static_cast<double>(sample) * static_cast<double>(sample);
    }

    if (numSamples == 0) {
        return 0.0;
    }

    double rms = std::sqrt(sumSquared / numSamples);
    return 20.0 * std::log10(rms / 32767.0);  // 16 位 PCM 最大值为 32767
}
#endif