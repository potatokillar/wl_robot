/*
 * @Author: 唐文浩
 * @Date: 2022-08-04
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-23
 * @Description: realsense的封装库，目前只支持距离读取
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "realsense.hpp"
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

// 错误处理宏
#define CHECK_AV_ERROR(func)                                                                                         \
    if ((err = func) < 0)                                                                                            \
    {                                                                                                                \
        char errbuf[AV_ERROR_MAX_STRING_SIZE];                                                                       \
        av_make_error_string(errbuf, AV_ERROR_MAX_STRING_SIZE, err);                                                 \
        std::cerr << "Error: " << __FILE__ << ":" << __LINE__ << " - " << errbuf << " (" << err << ")" << std::endl; \
        return err;                                                                                                  \
    }

class RealSenseEncoder
{
private:
    rs2::pipeline pipeline;
    rs2::config config;
    int width, height, fps;

    // FFmpeg组件
    AVFormatContext *format_context = nullptr;
    AVCodec *codec = nullptr;
    AVCodecContext *codec_context = nullptr;
    AVFrame *frame = nullptr;
    AVPacket *packet = nullptr;
    SwsContext *sws_context = nullptr;

    int err;
    bool is_initialized = false;

public:
    RealSenseEncoder(int w = 640, int h = 480, int f = 30) : width(w), height(h), fps(f)
    {
        // 初始化FFmpeg库
        avformat_network_init();
        av_register_all();
        avcodec_register_all();

        // 配置RealSense使用RGB8格式
        config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);
    }

    ~RealSenseEncoder()
    {
        close();
        avformat_network_deinit();
    }

    int initialize(const std::string &output_file)
    {
        // 启动RealSense流
        pipeline.start(config);

        // 分配输出格式上下文
        CHECK_AV_ERROR(avformat_alloc_output_context2(&format_context, nullptr, nullptr, output_file.c_str()));

        // 查找编码器
        codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!codec)
        {
            std::cerr << "Error: H.264 encoder not found" << std::endl;
            return -1;
        }

        // 创建流并配置编码器上下文
        AVStream *stream = avformat_new_stream(format_context, codec);
        if (!stream)
        {
            std::cerr << "Error: Failed to create new stream" << std::endl;
            return -1;
        }

        codec_context = avcodec_alloc_context3(codec);
        if (!codec_context)
        {
            std::cerr << "Error: Failed to allocate codec context" << std::endl;
            return -1;
        }

        codec_context->codec_id = AV_CODEC_ID_H264;
        codec_context->codec_type = AVMEDIA_TYPE_VIDEO;
        codec_context->pix_fmt = AV_PIX_FMT_YUV420P;
        codec_context->width = width;
        codec_context->height = height;
        codec_context->time_base = {1, fps};
        codec_context->framerate = {fps, 1};
        codec_context->gop_size = 10;
        codec_context->max_b_frames = 0; // 无B帧以减少延迟
        codec_context->bit_rate = 4000000;

        // 设置H.264预设和调优参数
        // av_opt_set(codec_context->priv_data, "preset", "ultrafast", 0);
        // av_opt_set(codec_context->priv_data, "tune", "zerolatency", 0);

        // 打开编码器
        CHECK_AV_ERROR(avcodec_open2(codec_context, codec, nullptr));

        // 复制编码器参数到流
        CHECK_AV_ERROR(avcodec_parameters_from_context(stream->codecpar, codec_context));

        // 打开输出文件
        if (!(format_context->oformat->flags & AVFMT_NOFILE))
        {
            CHECK_AV_ERROR(avio_open(&format_context->pb, output_file.c_str(), AVIO_FLAG_WRITE));
        }

        // 写入文件头
        CHECK_AV_ERROR(avformat_write_header(format_context, nullptr));

        // 分配帧和包
        frame = av_frame_alloc();
        frame->format = codec_context->pix_fmt;
        frame->width = width;
        frame->height = height;
        CHECK_AV_ERROR(av_frame_get_buffer(frame, 0));

        packet = av_packet_alloc();

        // 初始化颜色转换上下文 - 从RGB24转换到YUV420P
        sws_context = sws_getContext(width,
                                     height,
                                     AV_PIX_FMT_RGB24, // 注意这里是RGB24
                                     width,
                                     height,
                                     codec_context->pix_fmt,
                                     SWS_BILINEAR,
                                     nullptr,
                                     nullptr,
                                     nullptr);

        is_initialized = true;
        return 0;
    }

    int encodeFrame()
    {
        if (!is_initialized)
            return -1;

        // 等待RealSense帧并转换为OpenCV格式
        rs2::frameset frames = pipeline.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();

        // 创建RGB格式的OpenCV Mat
        cv::Mat frame_rgb(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // 转换颜色空间：RGB -> YUV420P
        const int stride[] = {static_cast<int>(frame_rgb.step[0])};
        sws_scale(sws_context, &frame_rgb.data, stride, 0, height, frame->data, frame->linesize);

        // 设置时间戳
        static int64_t pts = 0;
        frame->pts = pts++;

        // 发送帧到编码器
        CHECK_AV_ERROR(avcodec_send_frame(codec_context, frame));

        // 接收并写入编码后的包
        while (true)
        {
            err = avcodec_receive_packet(codec_context, packet);
            if (err == AVERROR(EAGAIN) || err == AVERROR_EOF)
                break;
            if (err < 0)
            {
                std::cerr << "Error during encoding" << std::endl;
                return err;
            }

            av_packet_rescale_ts(packet, codec_context->time_base, format_context->streams[0]->time_base);
            packet->stream_index = 0;

            CHECK_AV_ERROR(av_interleaved_write_frame(format_context, packet));
            av_packet_unref(packet);
        }

        return 0;
    }

    void close()
    {
        if (!is_initialized)
            return;

        // 刷新编码器
        avcodec_send_frame(codec_context, nullptr);
        while (avcodec_receive_packet(codec_context, packet) >= 0)
        {
            av_packet_rescale_ts(packet, codec_context->time_base, format_context->streams[0]->time_base);
            packet->stream_index = 0;
            av_interleaved_write_frame(format_context, packet);
            av_packet_unref(packet);
        }

        // 写入文件尾
        av_write_trailer(format_context);

        // 释放资源
        if (sws_context)
            sws_freeContext(sws_context);
        if (packet)
            av_packet_free(&packet);
        if (frame)
            av_frame_free(&frame);
        if (codec_context)
            avcodec_free_context(&codec_context);
        if (format_context && !(format_context->oformat->flags & AVFMT_NOFILE))
        {
            avio_closep(&format_context->pb);
        }
        if (format_context)
            avformat_free_context(format_context);

        pipeline.stop();
        is_initialized = false;
    }
};
//////////////////////////////////////////////////////////////////////////////////////

class RealSenseMemoryEncoder
{
private:
    rs2::pipeline pipeline;
    rs2::config config;
    int width, height, fps;

    // FFmpeg组件
    AVFormatContext *format_context = nullptr;
    AVCodec *codec = nullptr;
    AVCodecContext *codec_context = nullptr;
    AVFrame *frame = nullptr;
    AVPacket *packet = nullptr;
    SwsContext *sws_context = nullptr;
    AVIOContext *avio_context = nullptr;

    // 内存输出相关
    std::vector<uint8_t> output_data;
    uint8_t *avio_buffer = nullptr;
    size_t avio_buffer_size = 4096;
    int err;
    bool is_initialized = false;

    // 自定义的AVIOContext写回调函数
    static int write_callback(void *opaque, uint8_t *buf, int buf_size)
    {
        std::vector<uint8_t> *output_data = static_cast<std::vector<uint8_t> *>(opaque);
        output_data->insert(output_data->end(), buf, buf + buf_size);
        return buf_size;
    }

public:
    RealSenseMemoryEncoder(int w = 640, int h = 480, int f = 30) : width(w), height(h), fps(f)
    {
        // 初始化FFmpeg库
        avformat_network_init();
        av_register_all();
        avcodec_register_all();

        // 配置RealSense使用RGB8格式
        config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);
    }

    ~RealSenseMemoryEncoder()
    {
        close();
        avformat_network_deinit();
    }

    int initialize()
    {
        // 启动RealSense流
        pipeline.start(config);

        // 分配输出格式上下文，使用NULL让FFmpeg自动选择MUXER
        CHECK_AV_ERROR(avformat_alloc_output_context2(&format_context, nullptr, "h264", nullptr));
        if (!format_context)
        {
            std::cerr << "Error: Could not create output context" << std::endl;
            return -1;
        }

        // 查找编码器
        codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!codec)
        {
            std::cerr << "Error: H.264 encoder not found" << std::endl;
            return -1;
        }

        // 创建流并配置编码器上下文
        AVStream *stream = avformat_new_stream(format_context, codec);
        if (!stream)
        {
            std::cerr << "Error: Failed to create new stream" << std::endl;
            return -1;
        }

        codec_context = avcodec_alloc_context3(codec);
        if (!codec_context)
        {
            std::cerr << "Error: Failed to allocate codec context" << std::endl;
            return -1;
        }

        codec_context->codec_id = AV_CODEC_ID_H264;
        codec_context->codec_type = AVMEDIA_TYPE_VIDEO;
        codec_context->pix_fmt = AV_PIX_FMT_YUV420P;
        codec_context->width = width;
        codec_context->height = height;
        codec_context->time_base = {1, fps};
        codec_context->framerate = {fps, 1};
        codec_context->gop_size = 10;
        codec_context->max_b_frames = 0; // 无B帧以减少延迟
        codec_context->bit_rate = 4000000;

        // 设置H.264预设和调优参数
        // av_opt_set(codec_context->priv_data, "preset", "ultrafast", 0);
        // av_opt_set(codec_context->priv_data, "tune", "zerolatency", 0);

        // 打开编码器
        CHECK_AV_ERROR(avcodec_open2(codec_context, codec, nullptr));

        // 复制编码器参数到流
        CHECK_AV_ERROR(avcodec_parameters_from_context(stream->codecpar, codec_context));

        // 创建自定义的AVIOContext用于内存输出
        avio_buffer = (uint8_t *)av_malloc(avio_buffer_size);
        if (!avio_buffer)
        {
            std::cerr << "Error: Failed to allocate IO buffer" << std::endl;
            return -1;
        }

        avio_context = avio_alloc_context(avio_buffer,
                                          avio_buffer_size,
                                          1, // 写模式
                                          &output_data,
                                          nullptr,        // 读回调
                                          write_callback, // 写回调
                                          nullptr         // 寻道回调
        );

        if (!avio_context)
        {
            std::cerr << "Error: Failed to allocate AVIO context" << std::endl;
            av_free(avio_buffer);
            return -1;
        }

        format_context->pb = avio_context;
        format_context->flags |= AVFMT_FLAG_CUSTOM_IO;

        // 写入文件头（这里实际上是写入内存）
        CHECK_AV_ERROR(avformat_write_header(format_context, nullptr));

        // 分配帧和包
        frame = av_frame_alloc();
        frame->format = codec_context->pix_fmt;
        frame->width = width;
        frame->height = height;
        CHECK_AV_ERROR(av_frame_get_buffer(frame, 0));

        packet = av_packet_alloc();

        // 初始化颜色转换上下文 - 从RGB24转换到YUV420P
        sws_context = sws_getContext(width,
                                     height,
                                     AV_PIX_FMT_RGB24, // 注意这里是RGB24
                                     width,
                                     height,
                                     codec_context->pix_fmt,
                                     SWS_BILINEAR,
                                     nullptr,
                                     nullptr,
                                     nullptr);

        is_initialized = true;
        return 0;
    }

    int encodeFrame()
    {
        if (!is_initialized)
            return -1;

        // 等待RealSense帧并转换为OpenCV格式
        rs2::frameset frames = pipeline.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();

        // 创建RGB格式的OpenCV Mat
        cv::Mat frame_rgb(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // 转换颜色空间：RGB -> YUV420P
        const int stride[] = {static_cast<int>(frame_rgb.step[0])};
        sws_scale(sws_context, &frame_rgb.data, stride, 0, height, frame->data, frame->linesize);

        // 设置时间戳
        static int64_t pts = 0;
        frame->pts = pts++;

        // 发送帧到编码器
        CHECK_AV_ERROR(avcodec_send_frame(codec_context, frame));

        // 接收并写入编码后的包
        while (true)
        {
            err = avcodec_receive_packet(codec_context, packet);
            if (err == AVERROR(EAGAIN) || err == AVERROR_EOF)
                break;
            if (err < 0)
            {
                std::cerr << "Error during encoding" << std::endl;
                return err;
            }

            av_packet_rescale_ts(packet, codec_context->time_base, format_context->streams[0]->time_base);
            packet->stream_index = 0;

            CHECK_AV_ERROR(av_interleaved_write_frame(format_context, packet));
            av_packet_unref(packet);
        }

        return 0;
    }

    // 获取编码后的H.264数据
    std::vector<uint8_t> getEncodedData() const { return output_data; }
    void clear() { output_data.clear(); }

    void close()
    {
        if (!is_initialized)
            return;

        // 刷新编码器
        avcodec_send_frame(codec_context, nullptr);
        while (avcodec_receive_packet(codec_context, packet) >= 0)
        {
            av_packet_rescale_ts(packet, codec_context->time_base, format_context->streams[0]->time_base);
            packet->stream_index = 0;
            av_interleaved_write_frame(format_context, packet);
            av_packet_unref(packet);
        }

        // 写入文件尾
        av_write_trailer(format_context);

        // 释放资源
        if (sws_context)
            sws_freeContext(sws_context);
        if (packet)
            av_packet_free(&packet);
        if (frame)
            av_frame_free(&frame);
        if (codec_context)
            avcodec_free_context(&codec_context);
        if (avio_context)
        {
            avio_context->buffer = nullptr; // 防止被释放两次
            av_free(avio_context);
        }
        if (avio_buffer)
            av_free(avio_buffer);
        if (format_context)
        {
            format_context->pb = nullptr; // 防止被释放两次
            avformat_free_context(format_context);
        }

        pipeline.stop();
        is_initialized = false;
    }
};
//////////////////////////////////////////////////////////////////////////////////////////////////

void SaveRealsenseRGB(const std::string &name)
{
    RealSenseEncoder encoder(640, 480, 30);

    if (encoder.initialize(name) != 0)
    {
        std::cerr << "Failed to initialize encoder" << std::endl;
        return;
    }

    std::cout << "Encoding file frames. Press Enter to stop..." << std::endl;
    for (int i = 0; i < 300; i++)
    { // 编码300帧（约10秒）
        if (encoder.encodeFrame() != 0)
            break;

        if (std::cin.rdbuf()->in_avail() > 0)
        {
            std::cin.ignore();
            break;
        }
    }
    encoder.close();
    std::cout << "Encoding completed. Output saved to output.mp4" << std::endl;
}

static RealSenseMemoryEncoder encoder(640, 480, 30);
void RealsenseStart() { encoder.initialize(); }
void RealsenseStop()
{
    // 使用完后关闭编码器
    encoder.close();
}
std::vector<uint8_t> GetRealsenseRGB()
{
    // 编码一些帧
    // for (int i = 0; i < 100; i++) {
    encoder.encodeFrame();
    // }

    // 获取编码后的数据
    std::vector<uint8_t> h264_data = encoder.getEncodedData();
    encoder.clear();
    return h264_data;
}