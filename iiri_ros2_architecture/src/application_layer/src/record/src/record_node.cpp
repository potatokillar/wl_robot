/*
 * @Author: 唐文浩
 * @Date: 2025-04-09
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-18
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <alsa/asoundlib.h>
#include <speex/speex_preprocess.h>

#include <fstream>

#include "atomicops.h"
#include "interface/msg/audio_data_package.hpp"
#include "rclcpp/rclcpp.hpp"
#include "readerwriterqueue.h"
#include "std_msgs/msg/string.hpp"

// static unsigned int s_actual_record_sample_rate;
// static unsigned int s_actual_record_channels;
// static snd_pcm_format_t s_actual_record_format;
static std::atomic<bool> s_enable_write_pcm = false;
static moodycamel::ReaderWriterQueue<uint8_t> s_uint8_buffer(32 * 1024); // 缓存音频数据的容器

// WAV文件头结构体
struct WavHeader
{
    char riff[4];
    uint32_t chunkSize; // 文件大小 - 8
    char wave[4];
    char fmt[4];
    uint32_t subchunk1Size; // 对PCM来说是16
    uint16_t audioFormat;   // PCM
    uint16_t numChannels;
    uint32_t sampleRate;
    uint32_t byteRate;
    uint16_t blockAlign;
    uint16_t bitsPerSample;
    char data[4];
    uint32_t subchunk2Size; // 数据长度

    WavHeader() : chunkSize(0), subchunk1Size(16), audioFormat(1), numChannels(0), sampleRate(0), byteRate(0), blockAlign(0), bitsPerSample(0), subchunk2Size(0)
    {
        riff[0] = 'R';
        riff[1] = 'I';
        riff[2] = 'F';
        riff[3] = 'F';
        wave[0] = 'W';
        wave[1] = 'A';
        wave[2] = 'V';
        wave[3] = 'E';
        fmt[0] = 'f';
        fmt[1] = 'm';
        fmt[2] = 't';
        fmt[3] = ' ';
        data[0] = 'd';
        data[1] = 'a';
        data[2] = 't';
        data[3] = 'a';
    }
};

// 辅助函数，写入WAV文件头
static void write_wav_header(FILE *file, uint32_t data_size, uint32_t sample_rate, uint16_t num_channels, uint16_t bits_per_sample)
{
    // RIFF头
    const char riff_header[] = {'R', 'I', 'F', 'F'};
    fwrite(riff_header, 1, 4, file);

    uint32_t chunk_size = 36 + data_size; // 4 + (8 + 16) + (8 + data_size)
    fwrite(&chunk_size, 4, 1, file);

    const char wave_header[] = {'W', 'A', 'V', 'E'};
    fwrite(wave_header, 1, 4, file);

    // fmt子块
    const char fmt_header[] = {'f', 'm', 't', ' '};
    fwrite(fmt_header, 1, 4, file);

    uint32_t subchunk1_size = 16;
    fwrite(&subchunk1_size, 4, 1, file);

    uint16_t audio_format = 1; // PCM格式
    fwrite(&audio_format, 2, 1, file);

    fwrite(&num_channels, 2, 1, file);
    fwrite(&sample_rate, 4, 1, file);

    uint32_t byte_rate = sample_rate * num_channels * bits_per_sample / 8;
    fwrite(&byte_rate, 4, 1, file);

    uint16_t block_align = num_channels * bits_per_sample / 8;
    fwrite(&block_align, 2, 1, file);

    fwrite(&bits_per_sample, 2, 1, file);

    // data子块
    const char data_header[] = {'d', 'a', 't', 'a'};
    fwrite(data_header, 1, 4, file);
    fwrite(&data_size, 4, 1, file);
}

// 根据设备名，自动生成pcm设备代号名，比如plughw:1,0
static std::vector<std::string> FindDeviceByName(const std::string &target_name)
{
    std::vector<std::string> result = {};

    std::ifstream file("/proc/asound/cards");
    std::string line;
    int deviceNumber = -1;

    // 逐行读取文件内容
    while (getline(file, line))
    {
        // 找到包含目标设备名称的行
        if (line.find(target_name) != std::string::npos && line.find("[") != std::string::npos)
        {
            // 解析前面的序号
            std::istringstream iss(line);
            iss >> deviceNumber; // 第一个字段是序号
            if (deviceNumber != -1)
            {
                result.push_back("plughw:" + std::to_string(deviceNumber) + ",0");
            }
        }
    }

    return result;
}

class RecordNode : public rclcpp::Node
{
public:
    RecordNode() : Node("record_node")
    {
        _audio_data_pub_ptr = this->create_publisher<interface::msg::AudioDataPackage>("/record/audio_data", 10);
        _tts_content_pub_ptr = this->create_publisher<std_msgs::msg::String>("/tts/content", 10);

        // 读取参数，是否将音频数据写入到1个pcm文件中
        this->declare_parameter<int>("enable_write_pcm");
        if (this->get_parameter("enable_write_pcm").as_int() == 1)
        {
            s_enable_write_pcm.store(true);
            RCLCPP_INFO(this->get_logger(), "开启音频录制");
        }
        else
        {
            s_enable_write_pcm.store(false);
            RCLCPP_INFO(this->get_logger(), "关闭音频录制");
        }
        // 获取parameter，如果没获取到则使用默认值
#ifdef __x86_64__
        this->declare_parameter<std::string>("x86_record_device_name", "default_x86_device");
        _record_dev = this->get_parameter("x86_record_device_name").as_string();
#else
        this->declare_parameter<std::string>("arm_record_device_name", "default_arm_device");
        _record_dev = this->get_parameter("arm_record_device_name").as_string();
#endif
        RCLCPP_INFO(this->get_logger(), "_record_dev = %s", _record_dev.c_str());

        _thread_record = std::thread(&RecordNode::ThreadRecord, this);

        RCLCPP_INFO(this->get_logger(), "Record node has been started.");
    }

    int OpenDevice(const char *device, unsigned int sample_rate, unsigned int channels, snd_pcm_format_t format, unsigned int *actual_sample_rate,
                   unsigned int *actual_channels, snd_pcm_format_t *actual_format, snd_pcm_t **pcm_handle)
    {
        snd_pcm_hw_params_t *hw_params = NULL;
        int rc;

        // Open PCM device for recording
        rc = snd_pcm_open(pcm_handle, device, SND_PCM_STREAM_CAPTURE, 0);
        if (rc < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open PCM device: %s", snd_strerror(rc));
            return 1;
        }

        // Allocate and initialize hardware parameters structure
        snd_pcm_hw_params_alloca(&hw_params);
        snd_pcm_hw_params_any(*pcm_handle, hw_params);

        // Set hardware parameters
        rc = snd_pcm_hw_params_set_access(*pcm_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
        rc |= snd_pcm_hw_params_set_format(*pcm_handle, hw_params, format);
        rc |= snd_pcm_hw_params_set_channels(*pcm_handle, hw_params, channels);
        rc |= snd_pcm_hw_params_set_rate_near(*pcm_handle, hw_params, &sample_rate, 0);
        if (rc < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set parameters: %s", snd_strerror(rc));
            snd_pcm_close(*pcm_handle);
            *pcm_handle = NULL;
            return 1;
        }

        // Apply hardware parameters
        rc = snd_pcm_hw_params(*pcm_handle, hw_params);
        if (rc < 0)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to apply parameters for record: %s, sample_rate = %d, channels = %d",
                         snd_strerror(rc),
                         sample_rate,
                         channels);
            snd_pcm_close(*pcm_handle);
            *pcm_handle = NULL;
            return 1;
        }

        // Retrieve and display audio parameters
        snd_pcm_uframes_t frames;
        snd_pcm_hw_params_get_period_size(hw_params, &frames, 0);
        snd_pcm_hw_params_get_rate(hw_params, &sample_rate, 0);
        snd_pcm_hw_params_get_channels(hw_params, &channels);
        snd_pcm_hw_params_get_format(hw_params, &format);

        RCLCPP_INFO(this->get_logger(), "frames = %ld ************", frames);

        // Set output parameters
        if (actual_sample_rate)
            *actual_sample_rate = sample_rate;
        if (actual_channels)
            *actual_channels = channels;
        if (actual_format)
            *actual_format = format;

        return 0;
    }

    void ThreadRecord()
    {
        snd_pcm_t *pcm_handle = NULL;
        unsigned char *buffer = NULL;
        int rc;
        const char *device = nullptr;                    // 存放麦克风设备的字符串
        unsigned int sample_rate = 16000;                // 44.1 kHz
        unsigned int channels = 1;                       // Stereo
        snd_pcm_format_t format = SND_PCM_FORMAT_S16_LE; // 16-bit little-endian

        unsigned int actual_sample_rate;
        unsigned int actual_channels;
        snd_pcm_format_t actual_format;

        while (1)
        { // 最外部的工作循环，record节点要一直运行
            // 打开麦克风设备
            // 如果打不开给定的麦克风设备，则一直阻塞在这里
            bool local_mic_found = false;
            int local_time_step = 2000; // 重复尝试的时间间隔
            do
            {
                std::vector<std::string> l_device_list = FindDeviceByName(_record_dev); // 读取同名音频设备列表
                if (l_device_list.size() != 0)
                {
                    // Open PCM device for recording
                    for (unsigned int i = 0; i < l_device_list.size(); i++)
                    {
                        device = l_device_list.at(i).c_str();
                        RCLCPP_INFO(this->get_logger(), "try to open PCM device %s ...", device);
                        int ret = OpenDevice(device, sample_rate, channels, format, &actual_sample_rate, &actual_channels, &actual_format, &pcm_handle);
                        if (ret != 0)
                        { // 返回值为0是打开成功，否则都是打开失败
                            RCLCPP_INFO(this->get_logger(), "Failed to open PCM device: %s, try another device soon ...", device);
                            continue;
                        }
                        else
                        {
                            RCLCPP_INFO(this->get_logger(), "open PCM device %s successfully", device);
                            local_mic_found = true;
                            break;
                        }
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(local_time_step)); // 阻塞时，检测是否能重连的时间间隔
                RCLCPP_DEBUG(this->get_logger(), "looping per %d milliseconds... waiting for connection with mic device", local_time_step);
            } while (!local_mic_found);

            RCLCPP_DEBUG(this->get_logger(),
                         "actual_sample_rate: %d, actual_channels: %d, actual_format: %d",
                         actual_sample_rate,
                         actual_channels,
                         actual_format);

            // s_actual_record_sample_rate = actual_sample_rate;
            // s_actual_record_channels = actual_channels;
            // s_actual_record_format = actual_format;

            // Get hardware parameters
            snd_pcm_hw_params_t *hw_params;
            snd_pcm_hw_params_alloca(&hw_params);
            snd_pcm_hw_params_current(pcm_handle, hw_params);

            // Retrieve period size
            snd_pcm_uframes_t frames;
            snd_pcm_hw_params_get_period_size(hw_params, &frames, 0);

            // Calculate frame size
            size_t frame_size = snd_pcm_format_width(actual_format) / 8;

            RCLCPP_INFO(this->get_logger(), "Actual recording settings:");
            RCLCPP_INFO(this->get_logger(), "  Sample Rate: %u Hz", actual_sample_rate);
            RCLCPP_INFO(this->get_logger(), "  Channels: %u", actual_channels);
            RCLCPP_INFO(this->get_logger(), "  Bit Depth: %s, actual_format = %d", snd_pcm_format_name(actual_format), actual_format);
            RCLCPP_INFO(this->get_logger(), "  Frames: %lu", (unsigned long)frames);
            RCLCPP_INFO(this->get_logger(), "  Frame Size: %zu", frame_size);
            // SendTTSContent("麦克风已开启");  // 语音播报

            // Allocate PCM data buffer
            buffer = (unsigned char *)malloc(frames * frame_size * actual_channels);
            if (!buffer)
            {
                fprintf(stderr, "Memory allocation failed\n");
                snd_pcm_drain(pcm_handle);
                snd_pcm_close(pcm_handle);
            }

            // 降噪
            SpeexPreprocessState *speex_states[2] = {NULL, NULL};
            for (unsigned int i = 0; i < channels; i++)
            {
                speex_states[i] = speex_preprocess_state_init(frames, sample_rate);
                int denoise = 1;
                int noise_suppress = -10;
                speex_preprocess_ctl(speex_states[i], SPEEX_PREPROCESS_SET_DENOISE, &denoise);
                speex_preprocess_ctl(speex_states[i], SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, &noise_suppress);
            }

            // uint16_t *channel_buffers[2];
            // for (unsigned int i = 0; i < actual_channels; i++) {
            //     channel_buffers[i] = (uint16_t *)malloc(frames * sizeof(uint16_t));
            // }

            // 新增WAV文件相关变量
            FILE *wav_file = nullptr;
            size_t data_bytes_written = 0;
            const uint32_t wav_sample_rate = 16000;
            const uint16_t wav_channels = 1;
            const uint16_t wav_bits_per_sample = 16;
            if (s_enable_write_pcm.load())
            {
                // 打开设备后创建WAV文件
                wav_file = fopen("recording.wav", "wb");
                if (!wav_file)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to create WAV file");
                    if (pcm_handle)
                    {
                        snd_pcm_drain(pcm_handle);
                        snd_pcm_close(pcm_handle);
                    }
                    return;
                }
                write_wav_header(wav_file, 0, wav_sample_rate, wav_channels, wav_bits_per_sample);
            }

            // 开始录音的循环
            RCLCPP_INFO(this->get_logger(), "Recording started...");
            while (1)
            {
                rc = snd_pcm_readi(pcm_handle, buffer, frames); // rc是成功读取到的帧数(frame数)
                if (rc == -EPIPE)
                {
                    // 出错分支
                    RCLCPP_INFO(this->get_logger(), "Buffer overrun detected, recovering...");
                    snd_pcm_prepare(pcm_handle);
                }
                else if (rc < 0)
                {
                    // 出错分支
                    RCLCPP_INFO(this->get_logger(), "Read error: %s\n", snd_strerror(rc));
                    break;
                }
                else
                {
                    // 成功录制到音频的分支
                    // 组装AudioDataPackage数据包，发送给speaker节点进行播放
                    auto local_audio_pkg = interface::msg::AudioDataPackage();
                    size_t size = rc * frame_size * actual_channels;
                    // 计算 vector 的大小
                    size_t vectorSize = (size + sizeof(int16_t) - 1) / sizeof(int16_t);
                    // 创建一个 std::vector<int16_t> 对象
                    std::vector<int16_t> result(vectorSize);
                    // 将 unsigned char 数组的数据拷贝到 std::vector<int16_t> 中
                    for (size_t i = 0; i < vectorSize; ++i)
                    {
                        int16_t value = 0;
                        if (2 * i < size)
                        {
                            value = static_cast<int16_t>(buffer[2 * i]);
                        }
                        if (2 * i + 1 < size)
                        {
                            value |= static_cast<int16_t>(buffer[2 * i + 1]) << 8;
                        }
                        result[i] = value;
                    }

                    for (size_t i = 0; i < size; i++)
                    {
                        s_uint8_buffer.enqueue(buffer[i]);
                    }

                    local_audio_pkg.data = result;
                    _audio_data_pub_ptr->publish(local_audio_pkg);

                    // 打印log信息
                    // static int cnt = 0;
                    // if (cnt > 100) {
                    //     /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "recording... ");
                    //     cnt = 0;
                    // }
                    // cnt++;

                    // 将前16个字节打印出来
                    // size_t elementsToPrint = std::min(static_cast<size_t>(8), result.size());
                    // std::ostringstream oss;
                    // oss << std::hex;
                    // for (size_t i = 0; i < elementsToPrint; ++i) {
                    //     oss << std::setw(4) << std::setfill('0') << result[i];
                    //     if (i < elementsToPrint - 1) {
                    //         oss << " ";
                    //     }
                    // }
                    // /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "First 16 bytes of audio data (in hex): %s", oss.str().c_str());

                    if (s_enable_write_pcm.load())
                    {
                        // 写入wav文件
                        size_t bytes_available = rc * frame_size * actual_channels;
                        // size_t bytes_needed = rc * frame_size * wav_channels;
                        if (actual_channels == wav_channels)
                        {
                            // 直接写入数据
                            size_t written = fwrite(buffer, 1, bytes_available, wav_file);
                            data_bytes_written += written;
                        }
                        else
                        {
                            // 多声道转单声道（取平均值）
                            int16_t *src = reinterpret_cast<int16_t *>(buffer);
                            int16_t *dst = new int16_t[rc];

                            unsigned int temp =
                                rc; // 为了消除warning: comparison of integer expressions of different signedness: ‘snd_pcm_uframes_t’ and ‘int’
                            for (snd_pcm_uframes_t i = 0; i < temp; ++i)
                            {
                                int32_t sum = 0;
                                for (unsigned int c = 0; c < actual_channels; ++c)
                                {
                                    sum += src[i * actual_channels + c];
                                }
                                dst[i] = static_cast<int16_t>(sum / actual_channels);
                            }

                            size_t written = fwrite(dst, frame_size, rc, wav_file);
                            data_bytes_written += written * frame_size;
                            delete[] dst;
                        }
                    }
                }
            }
            if (s_enable_write_pcm.load())
            {
                // 循环结束后更新WAV头
                if (wav_file)
                {
                    fseek(wav_file, 0, SEEK_SET);
                    write_wav_header(wav_file, data_bytes_written, wav_sample_rate, wav_channels, wav_bits_per_sample);
                    fclose(wav_file);
                    RCLCPP_INFO(this->get_logger(), "Successfully saved %zu bytes to WAV file", data_bytes_written);
                }
            }
        }
    }

private:
    void SendTTSContent(std::string str)
    {
        auto str_to_tts = std_msgs::msg::String();
        str_to_tts.data = str;
        _tts_content_pub_ptr->publish(str_to_tts);
    }

    // 初始化ros通信接口
    // 初始化publisher侧的topic接口
    rclcpp::Publisher<interface::msg::AudioDataPackage>::SharedPtr _audio_data_pub_ptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _tts_content_pub_ptr;
    // 初始化subscriber侧的topic接口

    std::thread _thread_record;
    std::string _record_dev;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RecordNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}