#include "Microphone.hpp"

#include "Global.hpp"

std::shared_ptr<DequeTemplate<uint8_t>> g_ptr_byte_queue = std::make_shared<DequeTemplate<uint8_t>>();  // 与麦克风线程共用的容器指针，直接在堆上初始化这个容器
extern std::atomic<bool> g_asr_enabled;                                                                 // 全局变量，控制是否启用ASR

std::atomic<bool> global_audio_active(false);  // 控制音频处理激活状态
std::condition_variable cv_audio_active;       // 条件变量用于线程同步
std::mutex mtx_audio_active;                   // 互斥锁保护条件变量

void CheckOverflow(std::shared_ptr<DequeTemplate<uint8_t>> ptr, int max)
{
    if (ptr->Size() > max) {
        ptr->Clear();
    }
}

int receiveMicData(rclcpp::Logger logger)
{
    RCLCPP_INFO(logger, "receiveMicData() start!");

    snd_pcm_t *handle;
    snd_pcm_uframes_t frames;
    char *buffer;
    snd_pcm_hw_params_t *params;
    unsigned int val = 16000;

    signal(SIGINT, signal_handler_int);
    signal(SIGQUIT, signal_handler_quit);

    // 选择麦克风设备
    const char *device = nullptr;                                             // 存放麦克风设备的字符串
    std::string device_string = FindDeviceByName(global_record_device_name);  // 默认声卡设备
    device = device_string.c_str();
    RCLCPP_INFO(logger, "mic device = %s", device);

//     // 打开 WAV 文件
// #ifdef __x86_64__
//     std::ofstream wavFile("/home/wty/wl_ros_0304-tts-server-node/wl_ros/output.wav", std::ios::binary);
// #else
//     std::ofstream wavFile("/home/wl/bin/output.wav", std::ios::binary);
// #endif
//     if (!wavFile.is_open()) {
//         RCLCPP_ERROR(logger, "Failed to open WAV file for writing");
//         return -1;
//     }

    // 初始化WAV文件头
    WavHeader header;
    header.chunkSize = 0;                                                                   // 在最后设置
    header.numChannels = 1;                                                                 // 单声道
    header.sampleRate = 16000;                                                              // 采样率
    header.bitsPerSample = 16;                                                              // 16位样本
    header.byteRate = header.sampleRate * header.numChannels * (header.bitsPerSample / 8);  // byte率，每秒采集多少个byte
    header.blockAlign = header.numChannels * (header.bitsPerSample / 8);
    header.subchunk2Size = 0;  // 在最后设置

    // // 写入初始的 WAV 文件头
    // wavFile.write(reinterpret_cast<const char *>(&header), sizeof(WavHeader));

    while (g_run.load()) {
        // 打开麦克风设备
        // 如果打不开给定的麦克风设备，则一直阻塞在这里
        bool local_mic_found = false;
        int local_time_step = 2000;
        do {
            if (snd_pcm_open(&handle, device, SND_PCM_STREAM_CAPTURE, 0) < 0) {
                RCLCPP_ERROR_ONCE(logger, "can not open device %s", device);
            } else {
                local_mic_found = true;
                RCLCPP_INFO(logger, "open mic device successfully, device alias = %s", device);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(local_time_step));  // 阻塞时，检测是否能重连的时间间隔
            RCLCPP_DEBUG(logger, "looping per %d milliseconds... waiting for connection with mic device", local_time_step);
        } while (!local_mic_found);

        snd_pcm_hw_params_alloca(&params);                                                                                     // 分配硬件参数对象
        checkAlsaError(snd_pcm_hw_params_any(handle, params), "Cannot set default parameters");                                // 初始化硬件参数
        checkAlsaError(snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED), "Set access mode error");  // 设置访问类型
        checkAlsaError(snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE), "Set format error");               // 设置样本格式
        checkAlsaError(snd_pcm_hw_params_set_rate_near(handle, params, &val, 0), "Set sample rate error");                     // 设置采样率
        checkAlsaError(snd_pcm_hw_params_set_channels(handle, params, 1), "Set channels error");                               // 设置通道数
        checkAlsaError(snd_pcm_hw_params(handle, params), "Setting of hwparams failed");                                       // 应用硬件参数

        // int dir;
        // snd_pcm_hw_params_get_period_size(params, &frames, &dir);  // 获取缓冲区大小
        frames = 256;
        buffer = new char[frames * 2];  // 分配缓冲区，16-bit 样本，每个样本 2 字节

        // 准备录音
        checkAlsaError(snd_pcm_prepare(handle), "Prepare after underrun failed");

        // 线程开始时阻塞直到 global_audio_active 变为 true
        RCLCPP_INFO(logger, "check global_audio_active, waiting for Aliyun Api Connection... ");
        std::unique_lock<std::mutex> lk(mtx_audio_active);
        cv_audio_active.wait(lk, [] { return global_audio_active.load(); });
        RCLCPP_INFO(logger, "Aliyun Api Connected successfully, mic ready to collect data!");
        // 开始录制
        while (g_run.load()) {
            // 每次循环前再次检查 global_audio_active 的状态
            if (!global_audio_active.load()) {
                // 如果 global_audio_active 为 false，则再次等待条件变量通知
                cv_audio_active.wait(lk, [] { return !g_run.load() || global_audio_active.load(); });
                // 如果 g_run 已经变为 false，则直接退出循环
                if (!g_run.load()) {
                    RCLCPP_INFO(logger, "Loop will stop soon, for g_run = %s", g_run.load() ? "true" : "false");
                    break;
                }
            }

            // 读取音频数据
            // 就算g_asr_enabled == false也要读取，否则这些缓存仍然会在g_asr_enabled == true的时候被提取出来
            int ret = snd_pcm_readi(handle, buffer, frames);
            if (ret == -EPIPE) {
                // 溢出恢复
                RCLCPP_WARN(logger, "overflow occurred, recovering ...");
                snd_pcm_prepare(handle);
            } else if (ret < 0) {
                RCLCPP_WARN(logger, "read error: %d", ret);
                break;
            } else if (ret != static_cast<int>(frames)) {
                RCLCPP_WARN(logger, "short read, only read %d frames", ret);
            }

            if (g_asr_enabled.load()) {
                // 处理音频数据 (存入缓冲区用于语音识别)
                for (int i = 0; i < frames * 2; i++) {
                    g_ptr_byte_queue->Push(buffer[i]);
                }
            } else {
                const size_t num_bytes_to_fill = 512;
                for (size_t i = 0; i < num_bytes_to_fill; ++i) {
                    g_ptr_byte_queue->Push(0x00);
                }
            }

            // // 将音频数据写入 WAV 文件
            // wavFile.write(buffer, frames * 2);
            // header.subchunk2Size += frames * 2;

            // 局部计数器，每200次循环执行一次代码块
            static int counter = 0;
            if (++counter >= 200) {
                counter = 0;  // 重置计数器
                RCLCPP_DEBUG(logger, "Send byte to Aliyun Server... Global byte queue size = %d", g_ptr_byte_queue->Size());
            }
            CheckOverflow(g_ptr_byte_queue);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // // 更新 WAV 文件头
    // header.chunkSize = 36 + header.subchunk2Size;
    // wavFile.seekp(0, std::ios::beg);
    // wavFile.write(reinterpret_cast<const char *>(&header), sizeof(WavHeader));

    // 清理
    delete[] buffer;
    snd_pcm_close(handle);
    // wavFile.close();

    RCLCPP_INFO(logger, "receiveMicData() finished!");
    return 0;
}