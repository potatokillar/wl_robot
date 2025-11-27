#include <alsa/asoundlib.h>

#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "interface/msg/audio_data_package.hpp"
#include "robot_base/timer_tools.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"

#define debug_flag 0

// 定义队列和相关同步原语
std::queue<interface::msg::AudioDataPackage> global_audio_queue;
std::mutex global_audio_queue_mtx;
std::condition_variable global_audio_queue_cv;
std::atomic<bool> s_stop_thread = {false};
std::string global_alsa_device_name;
static unsigned int static_buffer_size = 1024;  // 音频设备缓冲区大小
static std::vector<int> static_alsa_device_idx;
static bool s_is_device_open = false;

using WavFileName = std::string;                           // 媒体文件的中文索引
using FilePath = std::string;                              // 媒体文件的相对路径
using WavMap = std::unordered_map<WavFileName, FilePath>;  // 媒体文件映射关系

static void FindDeviceByName2(const std::string &target_name)
{
    std::ifstream file("/proc/asound/cards");
    std::string line;
    int device_num = -1;
    // 逐行读取文件内容
    while (getline(file, line)) {
        // 找到包含目标设备名称的行
        if (line.find(target_name) != std::string::npos && line.find("[") != std::string::npos) {
            // 解析前面的序号
            std::istringstream iss(line);
            iss >> device_num;  // 第一个字段是序号
            if (device_num != -1) {
                static_alsa_device_idx.push_back(device_num);
            }
        }
    }
}

class SpeakerNode : public rclcpp::Node
{
public:
    SpeakerNode() : Node("speaker_node")
    {
        // 读取parameter，提供默认值以防止初始化失败
        std::vector<std::string> default_audio_dirs = {
            "/home/wl/autorun/iiri-ros/install/speech_recognition/share/speech_recognition/audio/",
            "install/speech_recognition/share/speech_recognition/audio/",
            "build_x86_shared/install/speech_recognition/share/speech_recognition/audio/",
            "build_arm_shared/install/speech_recognition/share/speech_recognition/audio/"
        };
        std::vector<std::string> default_wav_names = {
            "bo", "ding", "dog_bark2", "service_quit", "service_ready"
        };
        this->declare_parameter<std::vector<std::string>>("audio_dir_array", default_audio_dirs);
        this->declare_parameter<std::vector<std::string>>("wav_name", default_wav_names);
        this->get_parameter("audio_dir_array", _audio_dir_array);
        this->get_parameter("wav_name", _wav_file_array);
        SearchFiles();

        // 获取parameter，如果没获取到则使用默认值
#ifdef __x86_64__
        this->declare_parameter<std::string>("x86_play_device_name", "default_x86_device");
        global_alsa_device_name = this->get_parameter("x86_play_device_name").as_string();
#else
        this->declare_parameter<std::string>("arm_play_device_name", "default_arm_device");
        global_alsa_device_name = this->get_parameter("arm_play_device_name").as_string();
#endif

        // 初始化ros通信接口
        // 初始化publisher侧的topic接口
        _asr_control_pub_ptr = this->create_publisher<std_msgs::msg::String>("/asr/control", 10);
        _tts_content_pub_ptr = this->create_publisher<std_msgs::msg::String>("/tts/content", 10);
        _current_volume_pub_ptr = this->create_publisher<std_msgs::msg::UInt16>("speaker/current_volume", 10);
        // 初始化subscriber侧的topic接口
        _wav_file_sub_ptr = this->create_subscription<std_msgs::msg::String>("/speaker/wav_file", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            this->RxAudioFile(msg);
        });
        _audio_data_sub_ptr = this->create_subscription<interface::msg::AudioDataPackage>(
            "/speaker/audio_data",
            10,
            [this](const interface::msg::AudioDataPackage::SharedPtr msg) { this->RxAudioData(msg); });
        _iot_cmd_sub_ptr = this->create_subscription<std_msgs::msg::String>("/xiaozhi/iot_cmd", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
            this->RxIotCmd(msg);
        });

        _play_thread = std::thread(&SpeakerNode::Play, this);
        _pub_volume_thread = std::thread(&SpeakerNode::SendVolume, this);
    }

    // 播放线程函数
    void Play()
    {
        snd_pcm_t *handle;
        snd_pcm_hw_params_t *params;
        unsigned int rate = 16000;
        int dir;
        snd_pcm_uframes_t frames = static_buffer_size;
        const char *device = nullptr;  // 存放喇叭设备名的字符串

        // 最外部的工作循环，speaker节点要一直运行
        while (1) {
            // 打开播放设备
            // 如果打不开设备，则会一直阻塞在这里
            s_is_device_open = false;
            int local_time_step = 2000;  // 重复尝试的时间间隔
            do {
                FindDeviceByName2(global_alsa_device_name);  // 读取同名音频设备列表
                if (static_alsa_device_idx.size() != 0) {
#if debug_flag
                    {
                        RCLCPP_INFO(this->get_logger(), "static_alsa_device_idx.size() = %ld", static_alsa_device_idx.size());
                        int count = 0;
                        for (auto j = static_alsa_device_idx.begin(); j != static_alsa_device_idx.end(); j++) {
                            RCLCPP_INFO(this->get_logger(), "static_alsa_device_idx[%d] = %d", count, *j);
                            count++;
                        }
                    }
#endif
                    auto it = static_alsa_device_idx.begin();
                    std::string device_string;
                    while (it != static_alsa_device_idx.end()) {
                        device_string = "plughw:" + std::to_string(*it) + ",0";
                        device = device_string.c_str();
                        RCLCPP_INFO(this->get_logger(), "try to open PCM player device %s ...", device);
                        int ret = snd_pcm_open(&handle, device, SND_PCM_STREAM_PLAYBACK, 0);
                        if (ret < 0) {  // 返回值为0是打开成功，否则都是打开失败
                            RCLCPP_ERROR(this->get_logger(),
                                         "Failed to open PCM player device: %s, error: %s, try another device soon ...",
                                         device,
                                         snd_strerror(ret));
                            // 删除并获取下一个迭代器
                            it = static_alsa_device_idx.erase(it);
                        } else {
                            RCLCPP_INFO(this->get_logger(), "open PCM player device %s successfully", device);
                            s_is_device_open = true;
                            break;
                        }
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(local_time_step));  // 阻塞时，检测是否能重连的时间间隔
                RCLCPP_DEBUG(this->get_logger(), "looping per %d milliseconds... waiting for connection with player device", local_time_step);
            } while (!s_is_device_open);
#if debug_flag
            {
                RCLCPP_INFO(this->get_logger(), "static_alsa_device_idx.size() = %ld", static_alsa_device_idx.size());
                int count = 0;
                for (auto j = static_alsa_device_idx.begin(); j != static_alsa_device_idx.end(); j++) {
                    RCLCPP_INFO(this->get_logger(), "static_alsa_device_idx[%d] = %d", count, *j);
                    count++;
                }
            }
#endif

            // 分配硬件参数对象
            snd_pcm_hw_params_alloca(&params);
            snd_pcm_hw_params_any(handle, params);

            // 设置参数
            snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
            snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
            // 修改为单声道
            snd_pcm_hw_params_set_channels(handle, params, 1);
            snd_pcm_hw_params_set_rate_near(handle, params, &rate, &dir);
            snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

            // 将参数应用到 PCM 设备
            if (snd_pcm_hw_params(handle, params) < 0) {
                RCLCPP_ERROR(this->get_logger(), "无法设置 ALSA 参数");
                snd_pcm_close(handle);
                return;
            }
            // SendTTSContent("扬声器已开启");  // 语音播报

            // 开始播放的工作循环
            while (!s_stop_thread.load()) {
                // 解包AudioDataPackage数据包
                auto local_audio_pkg = interface::msg::AudioDataPackage();
                {
                    std::unique_lock<std::mutex> lock(global_audio_queue_mtx);
                    global_audio_queue_cv.wait(lock, [] { return !global_audio_queue.empty() || s_stop_thread.load(); });
                    if (s_stop_thread.load()) {
                        break;
                    }
                    local_audio_pkg = global_audio_queue.front();
                    global_audio_queue.pop();
                }

                if (local_audio_pkg.additional_info == "tts finish") {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    auto msg_to_asr = std_msgs::msg::String();
                    msg_to_asr.data = "start ASR";
                    _asr_control_pub_ptr->publish(msg_to_asr);
                } else {
                    std::vector<int16_t> audio_data;
                    audio_data = local_audio_pkg.data;

                    // 按照缓冲区的大小逐步输入给音频设备
                    // 将音频数据分块放入队列
                    for (size_t i = 0; i < audio_data.size(); i += static_buffer_size) {
                        std::vector<int16_t> chunk(audio_data.begin() + i, audio_data.begin() + std::min(i + static_buffer_size, audio_data.size()));
                        // 如果chunk的大小小于static_buffer_size，则将剩余部分填充为0x00
                        if (chunk.size() < static_buffer_size) {
                            size_t remaining = static_buffer_size - chunk.size();
                            chunk.insert(chunk.end(), remaining, 0x00);
                        }
                        {
                            // 播放音频数据
                            snd_pcm_sframes_t frames_written = snd_pcm_writei(handle, chunk.data(), chunk.size());
                            if (frames_written < 0) {
                                frames_written = snd_pcm_recover(handle, frames_written, 0);
                            }
                            if (frames_written < 0) {
                                RCLCPP_ERROR(this->get_logger(), "写入 ALSA 设备时出错");
                                s_stop_thread.store(true);
                                break;  // 一旦出错直接退出循环
                            }
                        }
                    }
                }
            }
            // 由于某些原因退出了播放循环，为了重试时能再次进入播放循环，所以这里要把s_stop_thread标志位恢复为false
            s_stop_thread.store(false);

            snd_pcm_drain(handle);
            snd_pcm_close(handle);
        }
    }

    // 处理 /speaker/wav_file 话题消息
    void RxAudioFile(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string wav_file_name = msg->data + ".wav";
        auto item = _wav_map.find(wav_file_name);
        std::string file_path = item->second;
        std::ifstream file(file_path, std::ios::binary);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "无法打开文件: %s", file_path.c_str());
            return;
        }

        // 跳过 WAV 文件头
        file.seekg(44, std::ios::beg);

        // 读取音频数据
        std::vector<int16_t> audio_data;
        uint16_t sample;
        while (file.read(reinterpret_cast<char *>(&sample), sizeof(sample))) {
            audio_data.push_back(sample);
        }

        // 组装AudioDataPackage数据包，准备放入队列
        auto local_audio_pkg = interface::msg::AudioDataPackage();
        local_audio_pkg.data = audio_data;
        local_audio_pkg.additional_info = wav_file_name;

        {
            std::lock_guard<std::mutex> lock(global_audio_queue_mtx);
            global_audio_queue.push(local_audio_pkg);
        }
        global_audio_queue_cv.notify_one();
    }

    // 处理 /speaker/audio_data 话题消息
    void RxAudioData(const interface::msg::AudioDataPackage::SharedPtr msg)
    {
        auto local_audio_pkg = interface::msg::AudioDataPackage();
        local_audio_pkg = *msg;
        {
            std::lock_guard<std::mutex> lock(global_audio_queue_mtx);
            global_audio_queue.push(local_audio_pkg);
        }
        global_audio_queue_cv.notify_one();
    }

    // 处理 /xiaozhi/iot_cmd 话题消息
    // 解析字符串里json格式的控制命令，判断是否属于本节点任务，如果不是则返回，如果是则执行
    void RxIotCmd(const std_msgs::msg::String::SharedPtr msg)
    {
        if (!s_is_device_open) {
            RCLCPP_WARN(this->get_logger(), "音量设置失败，未打开扬声器设备");
            return;
        }

        std::string iot_cmd_name = msg->data;
        std::string method, name;
        nlohmann::json j_str = nlohmann::json::parse(iot_cmd_name);
        nlohmann::json j;
        if (j_str.is_array()) {
            j = j_str.at(0);
        } else {
            j = j_str;
        }
#if debug_flag
        RCLCPP_INFO(this->get_logger(), "received iot_cmd_name: %s", iot_cmd_name.c_str());
        RCLCPP_INFO(this->get_logger(), "j_str: %s", j_str.dump().c_str());
        RCLCPP_INFO(this->get_logger(), "j: %s", j.dump().c_str());
#endif

        try {
            if (j.contains("method") && j.contains("name") && j.contains("parameters")) {
                method = j.at("method");
                name = j.at("name");
                if (method == "SetVolume" && name == "speaker") {
                    // 是speaker节点需要执行的调整音量的任务
                    int volume = j.at("parameters").at("volume");
                    std::string card_str;

                    if (!static_alsa_device_idx.empty()) {
                        card_str = "hw:" + std::to_string(*static_alsa_device_idx.begin());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "音量设置失败，设备index列表为空，未匹配到任何有效扬声器设备");
                        return;
                    }

                    const char *card = card_str.c_str();
                    snd_mixer_t *mixer = nullptr;
                    snd_mixer_selem_id_t *sid = nullptr;
                    long min, max;
                    int ret;

                    // 初始化音量ID
                    snd_mixer_selem_id_alloca(&sid);
                    snd_mixer_selem_id_set_index(sid, 0);
                    snd_mixer_selem_id_set_name(sid, "PCM");  // 主音量控制

                    ret = snd_mixer_open(&mixer, 0);                           // 打开混音器
                    ret |= snd_mixer_attach(mixer, card);                      // 附加到声卡
                    ret |= snd_mixer_selem_register(mixer, nullptr, nullptr);  // 注册混音器简单元素
                    ret |= snd_mixer_load(mixer);                              // 加载混音器元素
                    if (ret != 0) {
                        return;
                    }

                    // 查找音量控制
                    snd_mixer_elem_t *elem = snd_mixer_find_selem(mixer, sid);
                    if (!elem) {
                        RCLCPP_ERROR(this->get_logger(), "找不到音量控制");
                        snd_mixer_close(mixer);
                        return;
                    }

                    // 获取音量范围
                    snd_mixer_selem_get_playback_volume_range(elem, &min, &max);
                    // 设置音量 (将0-100的百分比转换为设备音量范围)
                    int new_volume;
                    if (volume < 1) {
                        new_volume = min;
                    } else if (volume > 99) {
                        new_volume = max;
                    } else {
                        new_volume = volume / 100.0 * (max - min);
                    }

#if debug_flag
                    RCLCPP_INFO(this->get_logger(), "支持的音量最小值: %ld, 音量最大值: %ld, new_volume: %d", min, max, new_volume);
#endif
                    if (snd_mixer_selem_set_playback_volume_all(elem, new_volume) < 0) {
                        RCLCPP_ERROR(this->get_logger(), "无法设置音量");
                        snd_mixer_close(mixer);
                        return;
                    }
                    _volume.store((u_int16_t)volume);

                    // 关闭混音器
                    snd_mixer_close(mixer);
                    return;
                } else {
                    // 不是speaker节点支持的任务，不做任何事，直接退出
                    RCLCPP_INFO(this->get_logger(), "并非属于speaker节点的任务");
                    return;
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "并不同时包含以下3个字段，请检查json内容: method, name, parameters");
            }
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "json parse error");
        }
    }

private:
    void SearchFiles()
    {
        for (const auto &wav_name : _wav_file_array) {
            std::string wav_filename = wav_name + ".wav";
            bool found = false;
            for (const auto &audio_dir : _audio_dir_array) {
                std::filesystem::path full_path = audio_dir;
                full_path /= wav_filename;
                if (std::filesystem::exists(full_path)) {
                    _wav_map[wav_filename] = full_path.string();
                    found = true;
                    RCLCPP_DEBUG(this->get_logger(), "file found. name: %s, path: %s", wav_filename.c_str(), full_path.string().c_str());
                    break;
                }
            }
            if (!found) {
                RCLCPP_DEBUG(this->get_logger(), "Error: File %s not found in any provided directories.", wav_filename.c_str());
            }
        }
    }

    void SendTTSContent(std::string str)
    {
        auto str_to_tts = std_msgs::msg::String();
        str_to_tts.data = str;
        _tts_content_pub_ptr->publish(str_to_tts);
    }

    void SendVolume()
    {
        while (true) {
            TimerTools::SleepForS(5);
            std_msgs::msg::UInt16 msg;
            msg.data = _volume.load();
            _current_volume_pub_ptr->publish(msg);
            // RCLCPP_INFO(this->get_logger(), "speaker节点，发布当前音量值: %d", msg.data);
        }
    }

    // topic通信接口
    // 发布端
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _asr_control_pub_ptr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _tts_content_pub_ptr;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr _current_volume_pub_ptr;  // 发布当前音量的百分比，范围是从0到100的整数
    // 接收端
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _wav_file_sub_ptr;
    rclcpp::Subscription<interface::msg::AudioDataPackage>::SharedPtr _audio_data_sub_ptr;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _iot_cmd_sub_ptr;  // 接收控制xiaozhi大模型开启关闭的指令

    std::thread _play_thread;
    std::thread _pub_volume_thread;
    std::vector<std::string> _audio_dir_array;
    std::vector<std::string> _wav_file_array;
    WavMap _wav_map;

    std::atomic<u_int16_t> _volume = {0};  // 当前的音量值
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<SpeakerNode>();
    // node_ptr->get_logger().set_level(rclcpp::Logger::Level::Debug);  // 开启日志的debug级别

    rclcpp::spin(node_ptr);

    rclcpp::shutdown();
    return 0;
}