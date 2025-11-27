/*
 * @Author: 唐文浩
 * @Date: 2024-10-17
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-04
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "speech_recognition.hpp"

#include "AliyunApi.hpp"
#include "CommandExtract.hpp"
#include "Microphone.hpp"
#include "robot_base/timer_tools.hpp"

extern std::shared_ptr<moodycamel::ReaderWriterQueue<std::string>> g_ptr_stt_queue;
extern std::shared_ptr<DequeTemplate<std::string>> g_ptr_cmd_queue; // 存放待执行的指令的容器指针
// 存放传递给deepseek的字符串的队列和互斥锁
std::deque<std::string> g_deepseek_msg;
std::mutex g_deepseek_msg_mutex;
std::condition_variable g_queue_cv;
/** @details 全局变量，控制是否启用ASR */
std::atomic<bool> g_asr_enabled = {true};
extern std::shared_ptr<DequeTemplate<uint8_t>> g_ptr_byte_queue; // 存放音频数据的容器

static std::deque<std::vector<int16_t>> local_audio_data_package_cache; // 接收AudioDataPackage的缓冲队列
static std::mutex local_mtx_cache;

// static std::atomic<uint64_t> _trigger_timestamp = {0};  // 记录收到Trigger信号的时间点
static std::atomic<bool> _cmd_ready = {false}; // 有语音指令准备执行的标志位
static uint64_t _cmd_start_timestamp = 0;      // 开始执行语音指令的时间点

static std::mutex _mtx_trigger; // trigger回调函数的锁
static std::condition_variable _cv_trigger;
static std::atomic<bool> _trigger_received = {false};

static std::mutex _mtx_cmd; // 语音指令执行时的锁
static std::condition_variable _cv_cmd;
static std::atomic<bool> _cmd_step_executed = {false}; // 语音指令的step已完成
static std::atomic<bool> _xiaozhi_enable = {false};    // 是否开启xiaozhi

static std::atomic<bool> _gamepad_takeover = {false};

// DogMoveStruct目前只支持1个成员为非零值
struct DogMoveStruct
{
    double vel_x = 0;
    double vel_y = 0;
    double vel_z = 0;
    double ang_x = 0;
    double ang_y = 0;
    double ang_z = 0;
};

static void AudioDataPackageCacheProcess()
{
    while (1)
    {
        if (!local_audio_data_package_cache.empty())
        {
            std::vector<int16_t> audio_data_vector = local_audio_data_package_cache.front();
            local_audio_data_package_cache.pop_front();
            for (int i = 0; i < audio_data_vector.size(); i++)
            {
                int16_t item = audio_data_vector.at(i);
                uint8_t byte1 = static_cast<uint8_t>((item >> 8) & 0xFF);
                uint8_t byte2 = static_cast<uint8_t>(item & 0xFF);
                // S16_LE格式是小端序，所以后面的字节要先存放进去
                g_ptr_byte_queue->Push(byte2);
                g_ptr_byte_queue->Push(byte1);
            }
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
}

#define WAVPLAY_ENABLE 0
#if WAVPLAY_ENABLE
WavPlay::WavPlay(const std::string &text_to_search)
{
    auto it = media_file_map.find(text_to_search);
    if (it != media_file_map.end())
    { // 找到了音频文件
        index_text_ = it->first;
        file_ = it->second;
        // Play();  // 播放该音频文件
        ShellPlay(); // 播放该音频文件
    }
    else
    {
        std::cerr << "MediaIndexText not found in the map." << std::endl;
        index_text_ = "";
        file_ = "";
    }
}

void WavPlay::Play()
{
    try
    {
        std::ifstream wavFile(file_, std::ios::binary | std::ios::ate);
        if (!wavFile)
        {
            throw std::runtime_error("Can not found WAV file.");
        }

        // 获取文件大小
        std::streamsize fileSize = wavFile.tellg();
        if (fileSize < sizeof(WavHeader))
        {
            throw std::runtime_error("WAV file is too small.");
        }

        // 重置文件指针到开始位置
        wavFile.seekg(0, std::ios::beg);

        WavHeader header;
        wavFile.read(reinterpret_cast<char *>(&header), sizeof(WavHeader));

        if (wavFile.gcount() != sizeof(WavHeader) || memcmp(header.riff, "RIFF", 4) != 0 || memcmp(header.wave, "WAVE", 4) != 0 ||
            memcmp(header.fmt, "fmt ", 4) != 0 || header.audioFormat != 1 || // Only support PCM
            header.bitsPerSample != 16)
        { // Assume 16-bit samples
            throw std::runtime_error("Unsupported or invalid WAV format.");
        }

        snd_pcm_t *handle;
        std::string device_string = FindDeviceByName(global_play_device_name);
        checkAlsaError(snd_pcm_open(&handle, device_string.c_str(), SND_PCM_STREAM_PLAYBACK, 0), "Playback open error");

        snd_pcm_hw_params_t *params;
        snd_pcm_hw_params_alloca(&params);
        checkAlsaError(snd_pcm_hw_params_any(handle, params), "Cannot set default parameters");
        // unsigned int rate = header.sampleRate;
        checkAlsaError(snd_pcm_hw_params_set_rate_near(handle, params, &header.sampleRate, 0), "Set sample rate error");
        checkAlsaError(snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED), "Set access mode error");
        checkAlsaError(snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE), "Set format error");
        checkAlsaError(snd_pcm_hw_params_set_channels(handle, params, header.numChannels), "Set channels error");
        checkAlsaError(snd_pcm_hw_params(handle, params), "Setting of hwparams failed");

        size_t frame_size = header.bitsPerSample / 8 * header.numChannels;
        const size_t buffer_frames = 2048;                     // 每次读取的帧数
        const size_t buffer_size = buffer_frames * frame_size; // 缓冲区大小
        char buffer[buffer_size];

        while (true)
        {
            wavFile.read(buffer, buffer_size);
            size_t bytesRead = wavFile.gcount();
            if (bytesRead == 0)
                break;

            size_t frames_to_deliver = bytesRead / frame_size;
            size_t total_frames_written = 0;

            while (total_frames_written < frames_to_deliver)
            {
                ssize_t frames_written = snd_pcm_writei(handle, buffer + total_frames_written * frame_size, frames_to_deliver - total_frames_written);

                if (frames_written < 0)
                {
                    if (frames_written == -EPIPE)
                    {
                        // EPIPE means underrun
                        std::cerr << "Underrun occurred, preparing device..." << std::endl;
                        checkAlsaError(snd_pcm_prepare(handle), "Prepare after underrun failed");
                        continue;
                    }
                    else
                    {
                        throw std::runtime_error("Write to device error: " + std::string(snd_strerror(frames_written)));
                    }
                }

                total_frames_written += static_cast<size_t>(frames_written);
            }
        }

        snd_pcm_drain(handle);
        snd_pcm_close(handle);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void WavPlay::ShellPlay()
{
    // 构造 FFplay 命令行参数。我们使用 -nodisp 来确保没有显示窗口弹出，
    // 并且使用 -autoexit 在播放结束后自动退出。
    std::string command = "ffplay -nodisp -autoexit -loglevel quiet \"" + file_ + "\"";

    // 调用 system() 执行命令。这里假设 file_ 中的路径是有效的，并且文件存在。
    // 注意：如果路径或文件名包含空格，则需要使用引号括起来，如上所示。
    system(command.c_str());
}
#endif

AliveTimer::AliveTimer() { Open(); }

AliveTimer::~AliveTimer() { Close(); }

void AliveTimer::Open()
{
    now_.store(duration_);
    if (!valid_.load())
    {
        timer_ = std::move(std::thread(&AliveTimer::Running, this));
        timer_.detach();
        valid_.store(true);
    }
}

void AliveTimer::Close() { valid_.store(false); }

void AliveTimer::Running()
{
    while (now_.load() > 0 && g_enable_deepseek.load())
    {
        /*DEBUG*/ RCLCPP_DEBUG(g_node_ptr->get_logger(), "now_ = %d", now_.load());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        now_.fetch_sub(1);
    }
    // 这里是完全可以将Close()替换为valid_.store(false)的。
    // 独立成Close()为了封装(未来可能存在的)更复杂的关闭逻辑
    Close();
}

bool AliveTimer::Check() { return valid_.load(); }

void AliveTimer::Reset() { Open(); }

speech_recognition::speech_recognition() : Node("speech_recognition_node")
{
    // 语音指令映射表
    // 移动指令
    AddSpeechCmd({"前进", "向前", "向前走", "往前走", "直走"}, [this]()
                 { this->SayDogMoveTrigger({1.0, 0, 0, 0, 0, 0}, 2000); });
    AddSpeechCmd({"后退", "向后", "向后走", "往后走"}, [this]()
                 { this->SayDogMoveTrigger({-1.0, 0, 0, 0, 0, 0}, 2000); });
    AddSpeechCmd({"向左平移", "向左移动", "左走一步", "左移一步"}, [this]()
                 { this->SayDogMoveTrigger({0, 1.0, 0, 0, 0, 0}, 2000); });
    AddSpeechCmd({"向右平移", "向右移动", "右走一步", "右移一步"}, [this]()
                 { this->SayDogMoveTrigger({0, -1.0, 0, 0, 0, 0}, 2000); });
    AddSpeechCmd({"左转", "向左转"}, [this]()
                 { this->SayDogMoveTrigger({0, 0, 0, 0, 0, 1.5}, 2000); });
    AddSpeechCmd({"右转", "向右转"}, [this]()
                 { this->SayDogMoveTrigger({0, 0, 0, 0, 0, -1.5}, 2000); });
    // 状态切换
    AddSpeechCmd({"起立", "站起来", "站立", "启立", "启力"}, [this]()
                 { this->SayStandUpTrigger(); });
    AddSpeechCmd({"趴下", "坐下", "休息"}, [this]()
                 { this->SayLieDownTrigger(); });
    // AddSpeechCmd({"进入跟随模式", "进入智能跟随", "开启跟随模式", "开启智能跟随", "打开跟随模式", "打开智能跟随", "跟着我"},
    //              [this]() { this->SaySmartFollowTrigger(true); });
    // AddSpeechCmd({"退出跟随模式", "退出智能跟随", "停止跟随模式", "停止智能跟随", "关闭跟随模式", "关闭智能跟随", "不用跟随", "不用跟着"},
    //              [this]() { this->SaySmartFollowTrigger(false); });
    // 表演动作
    AddSpeechCmd({"握手", "握握手", "握个手", "打招呼", "打个招呼", "问好"}, [this]()
                 { this->SayDanceActionTrigger("ShakeHand"); });
    AddSpeechCmd({"拜年"}, [this]()
                 { this->SayDanceActionTrigger("HappyNewYear"); });
    AddSpeechCmd({"欢乐跳"}, [this]()
                 { this->SayDanceActionTrigger("HappyJump"); });
    AddSpeechCmd({"太空步", "太空部", "太空布", "太恐怖"}, [this]()
                 { this->SayDanceActionTrigger("MoonWalk"); });
    AddSpeechCmd({"向前跳", "跳一下", "往前跳"}, [this]()
                 { this->SayDanceActionTrigger("JumpForward"); });
    AddSpeechCmd({"俯卧撑"}, [this]()
                 { this->SayDanceActionTrigger("PushUp"); });
    AddSpeechCmd({"跳舞", "跳个舞"}, [this]()
                 { this->SayDanceActionTrigger("DanceApplause"); });
    // 语音播报
    // AddWakeAndSpeechCmd({"播报", "语音播报"}, [this]() { this->SayBeware(); });
    // AddSpeechCmd({"开启语音播报", "打开语音播报", "开始语音播报"}, [this]() { this->SayBeware(); });
    // 聊天大模型
    // AddSpeechCmd({"你好小智"}, [this]() { this->SayEnableXiaozhi(true); });
    // AddSpeechCmd({"小智关机", "关闭小智"}, [this]() { this->SayEnableXiaozhi(false); });

    // 初始化回调组
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 初始化ros通信接口
    // 初始化publisher侧的topic接口
    _run_state_pub_ptr = this->create_publisher<interface::msg::RunState>("/motion_control/cmd_run_state", 10);
    _vel_pub_ptr = this->create_publisher<geometry_msgs::msg::Twist>("/motion_control/cmd_vel", 10); // 速度topic
    _deepseek_msg_pub_ptr = this->create_publisher<std_msgs::msg::String>("/asr/request_deepseek", 10);
    _tts_content_pub_ptr = this->create_publisher<std_msgs::msg::String>("/tts/content", 10);
    _wav_file_pub_ptr = this->create_publisher<std_msgs::msg::String>("/speaker/wav_file", 10);
    _xiaozhi_cmd_pub_ptr = this->create_publisher<std_msgs::msg::String>("/xiaozhi/cmd", 10);
    // _smart_follow_pub_ptr = this->create_publisher<std_msgs::msg::Bool>("/smart_follow/open", 10);
    // 初始化subscriber侧的topic接口
    // _run_state_sub_ptr = create_subscription<interface::msg::RunState>("/motion_control/cmd_run_state",
    //                                                                    10,
    //                                                                    [this](const interface::msg::RunState::SharedPtr msg) { this->CallbackRunState(msg);
    //                                                                    });
    _asr_control_sub_ptr = this->create_subscription<std_msgs::msg::String>("/asr/control", 10, [this](const std_msgs::msg::String::SharedPtr msg)
                                                                            { this->CallbackASRControl(msg); });
    _audio_data_sub_ptr =
        this->create_subscription<interface::msg::AudioDataPackage>("/record/audio_data", 10, [this](const interface::msg::AudioDataPackage::SharedPtr msg)
                                                                    { this->CallbackRecordAudioData(msg); });
    _xiaozhi_stt_sub_ptr = this->create_subscription<std_msgs::msg::String>("/xiaozhi/stt", 10, [this](const std_msgs::msg::String::SharedPtr msg)
                                                                            { this->CallbackXiaoZhiSTT(msg); });
    _gamepad_sub_ptr =
        this->create_subscription<interface::msg::GamepadCmd>("/motion_control/ret_gamepad_cmd", 10, [this](const interface::msg::GamepadCmd::SharedPtr msg)
                                                              { this->CallbackGamepad(msg); });
    // 初始化client侧的service接口
    _dog_move_cli_ptr = this->create_client<interface::srv::DogMove>("/dog_move", rmw_qos_profile_services_default, client_cb_group_);
    _dance_cli_ptr = this->create_client<interface::srv::Dance>("/dance", rmw_qos_profile_services_default, client_cb_group_);
    // 初始化server侧的service接口
    _trigger_srv_ptr = this->create_service<std_srvs::srv::Trigger>(
        "/speech_recognition/bt_trigger",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            this->CallbackTrigger(request, response);
        },
        rmw_qos_profile_services_default,
        service_cb_group_);

    // 初始化工作线程
    _thread = std::thread(&speech_recognition::Loop, this);
    // _send_message_thread = std::thread(&speech_recognition::TxDeepseekMsgLoop, this);
    _cache_thread = std::thread(AudioDataPackageCacheProcess);

    /*DEBUG*/
    if (_send_message_thread.joinable())
    {
        RCLCPP_INFO(this->get_logger(), "TxDeepseekMsgLoop thread started successfully.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to start TxDeepseekMsgLoop thread.");
    }

    // 获取parameter，选择音频设备
#ifdef __x86_64__
    this->declare_parameter<std::string>("x86_play_device_name", "PCH");
    this->declare_parameter<std::string>("x86_record_device_name", "UACDemoV10");
    global_play_device_name = this->get_parameter("x86_play_device_name").as_string();
    global_record_device_name = this->get_parameter("x86_record_device_name").as_string();
#else
    this->declare_parameter<std::string>("play_device_name", "UACDemoV1.0");
    this->declare_parameter<std::string>("record_device_name", "USB Audio Device");
    global_play_device_name = this->get_parameter("play_device_name").as_string();
    global_record_device_name = this->get_parameter("record_device_name").as_string();
#endif
    // 读取参数，是否将音频数据写入到1个pcm文件中
    this->declare_parameter<int>("enable_write_pcm", 0);
    if (this->get_parameter("enable_write_pcm").as_int() == 1)
    {
        g_enable_write_pcm.store(true);
    }
    else
    {
        g_enable_write_pcm.store(false);
    }
}

speech_recognition::~speech_recognition()
{
    if (_thread.joinable())
        _thread.join();
    if (_send_message_thread.joinable())
        _send_message_thread.join();
}

std::set<std::string> speech_recognition::GetCommandTextSet()
{
    std::set<std::string> ret;
    // 遍历commandSet中的每个键值对，提取键（即第一个字符串），转换并存储到宽字符set中
    for (const auto &item_set : _cmd_map)
    {
        for (const auto &item_string : item_set.first)
        {
            ret.insert(item_string);
        }
    }
    return ret;
}

void speech_recognition::AddSpeechCmd(std::initializer_list<std::string> list, CommandContent func)
{
    CommandText text(list.begin(), list.end());
    _cmd_map.insert({text, func});
}

void speech_recognition::AddWakeAndSpeechCmd(std::initializer_list<std::string> list, CommandContent func)
{
    CommandText added_head_text;
    for (const auto &str : list)
    {
        added_head_text.insert(cmd_head_ + str);
    }
    _cmd_map.insert({added_head_text, func});
}

void speech_recognition::SayDogMoveTrigger(DogMoveStruct dog_move_cmd, int time)
{
    _cmd_ready.store(true); // 是否有语音指令正在执行的标志位，设置为true
    // _cmd_start_timestamp = TimerTools::GetNowTickMs();
    // _trigger_timestamp.store(_cmd_start_timestamp);
    // /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "_cmd_start_timestamp = %d", _cmd_start_timestamp);
    // /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "_trigger_timestamp = %d", _trigger_timestamp.load());
    // /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "TimerTools::GetNowTickMs, line: %d = %d", __LINE__, TimerTools::GetNowTickMs());

    // 语音播报
    if (dog_move_cmd.vel_x > 0)
    {
        SendSpeakerWavFile("dog_bark2");
    }
    else if (dog_move_cmd.vel_x < 0)
    {
        SendSpeakerWavFile("dog_bark2");
    }
    else if (dog_move_cmd.vel_y < 0)
    {
        SendSpeakerWavFile("dog_bark2");
    }
    else if (dog_move_cmd.vel_y > 0)
    {
        SendSpeakerWavFile("dog_bark2");
    }
    else if (dog_move_cmd.ang_z > 0)
    {
        SendSpeakerWavFile("dog_bark2");
    }
    else if (dog_move_cmd.ang_z < 0)
    {
        SendSpeakerWavFile("dog_bark2");
    }

    // 初始化待发送的消息内容
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = dog_move_cmd.vel_x;
    message.linear.y = dog_move_cmd.vel_y;
    message.linear.z = dog_move_cmd.vel_z;
    message.angular.x = dog_move_cmd.ang_x;
    message.angular.y = dog_move_cmd.ang_y;
    message.angular.z = dog_move_cmd.ang_z;

    _cmd_start_timestamp = TimerTools::GetNowTickMs();
    /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "_cmd_start_timestamp = %d", _cmd_start_timestamp);

    bool l_need_run_state = true;
    while (TimerTools::GetNowTickMs() < _cmd_start_timestamp + time)
    {
        if (_gamepad_takeover.load())
        {
            break;
        }
        // /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "TimerTools::GetNowTickMs, line: %d = %d", __LINE__, TimerTools::GetNowTickMs());
        _cmd_step_executed.store(false);             // 指令step未执行
        std::unique_lock<std::mutex> lock(_mtx_cmd); // 阻塞
        _cv_cmd.wait(lock, []
                     { return _trigger_received.load(); }); // 等待收到新的trigger
        // /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "_cv_cmd wait finish");
        if (l_need_run_state)
        {
            auto message_runstate = interface::msg::RunState();
            message_runstate.value = interface::msg::RunState::STAND;
            TimerTools::SleepForMs(50);
            _run_state_pub_ptr->publish(message_runstate);
            message_runstate.value = interface::msg::RunState::WALK;
            TimerTools::SleepForMs(50);
            _run_state_pub_ptr->publish(message_runstate);
            TimerTools::SleepForMs(50);
            l_need_run_state = false;
        }
        _vel_pub_ptr->publish(message); // 执行step内容
        /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "publish velocity topic. timestamp %d ms", TimerTools::GetNowTickMs());
        _cmd_step_executed.store(true); // 指令的当前step已执行完
        _cv_trigger.notify_all();       // 通知trigger回调函数，可以返回了
        TimerTools::SleepForMs(50);
    }

    if (!_gamepad_takeover.load())
    {
        // 发送速度0的消息
        auto zero_velocity_msg = geometry_msgs::msg::Twist();
        zero_velocity_msg.linear.x = 0;
        zero_velocity_msg.linear.y = 0;
        zero_velocity_msg.linear.z = 0;
        zero_velocity_msg.angular.x = 0;
        zero_velocity_msg.angular.y = 0;
        zero_velocity_msg.angular.z = 0;
        _vel_pub_ptr->publish(zero_velocity_msg);
        RCLCPP_DEBUG(this->get_logger(), "publish zero velocity topic. timestamp %d ms", TimerTools::GetNowTickMs());
    }

    _cmd_ready.store(false); // 当前指令执行完了，所以不需要捕获trigger了，trigger会走另外一个分支
}

void speech_recognition::SayStandUpTrigger()
{
    _cmd_ready.store(true); // 是否有语音指令正在执行的标志位，设置为true
    _cmd_start_timestamp = TimerTools::GetNowTickMs();

    _cmd_step_executed.store(false);             // 指令step未执行
    std::unique_lock<std::mutex> lock(_mtx_cmd); // 阻塞
    _cv_cmd.wait(lock, []
                 { return _trigger_received.load(); }); // 等待收到新的trigger
    {
        // 把播放任务传递给speaker节点
        SendSpeakerWavFile("dog_bark2");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto message = interface::msg::RunState();

        message.value = interface::msg::RunState::STAND;
        _run_state_pub_ptr->publish(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        message.value = interface::msg::RunState::WALK;
        _run_state_pub_ptr->publish(message);

        // 为了等待动作完成而加一个等待时长，
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        message.value = interface::msg::RunState::WALK;
        _run_state_pub_ptr->publish(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    _cmd_step_executed.store(true); // 指令的当前step已执行完
    _cv_trigger.notify_all();       // 通知trigger回调函数，可以返回了

    _cmd_ready.store(false); // 当前指令执行完了，所以不需要捕获trigger了，trigger会走另外一个分支
}

void speech_recognition::SayLieDownTrigger()
{
    _cmd_ready.store(true); // 是否有语音指令正在执行的标志位，设置为true
    _cmd_start_timestamp = TimerTools::GetNowTickMs();

    _cmd_step_executed.store(false);             // 指令step未执行
    std::unique_lock<std::mutex> lock(_mtx_cmd); // 阻塞
    _cv_cmd.wait(lock, []
                 { return _trigger_received.load(); }); // 等待收到新的trigger
    {
        SendSpeakerWavFile("dog_bark2");
        auto message = interface::msg::RunState();
        // 在这里增加一个WALK状态的设置，解决了有时候趴下没响应的问题。但是原因不太清楚。
        message.value = interface::msg::RunState::WALK;
        _run_state_pub_ptr->publish(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));

        message.value = interface::msg::RunState::STAND;
        _run_state_pub_ptr->publish(message);
        std::this_thread::sleep_for(std::chrono::milliseconds(600));

        message.value = interface::msg::RunState::LIE;
        _run_state_pub_ptr->publish(message);

        // 为了等待动作完成而加一个等待时长，
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
    _cmd_step_executed.store(true); // 指令的当前step已执行完
    _cv_trigger.notify_all();       // 通知trigger回调函数，可以返回了

    _cmd_ready.store(false); // 当前指令执行完了，所以不需要捕获trigger了，trigger会走另外一个分支
}

void speech_recognition::SayDanceActionTrigger(const std::string &dance_name)
{
    _cmd_ready.store(true); // 是否有语音指令正在执行的标志位，设置为true
    _cmd_start_timestamp = TimerTools::GetNowTickMs();

    // 初始化待发送的消息内容
    auto request = std::make_shared<interface::srv::Dance::Request>(); // 初始化1条request
    request->name = dance_name;

    _cmd_step_executed.store(false);             // 指令step未执行
    std::unique_lock<std::mutex> lock(_mtx_cmd); // 阻塞
    _cv_cmd.wait(lock, []
                 { return _trigger_received.load(); }); // 等待收到新的trigger
    {
        // 执行step内容
        auto result = _dance_cli_ptr->async_send_request(request);
        SendSpeakerWavFile("dog_bark2");
        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        // 创建线程进行语音播放
        // std::thread thread_wav_play;
        // SendSpeakerWavFile("dog_bark2");
        // auto response_result = result.get()->success;
        // RCLCPP_INFO(this->get_logger(), "result: %d", response_result);
    }
    _cmd_step_executed.store(true); // 指令的当前step已执行完
    _cv_trigger.notify_all();       // 通知trigger回调函数，可以返回了

    // _trigger_block.store(false);

    _cmd_ready.store(false); // 当前指令执行完了，所以不需要捕获trigger了，trigger会走另外一个分支
}

void speech_recognition::SayBeware()
{
    SendSpeakerWavFile("beware");
    SendSpeakerWavFile("beware");
    SendSpeakerWavFile("beware");
    SendSpeakerWavFile("beware");
    SendSpeakerWavFile("beware");
}

void speech_recognition::SayEnableXiaozhi(bool cmd)
{
    /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "SayEnableXiaozhi, cmd: %s", cmd ? "TRUE" : "FALSE");
    auto msg = std_msgs::msg::String();
    if (cmd)
    {
        msg.data = "start";
    }
    else
    {
        msg.data = "stop";
    }
    _xiaozhi_enable.store(cmd);
    _xiaozhi_cmd_pub_ptr->publish(msg);
}

// void speech_recognition::SaySmartFollowTrigger(bool cmd)
// {
//     _cmd_ready.store(true);
//     _cmd_start_timestamp = TimerTools::GetNowTickMs();
//     std_msgs::msg::Bool msg;
//     if (cmd == true) {
//         msg.data = true;
//     } else {
//         msg.data = false;
//     }

//     _cmd_step_executed.store(false);
//     std::unique_lock<std::mutex> lock(_mtx_cmd);
//     _cv_cmd.wait(lock, [] { return _trigger_received.load(); });
//     {
//         _smart_follow_pub_ptr->publish(msg);
//         if (cmd == true) {
//             RCLCPP_DEBUG(this->get_logger(), "speech recognition send OPEN command to smart follow node.");
//         } else {
//             RCLCPP_DEBUG(this->get_logger(), "speech recognition send CLOSE command to smart follow node.");
//         }
//     }
//     _cmd_step_executed.store(true);
//     _cv_trigger.notify_all();

//     _cmd_ready.store(false);
// }

void speech_recognition::Loop()
{
    RCLCPP_INFO(this->get_logger(), "speech_recognition::Loop() start!");
    while (g_run.load())
    {
        // 从队列中获取命令
        auto command = g_ptr_cmd_queue->PopWithTimeout(std::chrono::milliseconds(100));

        // g_pause为true时，忽略所有指令
        if (g_pause.load())
        {
            continue;
        }

        if (command.has_value())
        { // 查找并执行对应的命令
            for (const auto &item_pair : _cmd_map)
            {
                auto set_to_search = item_pair.first;
                auto functional_to_exec = item_pair.second;
                auto it = set_to_search.find(command.value());
                if (it != set_to_search.end())
                {
                    RCLCPP_INFO(this->get_logger(), "execute command: %s", it->c_str());
                    functional_to_exec(); // 执行对应的函数对象
                }
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "speech_recognition::Loop() finished!");
}

// /** @brief 存放发送给deepseek的内容的字符串。将拼接结果临时存放在这里。发送后要清空 */
// static std::string local_msg_to_deepseek = "";
// /** @brief 1个倒计时计时器对象 */
// static std::shared_ptr<AliveTimer> local_ptr_timer;
/** @brief 给deepseek发送topic的loop函数 */
void speech_recognition::TxDeepseekMsgLoop()
{
    /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "enter TxDeepseekMsgLoop()");

    /** @details
     * (while)开始
     * 判断是否开启了深度思考模式
     *   - 如果开启了，初始化1个计时器
     *   - (while)判断计时器是否结束
     *       - 判断是否有新字符串可用
     *           - 如果有新字符串可用，拼接
     *           - 如果无字符串可用，则sleep然后结束本次step
     *       - 如果计时器结束，将字符串发送出去，关闭ASR
     *   - 如果没开启，则sleep然后结束本次step
     *
     * while true循环
     * 判断是否处于深度思考模式
     *     处于深度思考模式。创建计时器local_ptr_timer，创建字符串total_msg存储用户发言
     *
     *
     */
    while (true)
    {
        if (g_enable_deepseek.load())
        {
            if (g_llm_act_mode.load() == 0)
            { // 多轮发言模式
                std::shared_ptr<AliveTimer> local_ptr_timer = std::make_shared<AliveTimer>();
                std::string total_msg = {}; // 存放本轮用户发言
                TimerTools::SleepForMs(50);
                while (local_ptr_timer->Check())
                {
                    if (!g_enable_deepseek.load())
                    {                             // 如果此时已关闭深度思考模式，则退出这个循环
                        local_ptr_timer->Close(); // 关闭计时器
                        break;
                    }
                    if (!g_deepseek_msg.empty())
                    {
                        std::unique_lock<std::mutex> lock(g_deepseek_msg_mutex);
                        std::string this_step_message = g_deepseek_msg.front();
                        g_deepseek_msg.pop_front();
                        lock.unlock();
                        local_ptr_timer->Reset(); // 刷新计时器剩余时长
                        total_msg.append(this_step_message);
                        /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "获取字符串: %s， 当前用户发言: %s", this_step_message.c_str(), total_msg.c_str());
                    }
                    else
                    {
                        TimerTools::SleepForMs(50);
                    }
                }
                // 只有计时器触发了，也就是到达预定时间了，才会走出while循环来到下面的if结构
                // 如果本轮用户发言非空，而且正处于深度思考模式，则发送用户发言给deepseek api
                if (!total_msg.empty() && g_enable_deepseek.load())
                {
                    auto msg = std_msgs::msg::String();
                    msg.data = total_msg;
                    _deepseek_msg_pub_ptr->publish(msg);
                    /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "send a msg to Deepseek Node ... ");

                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    std::unique_lock<std::mutex> lock(g_deepseek_msg_mutex);
                    g_deepseek_msg.clear();
                }
            }
            else if (g_llm_act_mode.load() == 1)
            { // 单句发言模式
                if (!g_deepseek_msg.empty())
                {
                    std::string messages_to_send;
                    {
                        // 通过锁保护队列操作，一次性取出所有消息
                        std::lock_guard<std::mutex> lock(g_deepseek_msg_mutex);
                        messages_to_send = g_deepseek_msg.front();
                        g_deepseek_msg.pop_front();
                    }

                    // 发送所有取出的消息
                    auto msg = std_msgs::msg::String();
                    msg.data = messages_to_send;
                    _deepseek_msg_pub_ptr->publish(msg);
                    RCLCPP_DEBUG(this->get_logger(), "单句模式发送消息: %s", messages_to_send.c_str());
                }
                else
                {
                    TimerTools::SleepForMs(50);
                }
            }
        }
        else
        {
            // /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "深度思考模式未开启");
            TimerTools::SleepForMs(50);
        }
    }

    // while (true) {
    //     std::unique_lock<std::mutex> lock(g_deepseek_msg_mutex);
    //     g_queue_cv.wait(lock, [] { return !g_deepseek_msg.empty(); });
    //     local_ptr_timer->Open();
    //     std::string message = g_deepseek_msg.front();
    //     g_deepseek_msg.pop_front();
    //     /** @details 如果处于深度思考模式，则向deepseek api节点发送topic，否则不发送 */
    //     /** @todo - 在1次交互中，使用者说完了，计时器也触发了，这些内容要发送给deepseek节点了，从这个时刻开始
    //      * 一直到tts播报回复完成的时刻，在这个时间区间范围内，ASR还要不要识别
    //      * 目前的处理方式，不识别，这个时候就关闭ASR
    //      */
    //     if (g_enable_deepseek.load()) {
    //         if (!local_ptr_timer->Check()) {  // 如果计时器触发了，就发送内容给deepseek节点
    //             auto msg = std_msgs::msg::String();
    //             msg.data = message;
    //             _deepseek_msg_pub_ptr->publish(msg);
    //             /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "send a msg to Deepseek Node ... ");
    //         } else {
    //             /** @details 计时器仍未触发，则仅拼接内容，但是不发送内容给deepseek节点 */
    //         }
    //     } else {
    //         /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "not send msg to Deepseek Node, because Deepseek mode is disable.");
    //     }
    // }
}

void speech_recognition::SendSpeakerWavFile(std::string str)
{
    // 把播放任务传递给speaker节点
    auto msg_to_speaker = std_msgs::msg::String();
    msg_to_speaker.data = str;
    _wav_file_pub_ptr->publish(msg_to_speaker);
}

void speech_recognition::CallbackTrigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // _trigger_timestamp.store(TimerTools::GetNowTickMs());
    // /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "_trigger_timestamp = %d", _trigger_timestamp.load());
    // /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "TimerTools::GetNowTickMs, line: %d = %d", __LINE__, TimerTools::GetNowTickMs());

    if (_cmd_ready.load()) // 有语音指令已就绪准备执行
    {
        _trigger_received.store(true); // 修改标志位，收到了trigger
        _cv_cmd.notify_all();
        /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "_cv_cmd.notify_all()");
        std::unique_lock<std::mutex> lock(_mtx_trigger); // 阻塞在此处，等待语音指令执行完1个step
        _cv_trigger.wait(lock, []
                         { return _cmd_step_executed.load(); });
        /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "finish 1 step");
        // 什么都不做，立刻返回
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        response->success = true;
        response->message = "speech recognition executed one step successfully.";
        _trigger_received.store(false); // 修改标志位，这个trigger已被使用
    }
    else
    {
        // 什么都不做，立刻返回
        response->success = true;
        response->message = "speech recognition send response immediately. No speech command executed.";
    }
}

void speech_recognition::CallbackASRControl(const std_msgs::msg::String::SharedPtr msg)
{
    /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), msg->data.c_str());
    if (msg->data == "stop ASR")
    {
        g_asr_enabled.store(false);
    }
    else if (msg->data == "start ASR")
    {
        g_asr_enabled.store(true);
        SendSpeakerWavFile("bo");
    }
    else if (msg->data == "receive deepseek response")
    {
    }
}

void speech_recognition::CallbackRecordAudioData(const interface::msg::AudioDataPackage::SharedPtr msg)
{
    std::vector<int16_t> audio_data = msg->data;
    local_audio_data_package_cache.push_back(audio_data);
    // /*DEBUG*/ RCLCPP_DEBUG(this->get_logger(), "local_audio_data_package_cache size = %ld", local_audio_data_package_cache.size());
    // 将前16个字节打印出来
    // size_t elementsToPrint = std::min(static_cast<size_t>(8), audio_data.size());
    // std::ostringstream oss;
    // oss << std::hex;
    // for (size_t i = 0; i < elementsToPrint; ++i) {
    //     oss << std::setw(4) << std::setfill('0') << audio_data[i];
    //     if (i < elementsToPrint - 1) {
    //         oss << " ";
    //     }
    // }
    // RCLCPP_INFO(this->get_logger(), "First 16 bytes of audio data (in hex): %s", oss.str().c_str());
}

// void speech_recognition::CallbackRunState(const interface::msg::RunState::SharedPtr msg) { RCLCPP_INFO(this->get_logger(), "run state: %d", msg->value); }

void speech_recognition::CallbackXiaoZhiSTT(const std_msgs::msg::String::SharedPtr msg)
{
    std::string temp = msg->data;
    g_ptr_stt_queue->enqueue(temp);
    /*DEBUG*/ RCLCPP_INFO(this->get_logger(), "receive /xiaozhi/stt topic msg. content: %s", temp.c_str());
}

void speech_recognition::CallbackGamepad(const interface::msg::GamepadCmd::SharedPtr msg) { _gamepad_takeover.store(msg->takeover); }