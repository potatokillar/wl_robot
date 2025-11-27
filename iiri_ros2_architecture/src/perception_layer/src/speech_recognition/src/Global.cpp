/*
 * @Author: 唐文浩
 * @Date: 2024-11-27
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-21
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "Global.hpp"

#include <alsa/asoundlib.h>

#include <fstream>

extern std::string g_appkey = "";
extern std::string g_akId = "";
extern std::string g_akSecret = "";
extern std::string g_domain = "";
extern std::string g_api_version = "";
extern std::string g_url = "";
extern std::string g_vipServerDomain = "";
extern std::string g_vipServerTargetDomain = "";
extern std::string g_audio_path = "";
extern std::string global_play_device_name = "";
extern std::string global_record_device_name = "";
extern int g_threads = 1;
extern int g_cpu = 1;
extern int g_sync_timeout = 0;
extern bool g_save_audio = false;
extern bool g_on_message_flag = false;
extern bool g_continued_flag = false;
extern bool g_loop_file_flag = false;
extern long g_expireTime = -1;
std::atomic<bool> g_run = true;
std::atomic<bool> g_pause = false;
std::atomic<bool> g_user_speaking = false;
std::atomic<bool> g_enable_deepseek = false;
std::atomic<bool> g_enable_write_pcm = false;
std::atomic<int> g_llm_act_mode = 1; // 0 - 多轮发言模式，1 - 单句发言模式
std::condition_variable cv_pause;
std::mutex mtx_pause;
std::shared_ptr<speech_recognition> g_node_ptr = nullptr;
static moodycamel::ReaderWriterQueue<std::string> s_stt_queue(20);
std::shared_ptr<moodycamel::ReaderWriterQueue<std::string>> g_ptr_stt_queue = std::shared_ptr<moodycamel::ReaderWriterQueue<std::string>>(&s_stt_queue);

void signal_handler_int(int signo)
{
    std::cout << "\nget interrupt mesg\n"
              << std::endl;
    g_run.store(false);
}
void signal_handler_quit(int signo)
{
    std::cout << "\nget quit mesg\n"
              << std::endl;
    g_run.store(false);
}

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

std::string FindDeviceByName(const std::string &target_name)
{
    std::ifstream file("/proc/asound/cards");
    std::string line;
    int deviceNumber = -1;

    // 逐行读取文件内容
    while (getline(file, line))
    {
        // 找到包含目标设备名称的行
        if (line.find(target_name) != std::string::npos)
        {
            // 解析前面的序号
            std::istringstream iss(line);
            iss >> deviceNumber; // 第一个字段是序号
            break;
        }
    }

    // 如果找到了正确的设备编号，则生成并返回字符串
    if (deviceNumber != -1)
    {
        return "plughw:" + std::to_string(deviceNumber) + ",0";
    }
    else
    {
        return ""; // 如果未找到匹配项，则返回空字符串或采取其他错误处理措施
    }

    return 0;
}

void checkAlsaError(int err, const std::string &msg)
{
    if (err < 0)
    {
        throw std::runtime_error(msg + ": " + snd_strerror(err));
    }
}