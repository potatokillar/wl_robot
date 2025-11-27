/*
 * @Author: 唐文浩
 * @Date: 2024-11-27
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-21
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#ifndef GLOBAL_HPP_
#define GLOBAL_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include "atomicops.h"
#include "readerwriterqueue.h"
#include "speech_recognition.hpp"

#define SELF_TESTING_TRIGGER
#define FRAME_16K_20MS 640
#define SAMPLE_RATE_16K 16000

#define OPERATION_TIMEOUT_S 2
#define LOOP_TIMEOUT 60
#define DEFAULT_STRING_LEN 512

#define AUDIO_FILE_NUMS 4
#define AUDIO_FILE_NAME_LENGTH 32

extern std::string g_appkey;
extern std::string g_akId;
extern std::string g_akSecret;
extern std::string g_domain;
extern std::string g_api_version;
extern std::string g_url;
extern std::string g_vipServerDomain;
extern std::string g_vipServerTargetDomain;
extern std::string g_audio_path;
extern std::string global_play_device_name;
extern std::string global_record_device_name;
extern int g_threads;
extern int g_cpu;
extern int g_sync_timeout;
extern bool g_save_audio;
extern bool g_on_message_flag;
extern bool g_continued_flag;
extern bool g_loop_file_flag;
extern long g_expireTime;
extern std::atomic<bool> g_run;
extern std::atomic<bool> g_pause;
extern std::atomic<bool> g_user_speaking; // 用户是否正在说话
extern std::atomic<bool> g_enable_deepseek;
extern std::atomic<bool> g_enable_write_pcm;
extern std::atomic<int> g_llm_act_mode;
extern std::condition_variable cv_pause;
extern std::mutex mtx_pause;
extern std::shared_ptr<speech_recognition> g_node_ptr;
extern std::shared_ptr<moodycamel::ReaderWriterQueue<std::string>> g_ptr_stt_queue;

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
void signal_handler_int(int signo);
void signal_handler_quit(int signo);
std::string WideString2String(std::wstring str_wide);
std::wstring String2WideString(std::string str_narrow);
std::string FindDeviceByName(const std::string &target_name);
void checkAlsaError(int err, const std::string &msg);

#endif