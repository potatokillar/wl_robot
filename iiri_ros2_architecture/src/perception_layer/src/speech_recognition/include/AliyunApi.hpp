/*
 * @Author: 唐文浩
 * @Date: 2024-11-19
 * @LastEditors: 唐文浩
 * @LastEditTime: 2024-12-31
 * @Description: 基于阿里云在线识别stDemo进行改进，从麦克风获取数据上传server
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */

/*
 * Copyright 2021 Alibaba Group Holding Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ST_DEMO_MICROPHONE_HPP_
#define ST_DEMO_MICROPHONE_HPP_

#include <dirent.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "nlsClient.h"
#include "nlsEvent.h"
#include "nlsToken.h"
#include "speechTranscriberRequest.h"
// 自己编写的头文件
#include <chrono>
#include <iomanip>
#include <thread>

#include "AliyunCallback.hpp"
#include "Global.hpp"

typedef struct PROFILE_INFO
{
    char usr_name[32];
    unsigned int pid;
    float ave_cpu_percent;
    float ave_mem_percent;
    unsigned long long eAveTime;
} PROFILE_INFO;

// 自定义线程参数
struct ParamStruct
{
    char fileName[DEFAULT_STRING_LEN];
    char token[DEFAULT_STRING_LEN];
    char appkey[DEFAULT_STRING_LEN];
    char url[DEFAULT_STRING_LEN];
    char vipServerDomain[DEFAULT_STRING_LEN];
    char vipServerTargetDomain[DEFAULT_STRING_LEN];

    uint64_t startedConsumed;   /*started事件完成次数*/
    uint64_t firstConsumed;     /*首包完成次数*/
    uint64_t completedConsumed; /*completed事件次数*/
    uint64_t closeConsumed;     /*closed事件次数*/

    uint64_t failedConsumed;  /*failed事件次数*/
    uint64_t requestConsumed; /*发起请求次数*/

    uint64_t sendConsumed; /*sendAudio调用次数*/

    uint64_t startTotalValue; /*所有started完成时间总和*/
    uint64_t startAveValue;   /*started完成平均时间*/
    uint64_t startMaxValue;   /*调用start()到收到started事件最大用时*/
    uint64_t startMinValue;   /*调用start()到收到started事件最小用时*/

    uint64_t firstTotalValue; /*所有收到首包用时总和*/
    uint64_t firstAveValue;   /*收到首包平均时间*/
    uint64_t firstMaxValue;   /*调用start()到收到首包最大用时*/
    uint64_t firstMinValue;   /*调用start()到收到首包最小用时*/
    bool firstFlag;           /*是否收到首包的标记*/

    uint64_t endTotalValue; /*start()到completed事件的总用时*/
    uint64_t endAveValue;   /*start()到completed事件的平均用时*/
    uint64_t endMaxValue;   /*start()到completed事件的最大用时*/
    uint64_t endMinValue;   /*start()到completed事件的最小用时*/

    uint64_t closeTotalValue; /*start()到closed事件的总用时*/
    uint64_t closeAveValue;   /*start()到closed事件的平均用时*/
    uint64_t closeMaxValue;   /*start()到closed事件的最大用时*/
    uint64_t closeMinValue;   /*start()到closed事件的最小用时*/

    uint64_t sendTotalValue; /*单线程调用sendAudio总耗时*/

    uint64_t audioFileTimeLen; /*灌入音频文件的音频时长*/

    uint64_t s50Value;  /*start()到started用时50ms以内*/
    uint64_t s100Value; /*start()到started用时100ms以内*/
    uint64_t s200Value;
    uint64_t s500Value;
    uint64_t s1000Value;
    uint64_t s2000Value;

    pthread_mutex_t mtx;
};

struct SentenceParamStruct
{
    uint32_t sentenceId;
    std::string text;
    uint64_t beginTime;
    uint64_t endTime;
    struct timeval endTv;
};

// 自定义事件回调参数
class ParamCallBack
{
public:
    explicit ParamCallBack(ParamStruct *param)
    {
        userId = 0;
        memset(userInfo, 0, 8);
        tParam = param;

        pthread_mutex_init(&mtxWord, NULL);
        pthread_cond_init(&cvWord, NULL);
    };
    ~ParamCallBack()
    {
        tParam = NULL;
        pthread_mutex_destroy(&mtxWord);
        pthread_cond_destroy(&cvWord);
    };

    unsigned long userId;
    char userInfo[8];

    pthread_mutex_t mtxWord;
    pthread_cond_t cvWord;

    struct timeval startTv;
    struct timeval startedTv;
    struct timeval startAudioTv;
    struct timeval firstTv;
    struct timeval completedTv;
    struct timeval closedTv;
    struct timeval failedTv;

    ParamStruct *tParam;

    std::vector<struct SentenceParamStruct> sentenceParam;
};

// 统计参数
struct ParamStatistics
{
    ParamStatistics()
    {
        running = false;
        success_flag = false;
        failed_flag = false;
        audio_ms = 0;
        start_ms = 0;
        end_ms = 0;
        ave_ms = 0;
        s_cnt = 0;
    };

    bool running;
    bool success_flag;
    bool failed_flag;

    uint64_t audio_ms;
    uint64_t start_ms;
    uint64_t end_ms;
    uint64_t ave_ms;

    uint32_t s_cnt;
};

static int loop_timeout = LOOP_TIMEOUT; /*循环运行的时间, 单位s*/
static int loop_count = 3;              /*循环测试某音频文件的次数, 设置后loop_timeout无效*/
static pthread_mutex_t params_mtx;      /*全局统计参数g_statistics的操作锁*/
static std::map<unsigned long, struct ParamStatistics *> g_statistics;
static int sample_rate = SAMPLE_RATE_16K;
static int frame_size = FRAME_16K_20MS; /*每次推送音频字节数.*/
static int encoder_type = ENCODER_OPUS;
static int logLevel = AlibabaNls::LogDebug; /* 0:为关闭log */
static int max_sentence_silence = 0;        /*最大静音断句时间, 单位ms. 默认不设置.*/
static int run_cnt = 0;
static int run_start_failed = 0;
static int run_cancel = 0;
static int run_success = 0;
static int run_fail = 0;
static bool global_sys = true;
static PROFILE_INFO g_ave_percent;
static PROFILE_INFO g_min_percent;
static PROFILE_INFO g_max_percent;
static int profile_scan = -1;
static int cur_profile_scan = -1;
static PROFILE_INFO *g_sys_info = NULL;
static bool longConnection = false;
static bool sysAddrinfo = false;
static bool noSleepFlag = false;
static bool enableIntermediateResult = false;

static void vectorStartStore(unsigned long pid)
{
    pthread_mutex_lock(&params_mtx);

    std::map<unsigned long, struct ParamStatistics *>::iterator iter;
    iter = g_statistics.find(pid);
    if (iter != g_statistics.end())
    {
        // 已经存在
        struct timeval start_tv;
        gettimeofday(&start_tv, NULL);
        iter->second->start_ms = start_tv.tv_sec * 1000 + start_tv.tv_usec / 1000;
        RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"), "vectorStartStore start: %d", iter->second->start_ms);
    }

    pthread_mutex_unlock(&params_mtx);
    return;
}

static void vectorSetParams(unsigned long pid, bool add, struct ParamStatistics params)
{
    pthread_mutex_lock(&params_mtx);

    std::map<unsigned long, struct ParamStatistics *>::iterator iter;
    iter = g_statistics.find(pid);
    if (iter != g_statistics.end())
    {
        // 已经存在
        iter->second->running = params.running;
        iter->second->success_flag = params.success_flag;
        iter->second->failed_flag = false;
        if (params.audio_ms > 0)
        {
            iter->second->audio_ms = params.audio_ms;
        }
    }
    else
    {
        // 不存在, 新的pid
        if (add)
        {
            struct ParamStatistics *p_tmp = new (struct ParamStatistics);
            if (!p_tmp)
                return;
            memset(p_tmp, 0, sizeof(struct ParamStatistics));
            p_tmp->running = params.running;
            p_tmp->success_flag = params.success_flag;
            p_tmp->failed_flag = false;
            if (params.audio_ms > 0)
            {
                p_tmp->audio_ms = params.audio_ms;
            }
            g_statistics.insert(std::make_pair(pid, p_tmp));
        }
        else
        {
        }
    }

    pthread_mutex_unlock(&params_mtx);
    return;
}

static void vectorSetRunning(unsigned long pid, bool run)
{
    pthread_mutex_lock(&params_mtx);

    std::map<unsigned long, struct ParamStatistics *>::iterator iter;
    iter = g_statistics.find(pid);
    if (iter != g_statistics.end())
    {
        // 已经存在
        iter->second->running = run;
    }
    else
    {
    }

    pthread_mutex_unlock(&params_mtx);
    return;
}

static void vectorSetResult(unsigned long pid, bool ret)
{
    pthread_mutex_lock(&params_mtx);

    std::map<unsigned long, struct ParamStatistics *>::iterator iter;
    iter = g_statistics.find(pid);
    if (iter != g_statistics.end())
    {
        // 已经存在
        iter->second->success_flag = ret;

        if (ret)
        {
            struct timeval end_tv;
            gettimeofday(&end_tv, NULL);
            iter->second->end_ms = end_tv.tv_sec * 1000 + end_tv.tv_usec / 1000;
            uint64_t d_ms = iter->second->end_ms - iter->second->start_ms;

            if (iter->second->ave_ms == 0)
            {
                iter->second->ave_ms = d_ms;
            }
            else
            {
                iter->second->ave_ms = (d_ms + iter->second->ave_ms) / 2;
            }
            iter->second->s_cnt++;
        }
    }
    else
    {
    }

    pthread_mutex_unlock(&params_mtx);
    return;
}

static void vectorSetFailed(unsigned long pid, bool ret)
{
    pthread_mutex_lock(&params_mtx);

    std::map<unsigned long, struct ParamStatistics *>::iterator iter;
    iter = g_statistics.find(pid);
    if (iter != g_statistics.end())
    {
        // 已经存在
        iter->second->failed_flag = ret;
    }
    else
    {
    }

    pthread_mutex_unlock(&params_mtx);
    return;
}

static bool vectorGetRunning(unsigned long pid)
{
    pthread_mutex_lock(&params_mtx);

    bool result = false;
    std::map<unsigned long, struct ParamStatistics *>::iterator iter;
    iter = g_statistics.find(pid);
    if (iter != g_statistics.end())
    {
        // 存在
        result = iter->second->running;
    }
    else
    {
        // 不存在, 新的pid
    }

    pthread_mutex_unlock(&params_mtx);
    return result;
}

static bool vectorGetFailed(unsigned long pid)
{
    pthread_mutex_lock(&params_mtx);

    bool result = false;
    std::map<unsigned long, struct ParamStatistics *>::iterator iter;
    iter = g_statistics.find(pid);
    if (iter != g_statistics.end())
    {
        // 存在
        result = iter->second->failed_flag;
    }
    else
    {
        // 不存在, 新的pid
    }

    pthread_mutex_unlock(&params_mtx);
    return result;
}

/**
 * 根据AccessKey ID和AccessKey Secret重新生成一个token，
 * 并获取其有效期时间戳
 */
int generateToken(std::string akId, std::string akSecret, std::string *token, long *expireTime);

unsigned int getAudioFileTimeMs(const int dataSize, const int sampleRate, const int compressRate);

/**
 * @brief 获取sendAudio发送延时时间
 * @param dataSize 待发送数据大小
 * @param sampleRate 采样率 16k/8K
 * @param compressRate 数据压缩率，例如压缩比为10:1的16k opus编码，此时为10；
 *                     非压缩数据则为1
 * @return 返回sendAudio之后需要sleep的时间
 * @note 对于8k pcm 编码数据, 16位采样，建议每发送1600字节 sleep 100 ms.
         对于16k pcm 编码数据, 16位采样，建议每发送3200字节 sleep 100 ms.
         对于其它编码格式(OPUS)的数据, 由于传递给SDK的仍然是PCM编码数据,
         按照SDK OPUS/OPU 数据长度限制, 需要每次发送640字节 sleep 20ms.
 */
unsigned int getSendAudioSleepTime(const int dataSize, const int sampleRate, const int compressRate);

/** @brief: 语音识别核心功能 */
void AliyunApiClient(rclcpp::Logger logger);

#endif