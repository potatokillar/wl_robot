/*
 * @Author: 唐文浩
 * @Date: 2024-11-20
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-18
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
/**
 * 全局维护一个服务鉴权token和其对应的有效期时间戳，
 * 每次调用服务之前，首先判断token是否已经过期，
 * 如果已经过期，则根据AccessKey ID和AccessKey Secret重新生成一个token，
 * 并更新这个全局的token和其有效期时间戳。
 *
 * 注意：不要每次调用服务之前都重新生成新token，
 * 只需在token即将过期时重新生成即可。所有的服务并发可共用一个token。
 */
#include "AliyunApi.hpp"

#include "DequeTemplate.hpp"
#include "Global.hpp"
#include "nlsToken_c_api.h"

extern std::shared_ptr<DequeTemplate<uint8_t>> g_ptr_byte_queue; // 与麦克风线程共用的容器指针

extern std::atomic<bool> global_audio_active;   // 控制音频处理激活状态
extern std::condition_variable cv_audio_active; // 条件变量用于线程同步
extern std::mutex mtx_audio_active;             // 互斥锁保护条件变量

int generateToken(std::string akId, std::string akSecret, std::string *token, long *expireTime)
{
    AlibabaNlsCommon::NlsToken nlsTokenRequest;

    // nlsTokenRequest.setAccessKeyId(akId);
    nls_token_setAccessKeyId(&nlsTokenRequest, akId.c_str()); // 替换为C接口

    // nlsTokenRequest.setKeySecret(akSecret);
    nls_token_setKeySecret(&nlsTokenRequest, akSecret.c_str()); // 替换为C接口
    if (!g_domain.empty())
    {
        // nlsTokenRequest.setDomain(g_domain);
        nls_token_setDomain(&nlsTokenRequest, g_domain.c_str()); // 替换为C接口
    }
    // if (!g_api_version.empty()) {
    //     nlsTokenRequest.setServerVersion(g_api_version);
    // }

    int retCode = nlsTokenRequest.applyNlsToken();
    /*获取失败原因*/
    if (retCode < 0)
    {
        RCLCPP_WARN(g_node_ptr->get_logger().get_child("AliyunApiClient"), "Failed error code: %d, error msg: %s", retCode, nlsTokenRequest.getErrorMsg());
        return retCode;
    }

    *token = nlsTokenRequest.getToken();
    *expireTime = nlsTokenRequest.getExpireTime();

    return 0;
}

unsigned int getAudioFileTimeMs(const int dataSize, const int sampleRate, const int compressRate)
{
    // 仅支持16位采样
    const int sampleBytes = 16;
    // 仅支持单通道
    const int soundChannel = 1;

    // 当前采样率，采样位数下每秒采样数据的大小
    int bytes = (sampleRate * sampleBytes * soundChannel) / 8;

    // 当前采样率，采样位数下每毫秒采样数据的大小
    int bytesMs = bytes / 1000;

    // 待发送数据大小除以每毫秒采样数据大小，以获取sleep时间
    int fileMs = (dataSize * compressRate) / bytesMs;

    return fileMs;
}

unsigned int getSendAudioSleepTime(const int dataSize, const int sampleRate, const int compressRate)
{
    int sleepMs = getAudioFileTimeMs(dataSize, sampleRate, compressRate);
    return sleepMs;
}

void AliyunApiClient(rclcpp::Logger logger)
{
    RCLCPP_INFO(logger, "AliyunApiClient() start!");
    signal(SIGINT, signal_handler_int);
    signal(SIGQUIT, signal_handler_quit);
    if (sysAddrinfo)
    {
        AlibabaNls::NlsClient::getInstance()->setUseSysGetAddrInfo(true);
    }
    AlibabaNls::NlsClient::getInstance()->startWorkThread(1); // 只使用单核

    // 获取当前系统时间戳，判断token是否过期
    std::string l_token = "";
    long g_expireTime = -1;
    std::time_t curTime = std::time(0);
    // 循环直到成功生成token
    bool tokenGeneratedSuccessfully = false;
    int wait_second = 5; // 鉴权失败后重复鉴权的时间间隔

    while (!tokenGeneratedSuccessfully)
    {
        if (l_token.empty())
        {
            curTime = std::time(0); // 更新当前时间
            if (g_expireTime - curTime < 10)
            {
                RCLCPP_INFO(logger, "the token will be expired, please generate new token by AccessKey - ID and AccessKey - Secret.");
                int ret = generateToken(g_akId, g_akSecret, &l_token, &g_expireTime);
                if (ret < 0)
                {
                    RCLCPP_INFO(logger, "generate token failed");
                    std::this_thread::sleep_for(std::chrono::seconds(wait_second));
                    continue;
                }
                else
                {
                    if (l_token.empty() || g_expireTime < 0)
                    {
                        RCLCPP_INFO(logger, "generate empty token");
                        std::this_thread::sleep_for(std::chrono::seconds(wait_second));
                        continue;
                    }
                    RCLCPP_INFO(logger, "the token generated successfully.");
                    tokenGeneratedSuccessfully = true; // 结束循环
                }
            }
        }
        else
        {
            // 如果token不为空且未接近过期，这里可以结束循环或者执行其他逻辑
            tokenGeneratedSuccessfully = true;
        }
    }

    // 在旧的speechTranscriberMultFile函数中，下一步是构造ParamStruct对象，并创建线程，把ParamStruct传递给线程
    ParamStruct param_struct;
    memset(param_struct.token, 0, DEFAULT_STRING_LEN);
    memcpy(param_struct.token, l_token.c_str(), l_token.length());
    memset(param_struct.appkey, 0, DEFAULT_STRING_LEN);
    memcpy(param_struct.appkey, g_appkey.c_str(), g_appkey.length());

    // 由于现在不采用多线程识别，所以不创建线程了，直接进入pthreadFunction函数
    int sleepMs = 0;   // 根据发送音频数据帧长度计算sleep时间，用于模拟真实录音情景
    int testCount = 0; // 运行次数计数，用于超过设置的loop次数后退出
    bool timedwait_flag = false;

    // 从自定义线程参数中获取token, 配置文件等参数.
    ParamCallBack *cbParam = NULL;
    cbParam = new ParamCallBack(&param_struct);
    cbParam->userId = pthread_self();
    cbParam->userId = pthread_self();
    memset(cbParam->userInfo, 0, 8);
    strcpy(cbParam->userInfo, "User.");

    // 旧的pthreadFunction函数的工作循环
    do
    {
        cbParam->tParam->requestConsumed++;
        // 创建实时音频流识别SpeechTranscriberRequest对象
        AlibabaNls::SpeechTranscriberRequest *request = AlibabaNls::NlsClient::getInstance()->createTranscriberRequest("cpp", longConnection);
        if (request == NULL)
        {
            RCLCPP_INFO(logger, "createTranscriberRequest failed.");
            delete cbParam;
            cbParam = NULL;
            return;
        }

        // 设置用于接收结果的回调
        request->setOnTranscriptionStarted(onTranscriptionStarted, cbParam);             // 设置识别启动回调函数
        request->setOnTranscriptionResultChanged(onTranscriptionResultChanged, cbParam); // 设置识别结果变化回调函数
        request->setOnTranscriptionCompleted(onTranscriptionCompleted, cbParam);         // 设置语音转写结束回调函数
        request->setOnSentenceBegin(onSentenceBegin, cbParam);                           // 设置一句话开始回调函数
        request->setOnSentenceEnd(onSentenceEnd, cbParam);                               // 设置一句话结束回调函数
        request->setOnTaskFailed(onTaskFailed, cbParam);                                 // 设置异常识别回调函数
        request->setOnChannelClosed(onChannelClosed, cbParam);                           // 设置识别通道关闭回调函数
#ifdef __x86_64__
        if (g_on_message_flag)
        {
            request->setOnMessage(onMessage, cbParam); // 设置所有服务端返回信息回调函数
            request->setEnableOnMessage(true);         // 开启所有服务端返回信息回调函数, 其他回调(除了OnBinaryDataRecved)失效
        }
        request->setEnableContinued(g_continued_flag); // 是否开启重连机制
#endif

        /* 3. 设置request的相关参数 */
        // 设置AppKey和token
        if (strlen(param_struct.appkey) > 0)
        {
            request->setAppKey(param_struct.appkey);
        }
        if (strlen(param_struct.token) > 0)
        {
            request->setToken(param_struct.token);
        }
        request->setTimeout(500); // 设置链接超时500ms
#ifdef __x86_64__
        // 获取返回文本的编码格式
        const char *output_format = request->getOutputFormat();
        RCLCPP_DEBUG(logger, "text format: %s", output_format);
#endif
        request->setFormat("pcm");                   // 设置音频数据编码格式, 可选参数, 目前支持pcm,opus,opu. 默认是pcm
        request->setSampleRate(16000);               // 设置音频数据采样率, 可选参数，目前支持16000, 8000. 默认是16000
        request->setIntermediateResult(false);       // 设置是否返回中间识别结果, 可选参数. 默认false
        request->setPunctuationPrediction(false);    // 设置是否在后处理中添加标点, 可选参数. 默认false
        request->setInverseTextNormalization(false); // 设置是否在后处理中执行数字转写, 可选参数. 默认false
        // 设置链接超时时间
        request->setTimeout(5000);
        // 设置发送超时时间
        request->setSendTimeout(5000);
        // 设置是否开启接收超时
#ifdef __x86_64__
        request->setEnableRecvTimeout(false);
#endif

        /*
         * 4.
         * start()为同步/异步两种操作，默认异步。由于异步模式通过回调判断request是否成功运行有修改门槛，且部分旧版本为同步接口。
         *    为了能较为平滑的更新升级SDK，提供了同步/异步两种调用方式。
         *    异步情况：默认未调用setSyncCallTimeout()的时候，start()调用立即返回，
         *            且返回值并不代表request成功开始工作，需要等待返回started事件表示成功启动，或返回TaskFailed事件表示失败。
         *    同步情况：调用setSyncCallTimeout()设置同步接口的超时时间，并启动同步模式。start()调用后不会立即返回，
         *            直到内部得到成功(同时也会触发started事件回调)或失败(同时也会触发TaskFailed事件回调)后返回。
         *            此方法方便旧版本SDK
         */
        RCLCPP_DEBUG(logger, "start ->");
        struct timespec outtime;
        struct timeval now;
        gettimeofday(&(cbParam->startTv), NULL);
        int ret = request->start();
        run_cnt++;
        testCount++;
        if (ret < 0) // 启动实时识别服务失败的分支
        {
            RCLCPP_WARN(logger, "start failed(%d).", ret);
            run_start_failed++;
#ifdef __x86_64__
            // start()失败，释放request对象
            const char *request_info = request->dumpAllInfo();
            if (request_info)
            {
                RCLCPP_WARN(logger, "all info: %s", request_info);
            }
#endif
            AlibabaNls::NlsClient::getInstance()->releaseTranscriberRequest(request);
            break;
        }
        else
        { // 启动成功的分支
            if (g_sync_timeout == 0)
            {
                /*
                 * 4.1.
                 * g_sync_timeout等于0，即默认未调用setSyncCallTimeout()，异步方式调用start()
                 *      需要等待返回started事件表示成功启动，或返回TaskFailed事件表示失败。
                 *
                 * 等待started事件返回表示start()成功, 然后再发送音频数据。
                 * 语音服务器存在来不及处理当前请求的情况, 10s内不返回任何回调的问题,
                 * 然后在10s后返回一个TaskFailed回调, 所以需要设置一个超时机制。
                 */
                RCLCPP_DEBUG(logger, "wait started callback.");
                gettimeofday(&now, NULL);
                outtime.tv_sec = now.tv_sec + OPERATION_TIMEOUT_S;
                outtime.tv_nsec = now.tv_usec * 1000;
                pthread_mutex_lock(&(cbParam->mtxWord));
                // 这里面的代码，在terminal里没搜索到，说明根本没运行
                if (ETIMEDOUT == pthread_cond_timedwait(&(cbParam->cvWord), &(cbParam->mtxWord), &outtime))
                {
                    RCLCPP_DEBUG(logger, "start timeout.");
#ifdef __x86_64__
                    RCLCPP_DEBUG(logger, "current request task_id: %d", request->getTaskId());
#endif
                    timedwait_flag = true;
                    pthread_mutex_unlock(&(cbParam->mtxWord));
                    request->cancel();
                    run_cancel++;
#ifdef __x86_64__
                    const char *request_info = request->dumpAllInfo();
                    if (request_info)
                    {
                        RCLCPP_DEBUG(logger, "all info: %s", request_info);
                    }
#endif
                    AlibabaNls::NlsClient::getInstance()->releaseTranscriberRequest(request);
                    continue;
                }
                pthread_mutex_unlock(&(cbParam->mtxWord));
#ifdef __x86_64__
                RCLCPP_DEBUG(logger, "current request task_id: %d", request->getTaskId());
#endif
            }
            else
            {
            }
        }

        uint64_t sendAudio_us = 0;
        uint32_t sendAudio_cnt = 0;

        /*
         * 5. 从文件取音频数据循环发送音频
         */
        gettimeofday(&(cbParam->startAudioTv), NULL);

        global_audio_active.store(true);
        cv_audio_active.notify_all();
        RCLCPP_INFO(logger, "Ready to send data to Aliyun Api Server.");
        // 打开1个pcm文件
        std::ofstream file;
        if (g_enable_write_pcm.load())
        {
            RCLCPP_INFO(logger, "write audio data to a PCM file");
            file.open("aliyun_audio.pcm", std::ios::binary | std::ios::trunc);
        }

        // 发送麦克风拾音线程存储到缓冲区里的字节
        while (g_run.load())
        {
            uint8_t data[frame_size];
            memset(data, 0, frame_size);

            for (int i = 0; i < frame_size; i++)
            {
                data[i] = (uint8_t)g_ptr_byte_queue->Pop();
            }
            if (g_enable_write_pcm.load())
            {
                // 将音频写入pcm文件
                if (file.is_open())
                {
                    file.write(reinterpret_cast<const char *>(data), frame_size);
                }
            }

            struct timeval tv0, tv1;
            gettimeofday(&tv0, NULL);
            ret = request->sendAudio(data, frame_size, (ENCODER_TYPE)encoder_type);
            if (ret < 0)
            {
                // 发送失败, 退出循环数据发送
                RCLCPP_WARN(logger, "send data fail(%d).", ret);
                break;
            }
            else
            {
            }

            /*
             * 运行过程中如果需要改参数, 可以调用control接口.
             * 以如下max_sentence_silence为例, 传入json字符串
             * 目前仅支持设置 max_sentence_silence和vocabulary_id
             */
            gettimeofday(&tv1, NULL);
            uint64_t tmp_us = (tv1.tv_sec - tv0.tv_sec) * 1000000 + tv1.tv_usec - tv0.tv_usec;
            sendAudio_us += tmp_us;
            sendAudio_cnt++;

            if (noSleepFlag)
            {
            }
            else
            {
                // 根据发送数据大小，采样率，数据压缩比 来获取sleep时间
                sleepMs = getSendAudioSleepTime(ret, sample_rate, 1);
                // /*DEBUG*/ RCLCPP_DEBUG(logger, "sleepMs = %d", sleepMs);
                if (sleepMs * 1000 > tmp_us) // 语音数据发送延时控制, 实际使用中无需sleep
                {
                    usleep(sleepMs * 1000 - tmp_us);
                }
            }
            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        global_audio_active.store(false);

        param_struct.sendConsumed += sendAudio_cnt;
        param_struct.sendTotalValue += sendAudio_us;
        if (sendAudio_cnt > 0)
        {
            RCLCPP_DEBUG(logger, "sendAudio ave: %d us", (sendAudio_us / sendAudio_cnt));
        }

        /*
         * 6. 通知云端数据发送结束.
         *    stop()为同步/异步两种操作，默认异步。由于异步模式通过回调判断request是否成功运行有修改门槛，且部分旧版本为同步接口。
         *    为了能较为平滑的更新升级SDK，提供了同步/异步两种调用方式。
         *    异步情况：默认未调用setSyncCallTimeout()的时候，stop()调用立即返回，
         *            且返回值并不代表request成功结束，需要等待返回closed事件表示结束。
         *    同步情况：调用setSyncCallTimeout()设置同步接口的超时时间，并启动同步模式。stop()调用后不会立即返回，
         *            直到内部完成工作，并触发closed事件回调后返回。
         *            此方法方便旧版本SDK。
         */
        RCLCPP_DEBUG(logger, "stop ->");
        // stop()后会收到所有回调，若想立即停止则调用cancel()取消所有回调
        ret = request->stop();
        RCLCPP_DEBUG(logger, "stop done. ret(%d)", ret);
        if (ret < 0)
        {
            RCLCPP_WARN(logger, "stop failed(%d).", ret);
        }
        else
        {
            if (g_sync_timeout == 0)
            {
                /*
                 * 6.1.
                 * g_sync_timeout等于0，即默认未调用setSyncCallTimeout()，异步方式调用start()
                 *      需要等待返回started事件表示成功启动，或返回TaskFailed事件表示失败。
                 *
                 * 等待started事件返回表示start()成功, 然后再发送音频数据。
                 * 语音服务器存在来不及处理当前请求的情况, 10s内不返回任何回调的问题,
                 * 然后在10s后返回一个 TaskFailed 回调, 所以需要设置一个超时机制。
                 */
                // 等待closed事件后再进行释放, 否则会出现崩溃
                // 若调用了 setSyncCallTimeout() 启动了同步调用模式,
                // 则可以不等待closed事件。
                RCLCPP_DEBUG(logger, "wait closed callback.");
                /*
                 * 语音服务器存在来不及处理当前请求, 10s内不返回任何回调的问题,
                 * 然后在10s后返回一个TaskFailed回调, 错误信息为:
                 * "Gateway:IDLE_TIMEOUT:Websocket session is idle for too long time,
                 * the last directive is 'StopRecognition'!" 所以需要设置一个超时机制.
                 */
                gettimeofday(&now, NULL);
                outtime.tv_sec = now.tv_sec + OPERATION_TIMEOUT_S;
                outtime.tv_nsec = now.tv_usec * 1000;
                pthread_mutex_lock(&(cbParam->mtxWord));
                if (ETIMEDOUT == pthread_cond_timedwait(&(cbParam->cvWord), &(cbParam->mtxWord), &outtime))
                {
                    RCLCPP_DEBUG(logger, "stop timeout");
                    timedwait_flag = true;
                    pthread_mutex_unlock(&(cbParam->mtxWord));
#ifdef __x86_64__
                    const char *request_info = request->dumpAllInfo();
                    if (request_info)
                    {
                        RCLCPP_DEBUG(logger, "all info: %s", request_info);
                    }
#endif
                    AlibabaNls::NlsClient::getInstance()->releaseTranscriberRequest(request);
                    continue;
                }
                pthread_mutex_unlock(&(cbParam->mtxWord));
            }
            else
            {
                /*
                 * 6.2.
                 * g_sync_timeout大于0，即调用了setSyncCallTimeout()，同步方式调用stop()
                 *      返回值0即表示启动成功。
                 */
            }
        }

        /*
         * 7. 完成所有工作后释放当前请求。
         *    请在closed事件(确定完成所有工作)后再释放, 否则容易破坏内部状态机,
         * 会强制卸载正在运行的请求。
         */
#ifdef __x86_64__
        const char *request_info = request->dumpAllInfo();
        if (request_info)
        {
            RCLCPP_DEBUG(logger, "all info: %s", request_info);
        }
#endif
        AlibabaNls::NlsClient::getInstance()->releaseTranscriberRequest(request);

    } while (g_run.load());
    RCLCPP_INFO(logger, "AliyunApiClient() finished!");
}