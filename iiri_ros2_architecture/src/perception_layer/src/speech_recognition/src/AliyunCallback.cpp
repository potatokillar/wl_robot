#include "AliyunCallback.hpp"

#include "DequeTemplate.hpp"
#include "Global.hpp"

std::shared_ptr<DequeTemplate<std::string>> g_ptr_trans_result_queue = std::make_shared<DequeTemplate<std::string>>();  // 存放识别结果字符串的容器指针
// 存放传递给deepseek的字符串的队列和互斥锁
/** @todo 2025.02.07 - g_deepseek_msg和g_ptr_trans_result_queue是可以合并的，现在先不做 */
extern std::deque<std::string> g_deepseek_msg;
extern std::mutex g_deepseek_msg_mutex;
extern std::condition_variable g_queue_cv;

void onTranscriptionStarted(AlibabaNls::NlsEvent *cbEvent, void *cbParam)
{
    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"),
                 "onTranscriptionStarted: status code: %d, task id: %d, onTranscriptionStarted: All response: %s",
                 cbEvent->getStatusCode(),
                 cbEvent->getTaskId(),
                 cbEvent->getAllResponse());

    if (cbParam) {
        ParamCallBack *tmpParam = static_cast<ParamCallBack *>(cbParam);
        if (!tmpParam->tParam) return;
        RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"),
                     "onTranscriptionStarted Max Time: %d, userId: %d",
                     tmpParam->tParam->startMaxValue,
                     tmpParam->userId);

        gettimeofday(&(tmpParam->startedTv), NULL);

        tmpParam->tParam->startedConsumed++;

        unsigned long long timeValue1 = tmpParam->startedTv.tv_sec - tmpParam->startTv.tv_sec;
        unsigned long long timeValue2 = tmpParam->startedTv.tv_usec - tmpParam->startTv.tv_usec;
        unsigned long long timeValue = 0;
        if (timeValue1 > 0) {
            timeValue = (((timeValue1 * 1000000) + timeValue2) / 1000);
        } else {
            timeValue = (timeValue2 / 1000);
        }

        // max
        if (timeValue > tmpParam->tParam->startMaxValue) {
            tmpParam->tParam->startMaxValue = timeValue;
        }

        unsigned long long tmp = timeValue;
        if (tmp <= 50) {
            tmpParam->tParam->s50Value++;
        } else if (tmp <= 100) {
            tmpParam->tParam->s100Value++;
        } else if (tmp <= 200) {
            tmpParam->tParam->s200Value++;
        } else if (tmp <= 500) {
            tmpParam->tParam->s500Value++;
        } else if (tmp <= 1000) {
            tmpParam->tParam->s1000Value++;
        } else {
            tmpParam->tParam->s2000Value++;
        }

        // min
        if (tmpParam->tParam->startMinValue == 0) {
            tmpParam->tParam->startMinValue = timeValue;
        } else {
            if (timeValue < tmpParam->tParam->startMinValue) {
                tmpParam->tParam->startMinValue = timeValue;
            }
        }

        // ave
        tmpParam->tParam->startTotalValue += timeValue;
        if (tmpParam->tParam->startedConsumed > 0) {
            tmpParam->tParam->startAveValue = tmpParam->tParam->startTotalValue / tmpParam->tParam->startedConsumed;
        }

        // first package flag init
        tmpParam->tParam->firstFlag = false;

        // pid, add, run, success
        struct ParamStatistics params;
        params.running = true;
        params.success_flag = false;
        params.audio_ms = 0;
        vectorSetParams(tmpParam->userId, true, params);

        // 通知发送线程start()成功, 可以继续发送数据
        pthread_mutex_lock(&(tmpParam->mtxWord));
        pthread_cond_signal(&(tmpParam->cvWord));
        pthread_mutex_unlock(&(tmpParam->mtxWord));
    }
}

void onSentenceBegin(AlibabaNls::NlsEvent *cbEvent, void *cbParam)
{
    g_user_speaking.store(true);
    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"), "g_user_speaking = %s", g_user_speaking.load() ? "true" : "false");
    ParamCallBack *tmpParam = static_cast<ParamCallBack *>(cbParam);
    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"),
                 "onSentenceBegin: All response: %s",
                 cbEvent->getAllResponse());  // 获取服务端返回的全部信息
}

void onSentenceEnd(AlibabaNls::NlsEvent *cbEvent, void *cbParam)
{
    g_user_speaking.store(false);
    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"), "g_user_speaking = %s", g_user_speaking.load() ? "true" : "false");
    ParamCallBack *tmpParam = static_cast<ParamCallBack *>(cbParam);
    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"),
                 "onSentenceEnd: All response: %s",
                 cbEvent->getAllResponse());  // 获取服务端返回的全部信息

    std::string temp_result(cbEvent->getResult());
    g_ptr_trans_result_queue->Push(temp_result);
    // 存放到队列中，后面发送给ds节点
    if (g_enable_deepseek.load()) {  // 只有开启深度思考，这个字符串，才会被当做用户发言，准备拼接、发送给deepseek
        std::lock_guard<std::mutex> lock(g_deepseek_msg_mutex);
        g_deepseek_msg.push_back(temp_result);
        g_queue_cv.notify_all();
    }
    RCLCPP_INFO(g_node_ptr->get_logger().get_child("AliyunApiClient"), "Transcription result: %s", cbEvent->getResult());

    struct SentenceParamStruct param;
    param.sentenceId = cbEvent->getSentenceIndex();
    param.text.assign(cbEvent->getResult());
    param.beginTime = cbEvent->getSentenceBeginTime();
    param.endTime = cbEvent->getSentenceTime();
    gettimeofday(&(param.endTv), NULL);
    tmpParam->sentenceParam.push_back(param);
}

void onTranscriptionResultChanged(AlibabaNls::NlsEvent *cbEvent, void *cbParam)
{
    if (cbParam) {
        ParamCallBack *tmpParam = static_cast<ParamCallBack *>(cbParam);

        if (tmpParam->tParam->firstFlag == false) {
            tmpParam->tParam->firstConsumed++;
            tmpParam->tParam->firstFlag = true;

            gettimeofday(&(tmpParam->firstTv), NULL);

            unsigned long long timeValue1 = tmpParam->firstTv.tv_sec - tmpParam->startTv.tv_sec;
            unsigned long long timeValue2 = tmpParam->firstTv.tv_usec - tmpParam->startTv.tv_usec;
            unsigned long long timeValue = 0;
            if (timeValue1 > 0) {
                timeValue = (((timeValue1 * 1000000) + timeValue2) / 1000);
            } else {
                timeValue = (timeValue2 / 1000);
            }

            // max
            if (timeValue > tmpParam->tParam->firstMaxValue) {
                tmpParam->tParam->firstMaxValue = timeValue;
            }
            // min
            if (tmpParam->tParam->firstMinValue == 0) {
                tmpParam->tParam->firstMinValue = timeValue;
            } else {
                if (timeValue < tmpParam->tParam->firstMinValue) {
                    tmpParam->tParam->firstMinValue = timeValue;
                }
            }
            // ave
            tmpParam->tParam->firstTotalValue += timeValue;
            if (tmpParam->tParam->firstConsumed > 0) {
                tmpParam->tParam->firstAveValue = tmpParam->tParam->firstTotalValue / tmpParam->tParam->firstConsumed;
            }
        }  // firstFlag
    }

    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"),
                 "onTranscriptionResultChanged: All response: %s",
                 cbEvent->getAllResponse());  // 获取服务端返回的全部信息
}

void onTranscriptionCompleted(AlibabaNls::NlsEvent *cbEvent, void *cbParam)
{
    run_success++;

    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"),
                 "onTranscriptionCompleted: All response: %s",
                 cbEvent->getAllResponse());  // 获取服务端返回的全部信息

    if (cbParam) {
        ParamCallBack *tmpParam = static_cast<ParamCallBack *>(cbParam);
        if (!tmpParam->tParam) return;

        RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"),
                     "onTranscriptionCompleted Max Time: %d, userId: %d",
                     tmpParam->tParam->endMaxValue,
                     tmpParam->userId);

        gettimeofday(&(tmpParam->completedTv), NULL);
        tmpParam->tParam->completedConsumed++;

        unsigned long long timeValue1 = tmpParam->completedTv.tv_sec - tmpParam->startTv.tv_sec;
        unsigned long long timeValue2 = tmpParam->completedTv.tv_usec - tmpParam->startTv.tv_usec;
        unsigned long long timeValue = 0;
        if (timeValue1 > 0) {
            timeValue = (((timeValue1 * 1000000) + timeValue2) / 1000);
        } else {
            timeValue = (timeValue2 / 1000);
        }

        // max
        if (timeValue > tmpParam->tParam->endMaxValue) {
            tmpParam->tParam->endMaxValue = timeValue;
        }
        // min
        if (tmpParam->tParam->endMinValue == 0) {
            tmpParam->tParam->endMinValue = timeValue;
        } else {
            if (timeValue < tmpParam->tParam->endMinValue) {
                tmpParam->tParam->endMinValue = timeValue;
            }
        }
        // ave
        tmpParam->tParam->endTotalValue += timeValue;
        if (tmpParam->tParam->completedConsumed > 0) {
            tmpParam->tParam->endAveValue = tmpParam->tParam->endTotalValue / tmpParam->tParam->completedConsumed;
        }

        vectorSetResult(tmpParam->userId, true);
    }
}

void onTaskFailed(AlibabaNls::NlsEvent *cbEvent, void *cbParam)
{
    run_fail++;

    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"),
                 "onTaskFailed: All response: %s",
                 cbEvent->getAllResponse());  // 获取服务端返回的全部信息

    if (cbParam) {
        ParamCallBack *tmpParam = static_cast<ParamCallBack *>(cbParam);
        if (!tmpParam->tParam) return;

        tmpParam->tParam->failedConsumed++;

        vectorSetResult(tmpParam->userId, false);
        vectorSetFailed(tmpParam->userId, true);
    }
}

void onMessage(AlibabaNls::NlsEvent *cbEvent, void *cbParam)
{
    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"), "onMessage: All response: %s", cbEvent->getAllResponse());  // 获取服务端返回的全部信息
    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"), "onMessage: msg tyep: %s", cbEvent->getMsgType());

// 这里需要解析json
// x86和arm的sdk对parseJsonMsg函数的声明不同，x86的有入参，arm的无入参
#ifdef __x86_64__
    int result = cbEvent->parseJsonMsg(true);
#else
    int result = cbEvent->parseJsonMsg();
#endif
    if (result) {
        RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"), "onMessage: parseJsonMsg failed: %d", result);
    } else {
        ParamCallBack *tmpParam = static_cast<ParamCallBack *>(cbParam);
        switch (cbEvent->getMsgType()) {
            case AlibabaNls::NlsEvent::TaskFailed:
                break;
            case AlibabaNls::NlsEvent::TranscriptionStarted:
                // 通知发送线程start()成功, 可以继续发送数据
                pthread_mutex_lock(&(tmpParam->mtxWord));
                pthread_cond_signal(&(tmpParam->cvWord));
                pthread_mutex_unlock(&(tmpParam->mtxWord));
                break;
            case AlibabaNls::NlsEvent::Close:
                // 通知发送线程, 最终识别结果已经返回, 可以调用stop()
                pthread_mutex_lock(&(tmpParam->mtxWord));
                pthread_cond_signal(&(tmpParam->cvWord));
                pthread_mutex_unlock(&(tmpParam->mtxWord));
                break;
        }
    }
}

void onChannelClosed(AlibabaNls::NlsEvent *cbEvent, void *cbParam)
{
    RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"),
                 "OnChannelClosed: All response: %s",
                 cbEvent->getAllResponse());  // 获取服务端返回的全部信息

    if (cbParam) {
        ParamCallBack *tmpParam = static_cast<ParamCallBack *>(cbParam);
        if (!tmpParam->tParam) {
            RCLCPP_DEBUG(g_node_ptr->get_logger().get_child("AliyunApiClient"), "OnChannelCloseed tParam is nullptr");
            return;
        }

        tmpParam->tParam->closeConsumed++;
        gettimeofday(&(tmpParam->closedTv), NULL);

        unsigned long long timeValue1 = tmpParam->closedTv.tv_sec - tmpParam->startTv.tv_sec;
        unsigned long long timeValue2 = tmpParam->closedTv.tv_usec - tmpParam->startTv.tv_usec;
        unsigned long long timeValue = 0;
        if (timeValue1 > 0) {
            timeValue = (((timeValue1 * 1000000) + timeValue2) / 1000);
        } else {
            timeValue = (timeValue2 / 1000);
        }

        // max
        if (timeValue > tmpParam->tParam->closeMaxValue) {
            tmpParam->tParam->closeMaxValue = timeValue;
        }
        // min
        if (tmpParam->tParam->closeMinValue == 0) {
            tmpParam->tParam->closeMinValue = timeValue;
        } else {
            if (timeValue < tmpParam->tParam->closeMinValue) {
                tmpParam->tParam->closeMinValue = timeValue;
            }
        }
        // ave
        tmpParam->tParam->closeTotalValue += timeValue;
        if (tmpParam->tParam->closeConsumed > 0) {
            tmpParam->tParam->closeAveValue = tmpParam->tParam->closeTotalValue / tmpParam->tParam->closeConsumed;
        }

        // int vec_len = tmpParam->sentenceParam.size();
        // if (vec_len > 0) {
        //     std::cout << "  \n=================================" << std::endl;
        //     std::cout << "  |  max sentence silence: " << max_sentence_silence << "ms" << std::endl;
        //     std::cout << "  |  frame size: " << frame_size << "bytes" << std::endl;
        //     std::cout << "  --------------------------------" << std::endl;
        //     unsigned long long timeValue0 = tmpParam->startTv.tv_sec * 1000 + tmpParam->startTv.tv_usec / 1000;
        //     std::cout << "  |  start tv: " << timeValue0 << "ms" << std::endl;

        //     unsigned long long timeValue1 = tmpParam->startedTv.tv_sec * 1000 + tmpParam->startedTv.tv_usec / 1000;
        //     std::cout << "  |  started tv: " << timeValue1 << "ms" << std::endl;
        //     std::cout << "  |    started duration: " << timeValue1 - timeValue0 << "ms" << std::endl;

        //     unsigned long long timeValue2 = tmpParam->startAudioTv.tv_sec * 1000 + tmpParam->startAudioTv.tv_usec / 1000;
        //     std::cout << "  |  start audio tv: " << timeValue2 << "ms" << std::endl;
        //     std::cout << "  |    start audio duration: " << timeValue2 - timeValue0 << "ms" << std::endl;
        //     std::cout << "  --------------------------------" << std::endl;
        //     for (int i = 0; i < vec_len; i++) {
        //         struct SentenceParamStruct tmp = tmpParam->sentenceParam[i];
        //         std::cout << "  |  index: " << tmp.sentenceId << std::endl;
        //         std::cout << "  |  sentence duration: " << tmp.beginTime << " - " << tmp.endTime << "ms = " << (tmp.endTime - tmp.beginTime) << "ms"
        //                   << std::endl;
        //         unsigned long long endTimeValue = tmp.endTv.tv_sec * 1000 + tmp.endTv.tv_usec / 1000;
        //         std::cout << "  |  end tv duration: " << timeValue2 << " - " << endTimeValue << "ms = " << (endTimeValue - timeValue2) << "ms" << std::endl;
        //         std::cout << "  |  text: " << tmp.text << std::endl;
        //         std::cout << "  --------------------------------" << std::endl;
        //     }
        //     std::cout << "  =================================\n" << std::endl;

        //     for (int j = 0; j < vec_len; j++) {
        //         tmpParam->sentenceParam.pop_back();
        //     }
        //     tmpParam->sentenceParam.clear();
        // }

        // 通知发送线程, 最终识别结果已经返回, 可以调用stop()
        pthread_mutex_lock(&(tmpParam->mtxWord));
        pthread_cond_signal(&(tmpParam->cvWord));
        pthread_mutex_unlock(&(tmpParam->mtxWord));
    }
}