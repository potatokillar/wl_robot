/*
 * @Author: 唐文浩
 * @Date: 2025-04-21
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-30
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include "atomicops.h"
#include "readerwriterqueue.h"
#include "xiaozhi_node.hpp"

extern std::shared_ptr<moodycamel::ReaderWriterQueue<std::string>> g_ptr_stt_queue;
extern std::atomic<bool> g_wakeup;              // 唤醒状态
extern std::atomic<bool> g_audio_upload_enable; // 是否向服务端上传麦克风数据