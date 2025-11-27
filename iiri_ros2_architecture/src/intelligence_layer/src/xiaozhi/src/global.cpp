/*
 * @Author: 唐文浩
 * @Date: 2025-04-21
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-30
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "global.hpp"

static moodycamel::ReaderWriterQueue<std::string> s_stt_queue(20);
std::shared_ptr<moodycamel::ReaderWriterQueue<std::string>> g_ptr_stt_queue = std::shared_ptr<moodycamel::ReaderWriterQueue<std::string>>(&s_stt_queue);
std::atomic<bool> g_wakeup = {false};
std::atomic<bool> g_audio_upload_enable = {true};