/*
 * @Author: 唐文浩
 * @Date: 2024-10-22
 * @LastEditors: 唐文浩
 * @LastEditTime: 2024-12-27
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#ifndef MICROPHONE_H_
#define MICROPHONE_H_

#include <alsa/asoundlib.h>

#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>

#include "DequeTemplate.hpp"
#include "speech_recognition.hpp"

extern std::shared_ptr<DequeTemplate<uint8_t>> g_ptr_byte_queue; // 与麦克风线程共用的容器指针

void process_audio_data(const char *buffer, size_t size, std::ofstream &out);
void CheckOverflow(std::shared_ptr<DequeTemplate<uint8_t>> ptr, int max = 50000); // 检测容器是否过满
int receiveMicData(rclcpp::Logger logger);

#endif