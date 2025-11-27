/*
 * @Author: 唐文浩
 * @Date: 2022-08-04
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-04
 * @Description: realsense的封装库，目前只支持距离读取
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <librealsense2/rs.hpp>
#include <mutex>
#include <thread>

void SaveRealsenseRGB(const std::string &name);

void RealsenseStart();
void RealsenseStop();
std::vector<uint8_t> GetRealsenseRGB();