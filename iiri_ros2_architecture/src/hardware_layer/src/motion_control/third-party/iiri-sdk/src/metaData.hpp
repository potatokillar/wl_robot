/*
 * @Author: 唐文浩-0036
 * @Date: 2023-07-07
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-03-03
 * @Description: 元数据
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <string_view>

// 协议版本提升，SDK提第一二个版本号；修bug，提第三个版本号
inline constexpr std::string_view SDK_VERSION = "iiri.sdk.version: v1.16.0";
inline constexpr std::string_view PROTOCOL_VERSION = "iiri.protocol.version: v2.11";