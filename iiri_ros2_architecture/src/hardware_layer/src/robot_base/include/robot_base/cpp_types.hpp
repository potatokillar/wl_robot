/*
 * @Author: 唐文浩
 * @Date: 2025-01-02
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-17
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <cstdint>
#include <cstring>
#include <nlohmann/json.hpp>
#include <sstream>
#include <vector>

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using s8 = int8_t;
using s16 = int16_t;
using s32 = int32_t;
using s64 = int64_t;

// 返回状态，非成功的返回要告知失败原因
enum class RetState
{
    ok,        // 无错误
    netErr,    // 网络错误，该错误只用于sdk，这里只是为了统一
    outRange,  // 超范围错误
    timeout,   // 超时错误
    noSupport, // 功能不被支持错误
    parseErr,  // 协议解析错误
    noUpdate,  // 数据未更新，是旧的数据
    busy,      // 正忙，前一个任务正在执行
    interrupt, // 任务被中断(异常或正常)
    noExist,   // 请求值或功能不存在
    error,     // 其他错误
};

template <typename T>
using Result = std::pair<RetState, T>;

/**
 * @description: C结构体转vector<uint8_t>
 * @return {}
 */
template <typename T>
std::vector<uint8_t> Struct2Vector(const T &in)
{
    const uint8_t *rawData = reinterpret_cast<const uint8_t *>(&in);
    std::vector<uint8_t> data(rawData, rawData + sizeof(T));
    return data;
}

/**
 * @description: vector<uint8_t>转C结构体
 * @return {}
 */
template <typename T>
T Vector2Struct(const std::vector<uint8_t> &in)
{
    T structData;
    std::memcpy(&structData, in.data(), in.size());
    return structData;
}

inline std::string Double2Json(double number, size_t width = 6)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(width) << number;
    return ss.str();
};

inline double Json2Double(nlohmann::json data) { return stod(std::string(data)); };

/**
 * @description: BKDRHash函数
 * https://byvoid.com/zhs/blog/string-hash-compare/
 * @param str
 * @return {}
 */
constexpr u32 BKDRHash(const char *str)
{
    unsigned int seed = 131; // 31 131 1313 13131 131313 etc..
    unsigned int hash = 0;

    while (*str)
    {
        hash = hash * seed + (*str++);
    }

    return (hash & 0x7FFFFFFF);
}

/**
 * @description: 字符串hash操作符
 * @param str
 * @return {}
 */
constexpr u32 operator""_hash(const char *str, std::size_t) { return BKDRHash(str); }