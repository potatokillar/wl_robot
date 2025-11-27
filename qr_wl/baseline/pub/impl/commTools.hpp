
#pragma once

#include "cppType.hpp"

constexpr u32 BKDRHash(const char* str)
{
    unsigned int seed = 131;  // 31 131 1313 13131 131313 etc..
    unsigned int hash = 0;

    while (*str) {
        hash = hash * seed + (*str++);
    }

    return (hash & 0x7FFFFFFF);
}

/**
 * @description: 字符串hash操作符
 * @param str
 * @return {}
 */
constexpr u32 operator""_hash(const char* str, std::size_t) { return BKDRHash(str); }

template <typename T>
constexpr std::size_t GetHash(T hash)
{
    if constexpr (std::is_pointer<T>::value == true) {
        return reinterpret_cast<std::size_t>(hash);
    } else {
        return std::hash<T>{}(hash);
    }
}
