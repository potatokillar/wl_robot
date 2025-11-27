/*
 * @Author: 唐文浩
 * @Date: 2023-07-11
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-18
 * @Description:
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <netinet/in.h>

#include <string>
#include <tuple>
#include <vector>

struct NetClientInfo
{
    std::string serverName; // udp服务器的名字
    std::string clientIp;
    uint16_t clientPort;
    uint32_t lost; // 心跳丢失次数，超过一定次数表明掉线

    // Equality operator
    bool operator==(const NetClientInfo &other) const
    {
        return (clientIp == other.clientIp && clientPort == other.clientPort /* && other member comparisons if any */);
    }

    // Less than operator
    bool operator<(const NetClientInfo &other) const
    {
        return (clientIp < other.clientIp || (clientIp == other.clientIp && clientPort < other.clientPort) /* && other member comparisons if any */);
    }
};

std::pair<bool, struct sockaddr_in> String2Sockaddr(const std::string &ip, uint16_t port);
std::tuple<bool, std::string, uint16_t> Sockaddr2String(const struct sockaddr_in &addr);
