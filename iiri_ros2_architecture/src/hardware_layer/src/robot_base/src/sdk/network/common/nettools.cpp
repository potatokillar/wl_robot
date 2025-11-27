/*
 * @Author: 唐文浩
 * @Date: 2023-07-11
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-18
 * @Description:
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <arpa/inet.h>

#include "netType.hpp"

std::pair<bool, struct sockaddr_in> String2Sockaddr(const std::string &ip, uint16_t port)
{
    struct sockaddr_in sockaddr;
    // 将IP地址转换为网络字节序
    if (inet_pton(AF_INET, ip.c_str(), &(sockaddr.sin_addr)) != 1)
    {
        return {false, sockaddr}; // IP地址转换失败
    }

    // 填充端口号，需要进行字节序转换
    sockaddr.sin_port = htons(port);

    // 设置地址族为IPv4
    sockaddr.sin_family = AF_INET;

    return {true, sockaddr};
}
std::tuple<bool, std::string, uint16_t> Sockaddr2String(const struct sockaddr_in &sockaddr)
{
    // 解析IP地址
    char ipBuffer[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &(sockaddr.sin_addr), ipBuffer, INET_ADDRSTRLEN) == nullptr)
    {
        return {false, "null", 0}; // IP地址解析失败
    }
    std::string ip = ipBuffer;

    // 解析端口号，需要进行字节序转换
    uint16_t port = ntohs(sockaddr.sin_port);

    return {true, ip, port};
}