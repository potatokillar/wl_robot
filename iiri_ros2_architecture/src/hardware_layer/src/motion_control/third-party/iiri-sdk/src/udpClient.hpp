/*
 * @Author: 邹应龙
 * @Date: 2022-04-01
 * @LastEditors: zyl jdsb@163.com
 * @LastEditTime: 2024-03-08 14:18:12
 * @Description: UDP客户端
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <array>
#include <boost/asio.hpp>
#include <functional>
#include <string>
#include <vector>

namespace iiri
{
using DelayFunc = std::function<void(const boost::system::error_code)>;  // 定时函数
class UdpClient
{
public:
    UdpClient(const std::string &ip, uint32_t port);
    ~UdpClient();
    void Start();

    bool Send(const std::vector<uint8_t> &data);
    std::vector<uint8_t> Recv();
    void Loop();
    void Stop();
    void AsyncSend(const std::vector<uint8_t> &data);
    void AsyncRecv(std::function<void(const std::vector<uint8_t> &)> func);
    void SetDelayFunc(uint32_t ms, DelayFunc func);

private:
    std::vector<uint8_t> delaySendData_;
    // 最大UDP数据包长度，MTU(1500) - IP头(20) -UDP头(8)
    std::array<uint8_t, 1472 * 5> rxData_;

    boost::asio::io_service ioContext_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint serverEndpoint_;
    boost::asio::deadline_timer timer_{ioContext_};
    DelayFunc delayFunc_;
};
}  // namespace iiri
