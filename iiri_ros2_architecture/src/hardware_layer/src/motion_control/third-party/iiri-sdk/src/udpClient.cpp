/*
 * @Author: 邹应龙
 * @Date: 2022-04-01
 * @LastEditors: 邹应龙
 * @LastEditTime: 2024-11-04
 * @Description: UDP客户端的实现
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "udpClient.hpp"

#include <iostream>
#include <string>

#include "protocol.hpp"
using namespace std;
using namespace iiri;
using namespace boost;

UdpClient::UdpClient(const std::string &ip, uint32_t port) : socket_(ioContext_, asio::ip::udp::endpoint(asio::ip::udp::v4(), 0))
{
    try {
        serverEndpoint_ = asio::ip::udp::endpoint(asio::ip::address_v4::from_string(ip), port);
    } catch (const system::system_error &e) {
    }
}

UdpClient::~UdpClient() { Stop(); }

/**
 * @description: 同步发送
 * @param &data 数据包字节流
 * @return {}
 */
bool UdpClient::Send(const std::vector<uint8_t> &data)
{
    try {
        socket_.send_to(asio::buffer(data), serverEndpoint_);
    } catch (const system::system_error &e) {
        cout << e.what() << endl;
        return false;
    }
    return true;
}

/**
 * @description: 同步接收
 * @return {} 内部UDP缓存的常量引用
 */
std::vector<uint8_t> UdpClient::Recv()
{
    auto rxCnt = socket_.receive_from(boost::asio::buffer(rxData_), serverEndpoint_);
    return vector<uint8_t>(rxData_.begin(), rxData_.begin() + rxCnt);
}

/**
 * @description: 异步操作时，需要开启异步轮询
 * @return {}
 */
void UdpClient::Loop() { ioContext_.run(); }

/**
 * @description: 停止，异步操作时，析构前需要停止。让Loop退出
 * @return {}
 */
void UdpClient::Stop()
{
    socket_.cancel();
    ioContext_.stop();
}

/**
 * @description: 异步发送
 * @param &data
 * @return {}
 */
void UdpClient::AsyncSend(const std::vector<uint8_t> &data)
{
    socket_.async_send_to(boost::asio::buffer(data), serverEndpoint_, [this](boost::system::error_code ec, std::size_t) {
        if (ec) {
            std::cout << "Send error: " << ec.message() << std::endl;
        }
    });
}

/**
 * @description: 异步接收，仅一次，需要在回调函数中再次调用该函数才能继续接收
 * @param & 异步接收回调函数
 * @return {}
 */
void UdpClient::AsyncRecv(std::function<void(const std::vector<uint8_t> &)> func)
{
    socket_.async_receive_from(boost::asio::buffer(rxData_), serverEndpoint_,
                               [this, func](const boost::system::error_code &error, std::size_t rxCnt) {
                                   (void)error;
                                   func(vector<uint8_t>(rxData_.begin(), rxData_.begin() + rxCnt));
                               });
}

/**
 * @description: 延迟发送
 * 仅一次发送，多次发送需要多次调用。
 * 仅支持一条数据，会取消上一次未完成的延迟发送
 * @param ms
 * @param &data
 * @return {}
 */
void UdpClient::SetDelayFunc(uint32_t ms, DelayFunc func)
{
    timer_.expires_from_now(boost::posix_time::milliseconds(ms));  // 根据官方文档，该操作会取消未完成的异步
    delayFunc_ = func;
    timer_.async_wait([this](const boost::system::error_code &e) { this->delayFunc_(e); });
}
