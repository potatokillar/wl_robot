/*
 * @Author: 唐文浩
 * @Date: 2025-04-18
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-23
 * @Description: websocket客户端，不支持多个IP同时连接，但支持同一个IP下多个path todo
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <chrono>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>

#include "type/commType.hpp"

namespace iiri
{
    using DelayFunc = std::function<void(websocketpp::lib::error_code const &)>; // 定时函数
    class WebSocketClient
    {
        using client = websocketpp::client<websocketpp::config::asio_client>;

    public:
        WebSocketClient();
        // WebSocketClient(const std::vector<std::pair<std::string, std::string>> &addrs);
        virtual ~WebSocketClient();

        void Connect(const std::string &ip, uint16_t port, const std::string &path = "");
        void Loop();
        void Stop();

        void SetDelayFunc(uint32_t ms, DelayFunc func);

        // 没有同步接收，会往所有服务端发送
        bool Send(const std::vector<uint8_t> &data);
        bool Send(const std::string &data);

        // 单客户端接收，多客户端下无效
        void AsyncRecv(std::function<void(const std::string &)> func);
        void AsyncRecv(std::function<void(const std::vector<uint8_t> &)> func);

        // 多客户端,todo
        // bool Send(const std::string &name, const std::vector<uint8_t> &data);
        // bool Send(const std::string &name, const std::string &data);
        // void AsyncRecv(std::function<void(const std::string &, const std::string &)> func);
        // void AsyncRecv(std::function<void(const std::string &, const std::vector<uint8_t> &)> func);

        void SetEventCallback(std::function<void(RobotEvent)> func);
        void SetAutoConnect(bool auto_connect);

    private:
        void on_open(websocketpp::connection_hdl hdl);
        void connect(const std::string &name, const std::string &uri);
        void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg);
        void on_close(websocketpp::connection_hdl hdl);
        void on_fail(websocketpp::connection_hdl hdl);
        void reconnect(const std::string &uri);
        void StartReconnect();

    private:
        websocketpp::client<websocketpp::config::asio_client> client_;
        websocketpp::connection_hdl connection_;
        std::string uri_;

        std::function<void(const std::string &)> rx_str_func_;
        std::function<void(const std::vector<uint8_t> &)> rx_bin_func_;

        std::function<void(RobotEvent)> evt_func_;
        bool auto_connect_ = true;
        uint64_t reconnect_dura_ = 100;                    // 重连间隔，最低100ms
        const uint64_t MAX_RECONNECT_DURA = 60 * 1000 * 5; // 最长重连间隔，五分钟一次
    };
} // namespace iiri