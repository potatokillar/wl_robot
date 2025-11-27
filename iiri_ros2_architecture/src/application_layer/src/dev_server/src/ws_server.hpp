/*
 * @Author: 唐文浩
 * @Date: 2025-04-18
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-09
 * @Description: websocket服务端
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <future>
#include <iostream>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include "robot_base/cpp_types.hpp"

// 已连接的客户端信息
struct net_client_info
{
    websocketpp::connection_hdl hdl;
    // 机器人侧非公网IP，因此一般只支持局域网接入
    std::string ip{""};
    uint16_t port{0};
    std::string uuid{""};
    std::string role{""};
    std::string method{""}; // 包含/
};

inline bool operator<(const net_client_info &lhs, const net_client_info &rhs) { return lhs.ip < rhs.ip || lhs.port < rhs.port || lhs.method < rhs.method; }

class WebsocketServer
{
    using WsServerType = websocketpp::server<websocketpp::config::asio>;

public:
    WebsocketServer();
    ~WebsocketServer();
    void loop();
    void stop();

    // 发送数据
    bool send(const net_client_info &info, const std::string &data);
    bool send(const net_client_info &info, const std::vector<uint8_t> &data);

    // 设置消息回调
    void set_message_call(std::function<void(const net_client_info &, const std::string &)> func);
    void set_message_call(std::function<void(const net_client_info &, const std::vector<uint8_t> &)> func);

    // 设置连接和断开回调
    void set_open_call(std::function<void(const net_client_info &)> func);
    void set_close_call(std::function<void(const net_client_info &)> func);

private:
    void on_open(websocketpp::connection_hdl hdl);
    void on_message(websocketpp::connection_hdl hdl, WsServerType::message_ptr msg);
    void on_close(websocketpp::connection_hdl hdl);
    void on_fail(websocketpp::connection_hdl hdl);
    net_client_info get_client_info(websocketpp::connection_hdl hdl);

private:
    WsServerType server_;
    int port_{20555}; // 监听端口

    std::function<void(const net_client_info &)> open_cb_, close_cb_;
    std::function<void(const net_client_info &, const std::string &)> msg_str_cb_;
    std::function<void(const net_client_info &, const std::vector<uint8_t> &)> msg_bin_cb_;
};
