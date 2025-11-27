/*
 * @Author: 唐文浩
 * @Date: 2025-04-18
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-05-21
 * @Description: websocket客户端，但只支持和一个服务端通信
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

class WebSocketClient
{
    using client = websocketpp::client<websocketpp::config::asio_client>;

public:
    // 默认是dev 角色
    WebSocketClient(const std::string &addr, const std::string &uuid, const std::string &role = "dev");
    WebSocketClient(const std::string &addr, const std::string &uuid, const std::string &path, const std::string &role = "dev");
    WebSocketClient(const std::string &addr, const std::string &uuid, const std::vector<std::string> &paths, const std::string &role = "dev");
    ~WebSocketClient();
    void loop();

    // 给所有连接发送
    bool send(const std::vector<uint8_t> &data);
    bool send(const std::string &data);

    // 只给对应的path发送
    bool send(const std::string &path, const std::vector<uint8_t> &data);
    bool send(const std::string &path, const std::string &data);

    // 只注册对应的path
    void async_recv(const std::string &path, std::function<void(const std::string &)> func);
    void async_recv(const std::string &path, std::function<void(const std::vector<uint8_t> &)> func);

    // 对所有的path注册
    void async_recv(std::function<void(const std::string &, const std::string &)> func);
    void async_recv(std::function<void(const std::string &, const std::vector<uint8_t> &)> func);

private:
    void init();
    void stop();
    void on_open(websocketpp::connection_hdl hdl);
    void connect(const std::string &name, const std::string &uuid, const std::string &path, const std::string &role);
    void on_message(websocketpp::connection_hdl hdl, client::message_ptr msg);
    void on_close(websocketpp::connection_hdl hdl);
    void on_fail(websocketpp::connection_hdl hdl);

private:
    websocketpp::client<websocketpp::config::asio_client> client_;
    std::map<std::string, websocketpp::connection_hdl> connections_;

    std::function<void(const std::string &, const std::string &)> rx_str_func_;
    std::function<void(const std::string &, const std::vector<uint8_t> &)> rx_bin_func_;

    std::map<const std::string, std::function<void(const std::string &)>> rx_str_funcs_;
    std::map<const std::string, std::function<void(const std::vector<uint8_t> &)>> rx_bin_funcs_;

    bool isOpen_ = false;
    std::string uuid_;
};
