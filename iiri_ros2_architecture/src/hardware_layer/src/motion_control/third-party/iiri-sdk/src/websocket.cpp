/*
 * @Author: 唐文浩
 * @Date: 2025-04-18
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-23
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "websocket.hpp"

using namespace iiri;

WebSocketClient::WebSocketClient()
{
    // Set logging settings
    client_.set_access_channels(websocketpp::log::alevel::none);
    client_.set_error_channels(websocketpp::log::elevel::none);

    // Initialize Asio
    client_.init_asio();

    // Register our open handler
    client_.set_open_handler([this](websocketpp::connection_hdl hdl)
                             { this->on_open(hdl); });

    // Register our message handler
    client_.set_message_handler([this](websocketpp::connection_hdl hdl, client::message_ptr msg)
                                { this->on_message(hdl, msg); });

    // Register our close handler
    client_.set_close_handler([this](websocketpp::connection_hdl hdl)
                              { this->on_close(hdl); });

    // Register our fail handler
    client_.set_fail_handler([this](websocketpp::connection_hdl hdl)
                             { this->on_fail(hdl); });
}

WebSocketClient::~WebSocketClient() { Stop(); }

/**
 * @description:
 * @param ip
 * @param port
 * @param path 带/
 * @return {}
 */
void WebSocketClient::Connect(const std::string &ip, uint16_t port, const std::string &path)
{
    std::string addr = "ws://" + ip + ":" + std::to_string(port) + path;
    connect("iiri", addr);
}

void WebSocketClient::on_open(websocketpp::connection_hdl hdl)
{
    (void)hdl;
    if (evt_func_)
    {
        evt_func_(RobotEvent::connect);
    }
    reconnect_dura_ = 100;
}
void WebSocketClient::on_close(websocketpp::connection_hdl hdl)
{
    (void)hdl;
    StartReconnect();
    if (evt_func_)
    {
        evt_func_(RobotEvent::disconnect);
    }
}
void WebSocketClient::on_fail(websocketpp::connection_hdl hdl)
{
    (void)hdl;
    StartReconnect();
    if (evt_func_)
    {
        evt_func_(RobotEvent::disconnect);
    }
}

/**
 * @description: 接收回调函数
 * @param hdl
 * @param msg
 * @return {}
 */
void WebSocketClient::on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
{
    (void)hdl;
    if (msg->get_opcode() == websocketpp::frame::opcode::text)
    {
        if (rx_str_func_)
        {
            rx_str_func_(msg->get_payload());
        }
    }
    else if (msg->get_opcode() == websocketpp::frame::opcode::binary)
    {
        if (rx_bin_func_)
        {
            std::vector<uint8_t> data(msg->get_payload().begin(), msg->get_payload().end());
            rx_bin_func_(data);
        }
    }
}
/**
 * @description: 连接
 * @param name 对端名字
 * @param uri 地址
 * @return {}
 */
void WebSocketClient::connect(const std::string &name, const std::string &uri)
{
    (void)name;
    uri_ = uri;
    websocketpp::lib::error_code ec;
    client::connection_ptr con = client_.get_connection(uri, ec);
    client_.connect(con);
    connection_ = con;
}

/**
 * @description: 往所有服务端发送字符串信息
 * @param &data
 * @return {}
 */
bool WebSocketClient::Send(const std::string &data)
{
    bool ret = true;
    websocketpp::lib::error_code ec; // 不带这个的函数，会抛出异常
    client_.send(connection_, data, websocketpp::frame::opcode::text, ec);
    if (ec)
    {
        ret = false;
    }

    return ret;
}

/**
 * @description: 往所有服务端发送二进制信息
 * @return {}
 */
bool WebSocketClient::Send(const std::vector<uint8_t> &data)
{
    bool ret = true;
    websocketpp::lib::error_code ec; // 不带这个的函数，会抛出异常
    client_.send(connection_, data.data(), data.size(), websocketpp::frame::opcode::binary, ec);
    if (ec)
    {
        ret = false;
    }

    return ret;
}

/**
 * @description: 注册字符串和二进制回调函数
 * @param func
 * @return {}
 */
void WebSocketClient::AsyncRecv(std::function<void(const std::string &)> func) { rx_str_func_ = func; }
void WebSocketClient::AsyncRecv(std::function<void(const std::vector<uint8_t> &)> func) { rx_bin_func_ = func; }

void WebSocketClient::Loop() { client_.run(); }
void WebSocketClient::Stop() { client_.stop(); }

/**
 * @description: 设置延迟发送，仅发送一次
 * @param ms
 * @param func
 * @return {}
 */
void WebSocketClient::SetDelayFunc(uint32_t ms, DelayFunc func) { client_.set_timer(ms, func); }

void WebSocketClient::SetEventCallback(std::function<void(iiri::RobotEvent)> func) { evt_func_ = func; }

void WebSocketClient::SetAutoConnect(bool auto_connect) { auto_connect_ = auto_connect; }

void WebSocketClient::reconnect(const std::string &uri)
{
    reconnect_dura_ *= 2; // 每次翻倍，指数退避算法
    connect("iiri", uri);
}

void WebSocketClient::StartReconnect()
{
    if (reconnect_dura_ > MAX_RECONNECT_DURA)
    {
        reconnect_dura_ = MAX_RECONNECT_DURA;
    }
    // 最长重连间隔5分钟
    if (auto_connect_)
    {
        client_.set_timer(reconnect_dura_, [this](websocketpp::lib::error_code const &)
                          { this->reconnect(this->uri_); });
    }
}