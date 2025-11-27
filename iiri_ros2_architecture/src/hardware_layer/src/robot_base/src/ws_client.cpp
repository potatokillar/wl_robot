/*
 * @Author: 唐文浩
 * @Date: 2025-04-18
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-05-21
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "ws_client.hpp"

#include <exception>
using namespace std;

/**
 * @description: 支持0个或多个path客户端
 * @param addr
 * @param method
 * @return {}
 */
WebSocketClient::WebSocketClient(const std::string &addr, const std::string &uuid, const std::string &role)
{
    init();
    connect(addr, uuid, "", role); // 空，即addr/uuid
}
WebSocketClient::WebSocketClient(const std::string &addr, const std::string &uuid, const std::string &method, const std::string &role)
{
    init();
    connect(addr, uuid, method, role);
}
WebSocketClient::WebSocketClient(const std::string &addr, const std::string &uuid, const std::vector<std::string> &paths, const std::string &role)
{
    init();
    for (const auto &method : paths)
    {
        connect(addr, uuid, method, role);
    }
}

WebSocketClient::~WebSocketClient() { stop(); }

void WebSocketClient::init()
{
    // Set logging settings
    client_.set_access_channels(websocketpp::log::alevel::none);
    client_.set_error_channels(websocketpp::log::elevel::info);

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

void WebSocketClient::on_open(websocketpp::connection_hdl hdl)
{
    (void)hdl;
    isOpen_ = true;
}
void WebSocketClient::on_close(websocketpp::connection_hdl hdl)
{
    (void)hdl;
    isOpen_ = false;
}
void WebSocketClient::on_fail(websocketpp::connection_hdl hdl)
{
    (void)hdl;
    isOpen_ = false;
}

/**
 * @description: 接收回调函数
 * @param hdl
 * @param msg
 * @return {}
 */
void WebSocketClient::on_message(websocketpp::connection_hdl hdl, client::message_ptr msg)
{
    auto path = client_.get_con_from_hdl(hdl)->get_resource();
    if (path.size() < 17)
    {
        // /<12位uuid>/<3位role>，长度17，path可为空，因此原始path必须>=17
        return;
    }
    // 删除前面17位
    auto method = path.substr(17);
    if (msg->get_opcode() == websocketpp::frame::opcode::text)
    {
        if (rx_str_funcs_.find(method) != rx_str_funcs_.end())
        {
            rx_str_funcs_[method](msg->get_payload());
        }
        if (rx_str_func_)
        {
            rx_str_func_(method, msg->get_payload());
        }
    }
    else if (msg->get_opcode() == websocketpp::frame::opcode::binary)
    {
        std::vector<uint8_t> data(msg->get_payload().begin(), msg->get_payload().end());
        if (rx_bin_funcs_.find(method) != rx_bin_funcs_.end())
        {
            rx_bin_funcs_[method](data);
        }
        if (rx_bin_func_)
        {
            rx_bin_func_(method, data);
        }
    }
}
/**
 * @description: 连接
 * @param addr 地址
 * @param uuid 标识符
 * @param method 方法
 * @return {}
 */
void WebSocketClient::connect(const std::string &addr, const std::string &uuid, const std::string &method, const std::string &role)
{
    if ((method.size() > 0) && (method[0] != '/'))
    {
        throw std::invalid_argument("method must start with '/'");
    }
    std::string uri = "ws://" + addr + ":20555" + "/" + uuid + "/" + role + method;
    websocketpp::lib::error_code ec;
    client::connection_ptr con = client_.get_connection(uri, ec);
    client_.connect(con);
    connections_[method] = con;
}

/**
 * @description: 往所有服务端发送字符串信息
 * @param &data
 * @return {}
 */
bool WebSocketClient::send(const std::string &data)
{
    bool ret = true;
    websocketpp::lib::error_code ec; // 不带这个的函数，会抛出异常
    for (auto &pair : connections_)
    {
        auto &hdl = pair.second;
        client_.send(hdl, data, websocketpp::frame::opcode::text, ec);
        if (ec)
        {
            ret = false;
        }
    }
    return ret;
}

bool WebSocketClient::send(const std::string &method, const std::string &data)
{
    bool ret = true;
    websocketpp::lib::error_code ec; // 不带这个的函数，会抛出异常
    if (connections_.find(method) != connections_.end())
    {
        client_.send(connections_[method], data, websocketpp::frame::opcode::text, ec);
        if (ec)
        {
            ret = false;
        }
    }

    return ret;
}

/**
 * @description: 往所有服务端发送二进制信息
 * @return {}
 */
bool WebSocketClient::send(const std::vector<uint8_t> &data)
{
    bool ret = true;
    websocketpp::lib::error_code ec; // 不带这个的函数，会抛出异常
    for (auto &pair : connections_)
    {
        auto &hdl = pair.second;
        client_.send(hdl, data.data(), data.size(), websocketpp::frame::opcode::binary, ec);
        if (ec)
        {
            ret = false;
        }
    }
    return ret;
}

bool WebSocketClient::send(const std::string &method, const std::vector<uint8_t> &data)
{
    bool ret = true;
    websocketpp::lib::error_code ec; // 不带这个的函数，会抛出异常
    if (connections_.find(method) != connections_.end())
    {
        client_.send(connections_[method], data.data(), data.size(), websocketpp::frame::opcode::binary, ec);
        if (ec)
        {
            ret = false;
        }
    }

    return ret;
}

/**
 * @description: 注册字符串和二进制回调函数，对具体的path注册
 * @param func
 * @return {}
 */
void WebSocketClient::async_recv(const std::string &method, std::function<void(const std::string &)> func) { rx_str_funcs_[method] = func; }
void WebSocketClient::async_recv(const std::string &method, std::function<void(const std::vector<uint8_t> &)> func) { rx_bin_funcs_[method] = func; }

/**
 * @description: 对所有path注册
 * @param func
 * @return {}
 */
void WebSocketClient::async_recv(std::function<void(const std::string &, const std::string &)> func) { rx_str_func_ = func; }
void WebSocketClient::async_recv(std::function<void(const std::string &, const std::vector<uint8_t> &)> func) { rx_bin_func_ = func; }

void WebSocketClient::loop() { client_.run(); }
void WebSocketClient::stop() { client_.stop(); }
