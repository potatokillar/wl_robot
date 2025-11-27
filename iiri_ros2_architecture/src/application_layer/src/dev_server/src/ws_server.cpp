/*
 * @Author: 唐文浩
 * @Date: 2025-04-18
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-09
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "ws_server.hpp"

#include "rclcpp/rclcpp.hpp"
using namespace std;

WebsocketServer::WebsocketServer()
{
    // Set logging settings
    server_.set_access_channels(websocketpp::log::alevel::none);
    server_.set_error_channels(websocketpp::log::elevel::warn);

    // Initialize Asio
    server_.init_asio();

    // Register our open handler
    server_.set_open_handler([this](websocketpp::connection_hdl hdl)
                             { this->on_open(hdl); });

    // Register our message handler
    server_.set_message_handler([this](websocketpp::connection_hdl hdl, WsServerType::message_ptr msg)
                                { this->on_message(hdl, msg); });

    // Register our close handler
    server_.set_close_handler([this](websocketpp::connection_hdl hdl)
                              { this->on_close(hdl); });

    // Register our fail handler
    server_.set_fail_handler([this](websocketpp::connection_hdl hdl)
                             { this->on_fail(hdl); });

    server_.set_reuse_addr(true); // 快速复用
}

WebsocketServer::~WebsocketServer()
{
    RCLCPP_INFO(rclcpp::get_logger("ws_server"), "websocket stop listening on port %d", port_);
    server_.stop_listening();
    server_.stop();
}

void WebsocketServer::on_open(websocketpp::connection_hdl hdl)
{
    net_client_info info = get_client_info(hdl);
    if (info.uuid.empty())
    {
        // 拒绝连接
        server_.close(hdl, websocketpp::close::status::policy_violation, "no support");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("ws_server"),
                "client connected success, %s, %d, %s, %s",
                info.ip.c_str(),
                info.port,
                info.role.c_str(),
                info.method.c_str());
    if (open_cb_)
    {
        open_cb_(info);
    }
}

/**
 * @description:
 * @param hdl
 * @param msg
 * @return {}
 */
void WebsocketServer::on_message(websocketpp::connection_hdl hdl, WsServerType::message_ptr msg)
{
    if (msg_bin_cb_)
    {
        if (msg->get_opcode() == websocketpp::frame::opcode::binary)
        {
            std::vector<u8> data(msg->get_payload().begin(), msg->get_payload().end());
            msg_bin_cb_(get_client_info(hdl), data);
        }
    }

    if (msg_str_cb_)
    {
        if (msg->get_opcode() == websocketpp::frame::opcode::text)
        {
            msg_str_cb_(get_client_info(hdl), msg->get_payload());
        }
    }
}

void WebsocketServer::on_close(websocketpp::connection_hdl hdl)
{
    RCLCPP_INFO(rclcpp::get_logger("ws_server"), "websocket client close"); // 断开后就无法访问了
    if (close_cb_)
    {
        close_cb_(get_client_info(hdl));
    }
}

void WebsocketServer::on_fail(websocketpp::connection_hdl hdl)
{
    RCLCPP_INFO(rclcpp::get_logger("ws_server"), "websocket client fail");
    if (close_cb_)
    {
        close_cb_(get_client_info(hdl));
    }
}

void WebsocketServer::loop()
{
    RCLCPP_INFO(rclcpp::get_logger("ws_server"), "websocket start listening on port %d", port_);
    server_.listen(port_);  // 监听指定端口
    server_.start_accept(); // 开始接受连接
    server_.run();          // 启动事件循环
}

bool WebsocketServer::send(const net_client_info &info, const std::vector<uint8_t> &data)
{
    websocketpp::lib::error_code ec;
    server_.send(info.hdl, data.data(), data.size(), websocketpp::frame::opcode::binary, ec);
    if (ec)
    {
        return false;
    }
    return true;
}
bool WebsocketServer::send(const net_client_info &info, const std::string &data)
{
    websocketpp::lib::error_code ec;
    server_.send(info.hdl, data, websocketpp::frame::opcode::text, ec);
    if (ec)
    {
        return false;
    }
    return true;
}

/**
 * @description: 设置连接和断开的回调
 * @param func
 * @return {}
 */
void WebsocketServer::set_open_call(std::function<void(const net_client_info &)> func) { open_cb_ = func; }
void WebsocketServer::set_close_call(std::function<void(const net_client_info &)> func) { close_cb_ = func; }

/**
 * @description: 消息回调
 * @param func
 * @return {}
 */
void WebsocketServer::set_message_call(std::function<void(const net_client_info &, const std::string &)> func) { msg_str_cb_ = func; }
void WebsocketServer::set_message_call(std::function<void(const net_client_info &, const std::vector<uint8_t> &)> func) { msg_bin_cb_ = func; }

/**
 * @description: 获取对端客户端信息
 * @param hdl
 * @return {}
 */
net_client_info WebsocketServer::get_client_info(websocketpp::connection_hdl hdl)
{
    net_client_info info; // 默认是空
    try
    {
        auto conn = server_.get_con_from_hdl(hdl);
        info.ip = conn->get_socket().remote_endpoint().address().to_string(); // ipv6样式的ipv4地址
        info.port = conn->get_socket().remote_endpoint().port();
        info.hdl = hdl;

        auto path = server_.get_con_from_hdl(hdl)->get_resource(); // 获取请求的原始路径
        if (path.size() >= 18)
        {
            info.uuid = path.substr(1, 12); // 对于本机来讲，uuid是没有用的，因为只有一个。这里是为了和非本机的兼容
            info.role = path.substr(14, 3);
            info.method = path.substr(17);
        }
    }
    catch (const std::exception &e)
    {
        //  RCLCPP_INFO(rclcpp::get_logger("ws_server"), "get client info error: %s", e.what());
    }

    return info;
}

void WebsocketServer::stop() { server_.stop(); }