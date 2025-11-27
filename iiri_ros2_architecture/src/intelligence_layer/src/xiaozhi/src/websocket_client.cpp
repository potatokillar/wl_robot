// SPDX-License-Identifier: GPL-3.0-only
/*
 * Copyright (c) 2008-2023 100askTeam : Dongshan WEI <weidongshan@100ask.net>
 * Discourse:  https://forums.100ask.net
 */

/*  Copyright (C) 2008-2023 深圳百问网科技有限公司
 *  All rights reserved
 *
 * 免责声明: 百问网编写的文档, 仅供学员学习使用, 可以转发或引用(请保留作者信息),禁止用于商业用途！
 * 免责声明: 百问网编写的程序, 用于商业用途请遵循GPL许可, 百问网不承担任何后果！
 *
 * 本程序遵循GPL V3协议, 请遵循协议
 * 百问网学习平台   : https://www.100ask.net
 * 百问网交流社区   : https://forums.100ask.net
 * 百问网官方B站    : https://space.bilibili.com/275908810
 * 本程序所用开发板 : Linux开发板
 * 百问网官方淘宝   : https://100ask.taobao.com
 * 联系我们(E-mail) : weidongshan@100ask.net
 *
 *          版权所有，盗版必究。
 *
 * 修改历史     版本号           作者        修改内容
 *-----------------------------------------------------
 * 2025.03.20      v01         百问科技      创建文件
 *-----------------------------------------------------
 */

#include "websocket_client.h"

#include <dirent.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_client.hpp>

#include "json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "xiaozhi_node.hpp"

// 声明全局变量
extern std::shared_ptr<XiaoZhiNode> g_node_ptr;
extern std::shared_ptr<moodycamel::ReaderWriterQueue<std::string>> g_ptr_stt_queue;
extern bool g_service_sound_enable;
// 声明类型别名
typedef websocketpp::client<websocketpp::config::asio_tls_client> client;
typedef websocketpp::lib::shared_ptr<websocketpp::lib::asio::ssl::context> context_ptr;
// 展开命名空间
using websocketpp::lib::bind;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::placeholders::_3;
using websocketpp::lib::placeholders::_4;
using json = nlohmann::json;
// 定义局部静态变量
static client *s_p_ws_client;
static websocketpp::connection_hdl s_hdl;
static websocket_data_t *s_ws_data;
static ws_recv_callback_t s_ws_recv_bin_cb;
static ws_recv_callback_t s_ws_recv_txt_cb;
static volatile int s_iHasShaked = 0;
static volatile int s_iHasConnected = 0;

// 心跳检测间隔(毫秒)
const int HEARTBEAT_INTERVAL = 1000;
// 超时时间(毫秒)
const int PONG_TIMEOUT = 5000;

// 定时器指针类型
typedef std::shared_ptr<websocketpp::lib::asio::steady_timer> timer_ptr;

// 存储每个连接的定时器
std::map<websocketpp::connection_hdl, timer_ptr, std::owner_less<websocketpp::connection_hdl>> heartbeat_timers;
websocketpp::lib::mutex timer_lock;

// 发送Ping帧并设置超时
void send_ping(client *c, websocketpp::connection_hdl hdl)
{
    try {
        client::connection_ptr con = c->get_con_from_hdl(hdl);

        // 检查连接状态
        if (con->get_state() != websocketpp::session::state::open) {
            RCLCPP_WARN(g_node_ptr->get_logger(), "Connection is not open, skipping ping");
            return;
        }

        // 发送Ping帧
        websocketpp::lib::error_code ec;
        con->ping("heartbeat", ec);

        if (ec) {
            RCLCPP_WARN(g_node_ptr->get_logger(), "Error sending ping: %s", ec.message().c_str());
            return;
        }

        // 设置超时时间
        con->set_pong_timeout(PONG_TIMEOUT);
    } catch (const std::exception &e) {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Exception in send_ping: %s", e.what());
    }
}

// 处理Ping超时
void on_pong_timeout(client *c, websocketpp::connection_hdl hdl)
{
    try {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Pong timeout, connection may be lost");

        // 从定时器映射中移除
        {
            websocketpp::lib::lock_guard<websocketpp::lib::mutex> guard(timer_lock);
            heartbeat_timers.erase(hdl);
        }

        // 关闭连接
        client::connection_ptr con = c->get_con_from_hdl(hdl);
        con->close(websocketpp::close::status::policy_violation, "Pong timeout");
        RCLCPP_INFO(g_node_ptr->get_logger(), "Connection closed due to pong timeout");
    } catch (const std::exception &e) {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Exception in on_pong_timeout: %s", e.what());
    }
}

// 安排下一次Ping发送
void schedule_next_ping(client *c, websocketpp::connection_hdl hdl)
{
    try {
        // 创建定时器
        timer_ptr timer = std::make_shared<websocketpp::lib::asio::steady_timer>(
            c->get_io_service(),
            websocketpp::lib::asio::steady_timer::duration(std::chrono::milliseconds(HEARTBEAT_INTERVAL)));

        // 存储定时器
        {
            websocketpp::lib::lock_guard<websocketpp::lib::mutex> guard(timer_lock);
            heartbeat_timers[hdl] = timer;
        }

        // 设置回调
        timer->async_wait([c, hdl](const websocketpp::lib::error_code &ec) {
            if (!ec) {
                send_ping(c, hdl);
            } else {
                RCLCPP_WARN(g_node_ptr->get_logger(), "Timer error: %s", ec.message().c_str());
            }
        });
    } catch (const std::exception &e) {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Exception in schedule_next_ping: %s", e.what());
    }
}

// 处理接收到的Pong帧
void on_pong(client *c, websocketpp::connection_hdl hdl, std::string msg)
{
    (void)msg;
    // 重新安排下一次Ping
    schedule_next_ping(c, hdl);
}

/**
 * 处理接收到的消息
 *
 * @param c 指向WebSocket客户端的指针
 * @param hdl 连接句柄
 * @param msg 接收到的消息
 */
static void on_message(client *c, websocketpp::connection_hdl hdl, client::message_ptr msg)
{
    (void)c;
    (void)hdl;
    // 获取操作码
    auto opcode = msg->get_opcode();
    std::string payload = msg->get_payload();
    if (opcode == websocketpp::frame::opcode::binary) {
        // 处理opus数据:msg->get_payload();
        s_ws_recv_bin_cb(payload.data(), payload.size());
        return;
    }

    json j = json::parse(payload);
    std::string type, text, session_id, state;
    if (j.contains("type") && !j["type"].is_null()) {
        type = j["type"];
    }
    if (j.contains("text") && !j["text"].is_null()) {
        text = j["text"];
    }
    if (j.contains("state") && !j["state"].is_null()) {
        state = j["state"];
    }
    if (j.contains("session_id") && !j["session_id"].is_null()) {
        session_id = j["session_id"];
    }

    try {
        // 处理json数据:msg->get_payload();
        s_ws_recv_txt_cb(payload.data(), payload.size());
    } catch (json::parse_error &e) {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Failed to parse JSON message: %s", e.what());
    } catch (std::exception &e) {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Error processing message: %s", e.what());
    }
}

/**
 * 验证证书的主体备用名称是否匹配给定的主机名
 *
 * @param hostname 主机名
 * @param cert 证书
 * @return 如果匹配则返回true，否则返回false
 */
static bool verify_subject_alternative_name(const char *hostname, X509 *cert)
{
    STACK_OF(GENERAL_NAME) *san_names = NULL;

    san_names = (STACK_OF(GENERAL_NAME) *)X509_get_ext_d2i(cert, NID_subject_alt_name, NULL, NULL);
    if (san_names == NULL) {
        return false;
    }

    int san_names_count = sk_GENERAL_NAME_num(san_names);

    bool result = false;

    for (int i = 0; i < san_names_count; i++) {
        const GENERAL_NAME *current_name = sk_GENERAL_NAME_value(san_names, i);

        if (current_name->type != GEN_DNS) {
            continue;
        }

        char const *dns_name = (char const *)ASN1_STRING_get0_data(current_name->d.dNSName);

        // 确保DNS名称中没有嵌入的NUL字符
        if (ASN1_STRING_length(current_name->d.dNSName) != (int)strlen(dns_name)) {
            break;
        }
        // 比较预期的主机名与CN
        result = (strcasecmp(hostname, dns_name) == 0);
    }
    sk_GENERAL_NAME_pop_free(san_names, GENERAL_NAME_free);

    return result;
}

/**
 * 验证证书的通用名称是否匹配给定的主机名
 *
 * @param hostname 主机名
 * @param cert 证书
 * @return 如果匹配则返回true，否则返回false
 */
static bool verify_common_name(char const *hostname, X509 *cert)
{
    // 查找证书主题字段中的CN字段
    int common_name_loc = X509_NAME_get_index_by_NID(X509_get_subject_name(cert), NID_commonName, -1);
    if (common_name_loc < 0) {
        return false;
    }

    // 提取CN字段
    X509_NAME_ENTRY *common_name_entry = X509_NAME_get_entry(X509_get_subject_name(cert), common_name_loc);
    if (common_name_entry == NULL) {
        return false;
    }

    // 将CN字段转换为C字符串
    ASN1_STRING *common_name_asn1 = X509_NAME_ENTRY_get_data(common_name_entry);
    if (common_name_asn1 == NULL) {
        return false;
    }

    char const *common_name_str = (char const *)ASN1_STRING_get0_data(common_name_asn1);

    // 确保CN中没有嵌入的NUL字符
    if (ASN1_STRING_length(common_name_asn1) != (int)strlen(common_name_str)) {
        return false;
    }

    // 比较预期的主机名与CN
    return (strcasecmp(hostname, common_name_str) == 0);
}

/**
 * 验证证书是否有效
 *
 * @param hostname 主机名
 * @param preverified 预验证结果
 * @param ctx 验证上下文
 * @return 如果验证通过则返回true，否则返回false
 */
static bool verify_certificate(const char *hostname, bool preverified, boost::asio::ssl::verify_context &ctx)
{
    // 验证回调用于检查是否证书有效
    // RFC 2818 描述了如何为HTTPS执行此操作
    // 参考OpenSSL文档以获取更多详细信息

    // 获取当前证书在证书链中的深度
    int depth = X509_STORE_CTX_get_error_depth(ctx.native_handle());

    // 如果是最终证书且预验证通过，则确保主机名匹配SAN或CN
    if (depth == 0 && preverified) {
        X509 *cert = X509_STORE_CTX_get_current_cert(ctx.native_handle());

        if (verify_subject_alternative_name(hostname, cert)) {
            return true;
        } else if (verify_common_name(hostname, cert)) {
            return true;
        } else {
            return false;
        }
    }

    return preverified;
}

/**
 * TLS初始化处理程序
 *
 * @param hostname 主机名
 * @param hdl 连接句柄
 * @return TLS上下文指针
 */
static context_ptr on_tls_init(const char *hostname, websocketpp::connection_hdl)
{
    context_ptr ctx = websocketpp::lib::make_shared<boost::asio::ssl::context>(boost::asio::ssl::context::sslv23);

    try {
        ctx->set_options(boost::asio::ssl::context::default_workarounds | boost::asio::ssl::context::no_sslv2 | boost::asio::ssl::context::no_sslv3 |
                         boost::asio::ssl::context::single_dh_use);

        // 注释掉下面这行，否则会出现TLS握手失败错误
        // ctx->set_verify_mode(boost::asio::ssl::verify_peer);
        ctx->set_verify_callback(bind(&verify_certificate, hostname, ::_1, ::_2));

        // 注释掉下面这行，否则会打印:load_verify_file: No such file or directory
        // ctx->load_verify_file("ca-chain.cert.pem");
    } catch (std::exception &e) {
        RCLCPP_WARN(g_node_ptr->get_logger(), "exit in on_tls_init: %s", e.what());
    }
    return ctx;
}

/**
 * 处理连接打开事件
 *
 * @param c 指向WebSocket客户端的指针
 * @param hdl 连接句柄
 */
static void on_open(client *c, websocketpp::connection_hdl hdl)
{
    s_hdl = hdl;
    s_iHasConnected = 1;
    websocket_data_t *ws_data = s_ws_data;

    RCLCPP_DEBUG(g_node_ptr->get_logger(), "Connection opened");
#if 0    
     std::string hello = R"(
     {
         "type": "hello",
         "version": 1,
         "transport": "websocket",
         "audio_params": {
             "format": "opus",
             "sample_rate": 16000,
             "channels": 1,
             "frame_duration": 60
         }
    })";
#endif
    std::string hello = ws_data->hello;

    try {
        c->send(hdl, hello, websocketpp::frame::opcode::text);

        // 连接建立后立即发送第一个Ping
        send_ping(c, hdl);
    } catch (const websocketpp::lib::error_code &e) {
        RCLCPP_WARN(g_node_ptr->get_logger(), "Error sending message: %d (%s)", e.value(), e.message().c_str());
    }
}

static void on_fail(client *c, websocketpp::connection_hdl hdl)
{
    s_iHasConnected = 0;
    s_iHasShaked = 0;
    client::connection_ptr con = c->get_con_from_hdl(hdl);

    RCLCPP_DEBUG(g_node_ptr->get_logger(), "Connection failed. Code: %d, Reason: %s", con->get_remote_close_code(), con->get_remote_close_reason().c_str());

    // 清理定时器
    {
        websocketpp::lib::lock_guard<websocketpp::lib::mutex> guard(timer_lock);
        heartbeat_timers.erase(hdl);
    }
}

/**
 * 处理连接关闭事件
 *
 * @param c 指向WebSocket客户端的指针
 * @param hdl 连接句柄
 * @param ec 错误代码
 * @param reason 关闭原因
 */
static void on_close(client *c, websocketpp::connection_hdl hdl)
{
    s_iHasConnected = 0;
    s_iHasShaked = 0;
    client::connection_ptr con = c->get_con_from_hdl(hdl);

    RCLCPP_DEBUG(g_node_ptr->get_logger(), "Connection closed. Code: %d, Reason: %s", con->get_remote_close_code(), con->get_remote_close_reason().c_str());
    if (con->get_remote_close_code() == 1005) {
    } else {
        g_service_sound_enable = true;
        g_node_ptr->SendSpeakerWavFile("service_quit");
    }

    // 清理定时器
    {
        websocketpp::lib::lock_guard<websocketpp::lib::mutex> guard(timer_lock);
        heartbeat_timers.erase(hdl);
    }

    // 重新连接逻辑可以在这里实现
    // 例如，等待一段时间后重新启动WebSocket连接
    // std::this_thread::sleep_for(std::chrono::seconds(5)); // 等待5秒后重新连接
    // websocket_start(); // 重新启动WebSocket线程
}

/**
 * 建立WebSocket连接
 *
 * @param c 指向WebSocket客户端的指针
 * @return 错误码
 */
static int websocket_connect(client *c)
{
    websocket_data_t *ws_data = s_ws_data;

    std::string hostname = ws_data->hostname;
    std::string port = ws_data->port;
    std::string path = ws_data->path;

    std::string uri = "wss://" + hostname + ":" + port + path;

    RCLCPP_DEBUG(g_node_ptr->get_logger(), "Connecting to: %s", uri.c_str());

    try {
        // 设置日志级别
        c->clear_access_channels(websocketpp::log::alevel::all);  // 禁用所有访问日志
        c->set_access_channels(websocketpp::log::alevel::app);    // 仅启用应用程序日志
        c->set_error_channels(websocketpp::log::elevel::all);     // 启用所有错误日志

        // 初始化ASIO
        c->init_asio();

        // 注册消息处理程序
        c->set_message_handler(bind(&on_message, c, ::_1, ::_2));
        c->set_tls_init_handler(bind(&on_tls_init, hostname.c_str(), ::_1));

        // 注册连接打开处理程序
        c->set_open_handler(bind(&on_open, c, ::_1));

        // 注册连接关闭处理程序
        c->set_close_handler(bind(&on_close, c, ::_1));

        // 注册连接失败回调函数
        c->set_fail_handler(bind(&on_fail, c, ::_1));

        // 注册Ping超时处理程序
        c->set_pong_timeout_handler(bind(&on_pong_timeout, c, ::_1));

        // 注册Pong接收处理程序
        c->set_pong_handler(bind(&on_pong, c, ::_1, ::_2));

        websocketpp::lib::error_code ec;
        client::connection_ptr con = c->get_connection(uri, ec);
        if (ec) {
            RCLCPP_WARN(g_node_ptr->get_logger(), "could not create connection because: %s", ec.message().c_str());
            return -1;
        }

        // 设置自定义HTTP头
        std::string headers = ws_data->headers;
        RCLCPP_DEBUG(g_node_ptr->get_logger(), "HTTP headers: %s", headers.c_str());
        try {
            // 解析headers字符串为JSON对象
            json headers_json = json::parse(headers);

            // 遍历JSON对象中的每个键值对
            for (json::iterator it = headers_json.begin(); it != headers_json.end(); ++it) {
                std::string key = it.key();
                std::string value = it.value();
                RCLCPP_DEBUG(g_node_ptr->get_logger(), "key: %s, value: %s", key.c_str(), value.c_str());
                con->append_header(key, value);
            }
        } catch (json::parse_error &e) {
            RCLCPP_WARN(g_node_ptr->get_logger(), "Failed to parse headers JSON: %s", e.what());
        }
#if 0
        con->append_header("Authorization", "Bearer test-token");
        con->append_header("Protocol-Version", "1");
        con->append_header("Device-Id", "00:0c:29:bd:43:05");
        con->append_header("Client-Id", "d560294c-01d9-47d0-b538-085f38744b05");
#endif
        // 请求连接
        c->connect(con);

        c->get_alog().write(websocketpp::log::alevel::app, "Connecting to " + uri);
    } catch (websocketpp::exception const &e) {
        RCLCPP_WARN(g_node_ptr->get_logger(), "exit here: %s", e.what());
        return -1;
    }
    return 0;
}

/**
 * 发送二进制数据
 *
 * @param data 数据指针
 * @param size 数据大小
 * @return 错误码
 */
int websocket_send_binary(const char *data, int size)
{
    client *c = s_p_ws_client;
    websocketpp::connection_hdl hdl = s_hdl;

    // 正确检查连接状态
    websocketpp::lib::error_code ec;

    if (!hdl.expired()) {
        auto con = c->get_con_from_hdl(hdl, ec);  // 使用 get_con_from_hdl 获取连接对象
        if (ec || !con || con->get_state() != websocketpp::session::state::open) {
            return 1;  // 连接未处于正常状态，不做任何事，不发送数据，直接返回
        }
    }

    // 发送二进制数据
    if (s_iHasConnected && s_iHasShaked) {
        try {
            c->send(hdl, data, size, websocketpp::frame::opcode::binary);
        } catch (websocketpp::exception const &e) {
            RCLCPP_WARN(g_node_ptr->get_logger(), "exit in websocket_send_binary: %s", e.what());
            websocket_connect(c);
        }
    }
    return 0;
}

/**
 * 发送文本数据
 *
 * @param data 数据指针
 * @param size 数据大小
 * @return 错误码
 */
int websocket_send_text(const char *data, int size)
{
    client *c = s_p_ws_client;
    websocketpp::connection_hdl hdl = s_hdl;

    // 发送文本数据
    if (s_iHasConnected) {
        try {
            c->send(hdl, data, size, websocketpp::frame::opcode::text);
        } catch (websocketpp::exception const &e) {
            RCLCPP_WARN(g_node_ptr->get_logger(), "exit in websocket_send_text: %s", e.what());
            websocket_connect(c);
        }
        s_iHasShaked = 1;
    }
    return 0;
}

/**
 * 设置回调函数和数据
 *
 * @param bin_cb 二进制数据接收回调
 * @param txt_cb 文本数据接收回调
 * @param ws_data WebSocket数据
 * @return 返回值
 */
int websocket_set_callbacks(ws_recv_callback_t bin_cb, ws_recv_callback_t txt_cb, websocket_data_t *ws_data)
{
    s_ws_recv_bin_cb = bin_cb;
    s_ws_recv_txt_cb = txt_cb;
    s_ws_data = ws_data;
    return 0;
}

/**
 * 启动WebSocket线程
 *
 * @return 返回值
 */
int websocket_start()
{
    s_p_ws_client = new client();
    try {
        websocket_connect(s_p_ws_client);
        if (g_node_ptr) {
            if (g_service_sound_enable) {
                g_node_ptr->SendSpeakerWavFile("service_ready");
                g_service_sound_enable = false;
            }
        }
        s_p_ws_client->run();
        s_p_ws_client->stop();
        s_p_ws_client = nullptr;
        RCLCPP_DEBUG(g_node_ptr->get_logger(), "exit from websocket_thread");
    } catch (websocketpp::exception const &e) {
        RCLCPP_WARN(g_node_ptr->get_logger(), "exit here: %s", e.what());
    }

    return 0;
}