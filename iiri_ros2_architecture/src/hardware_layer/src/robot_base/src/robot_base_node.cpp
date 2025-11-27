
/*
 * @Author: 唐文浩
 * @Date: 2025-01-14
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-07-23
 * @Description: 基础功能节点，负责整个机器人的通用功能
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <mqtt/client.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cstdio>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "cppSqlite.hpp"
#include "interface/srv/my_info.hpp"

// 版本管理相关头文件
#ifdef HAVE_VERSION_CONFIG
#include "git_version_info.hpp" // 生成的版本信息头文件
#endif

using namespace std;
class RobotBase : public rclcpp::Node
{
public:
    RobotBase() : Node("robot_base_node")
    {
// 注册并输出版本信息
#ifdef HAVE_VERSION_CONFIG
        REGISTER_ROS2_PACKAGE_VERSION("robot_base");
#endif

        this->declare_parameter<string>("server_local_ip", "127.0.0.1"); // 本机服务器是必存在的
        this->declare_parameter<string>("server_lan_ip", "");            // 局域网和公网服务器可能不存在
        this->declare_parameter<string>("server_wan_ip", "");
        this->declare_parameter<string>("rtsp_ip", "");                  // RTSP 拉流地址

        vector<string> tmp;
        this->declare_parameter<vector<string>>("net_name", tmp);

        srv_my_info_ = this->create_service<interface::srv::MyInfo>(
            "/robot_base/my_info",
            [this](const std::shared_ptr<interface::srv::MyInfo::Request> request, std::shared_ptr<interface::srv::MyInfo::Response> response)
            {
                this->get_my_info(request, response);
            });

        mqtt_task_ = std::async(std::launch::async, [this]()
                                { this->mqtt_init(); });
    }

    ~RobotBase()
    {
        if (mqtt_client_ && mqtt_client_->is_connected())
        {
            // 主动更新状态为 offline 并推送
            nlohmann::json j = nlohmann::json::parse(device_info_);
            j["status"] = "offline";
            std::string topic = "/" + get_uuid() + "/device/info";
            mqtt_client_->publish(topic, j.dump(), 1, true); // QoS=1, retain=true

            // 优雅断开连接
            auto token = mqtt_client_->disconnect();
            token->wait(); // 等待断开完成
        }
    }

private:
    std::string get_uuid()
    {
        auto [ret, data] = sql_.Read("device", "UUID");
        if (ret == false)
        {
            // 生成 UUID v4
            std::string uuid = boost::uuids::to_string(boost::uuids::random_generator()());
            uuid = uuid.substr(uuid.size() - 12); // 仅使用后12位，足够了
            sql_.AsyncWrite("device", "UUID", "string", uuid);
            return uuid;
        }
        return data;
    }

    /**
     * @description: 服务器地址
     * @return {}
     */
    void get_my_info(const std::shared_ptr<interface::srv::MyInfo::Request> request, std::shared_ptr<interface::srv::MyInfo::Response> response)
    {
        (void)request;
        response->addr[0] = this->get_parameter("server_local_ip").as_string();
        response->addr[1] = this->get_parameter("server_lan_ip").as_string();
        response->addr[2] = this->get_parameter("server_wan_ip").as_string();
        response->addr[3] = this->get_parameter("rtsp_ip").as_string();
        response->uuid = get_uuid();
        RCLCPP_INFO(this->get_logger(), "Robot UUID: '%s'", response->uuid.c_str());
    }

    /**
     * @description: mqtt往阿里云注册，用于IP查找，todo临时，将来未必使用这方案
     * @return {}
     */
    void mqtt_init()
    {
        init_register();

        mqtt::connect_options connOpts;
        connOpts.set_keep_alive_interval(20);
        connOpts.set_clean_session(true);
        connOpts.set_mqtt_version(MQTTVERSION_5);

        auto uuid = get_uuid();
        auto ip = this->get_parameter("server_wan_ip").as_string();

        // 添加LWT设置
        auto will_topic = "/" + uuid + "/device/info";
        auto will = mqtt::message(will_topic, device_info_, 1, true); // QoS1，保留消息
        connOpts.set_will(will);

        int cnt = 0;
        // RCLCPP_INFO(this->get_logger(), "Connecting to MQTT broker... ip: %s", ip.c_str());

        mqtt_client_ = std::make_unique<mqtt::async_client>(ip, uuid, mqtt::create_options(MQTTVERSION_5));

        while (true)
        {
            // 连接本机服务器时，本程序可能比服务器先启动
            try
            {
                auto token = mqtt_client_->connect(connOpts);
                if (token->wait_for(std::chrono::seconds(3)))
                {
                    RCLCPP_INFO(this->get_logger(), "MQTT connected successfully, ip: %s", ip.c_str());
                    break; // 连接成功，退出循环
                }
            }
            catch (const mqtt::exception &)
            {
            }
            RCLCPP_WARN(this->get_logger(), "MQTT connection failed. Retrying... ip: %s", ip.c_str());
            // 10秒后尝试，最长一分钟
            std::this_thread::sleep_for(std::chrono::seconds(10));
            cnt++;
            if (cnt > 5)
            {
                RCLCPP_ERROR(this->get_logger(), "MQTT connection failed too much. ip: %s", ip.c_str());
                break;
            }
        }

        register_me();
    }

    void register_me()
    {
        nlohmann::json j = nlohmann::json::parse(device_info_);
        j["status"] = "online";
        mqtt_client_->publish("/" + get_uuid() + "/device/info", j.dump(), 1, true);
    }

    void init_register()
    {
        CppSqlite sql;
        auto name = sql.Read("device", "name");
        if (name.first == false)
        {
            name.second = "no_init";
        }

        auto description = sql.Read("device", "description");
        if (description.first == false)
        {
            description.second = "no_init";
        }
        nlohmann::json j = nlohmann::json::parse(device_info_);
        j["ip"] = get_host_ip();
        j["uuid"] = get_uuid();
        j["name"] = name.second;
        j["description"] = description.second;
        j["status"] = "offline"; // 默认掉线

        device_info_ = j.dump();
    }

    set<std::string> get_host_ip()
    {
        struct ifaddrs *ifList = nullptr; // getifaddrs()创建的链表上的数据结构
                                          // 获取本地网络接口的信息。
        if (getifaddrs(&ifList) < 0)
        {
            return {};
        }
        auto net_names = this->get_parameter("net_name").as_string_array();
        std::set<std::string> ipset;
        for (struct ifaddrs *ifa = ifList; ifa != nullptr; ifa = ifa->ifa_next)
        {
            // arm平台还需要判断ifa_addr非空
            if ((ifa->ifa_addr) && (ifa->ifa_addr->sa_family == AF_INET))
            {
                // 查看是否是查找的网卡
                if (std::find(net_names.begin(), net_names.end(), ifa->ifa_name) == net_names.end())
                {
                    continue;
                }
                // if (ifa->ifa_name != nullptr) RCLCPP_WARN(this->get_logger(), "ifa_name: %s", ifa->ifa_name);
                struct sockaddr_in *sin = (struct sockaddr_in *)(ifa->ifa_addr);
                char ipstr[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &(sin->sin_addr), ipstr, sizeof(ipstr));
                ipset.insert(ipstr);
            }
        }

        freeifaddrs(ifList);
        return ipset;
    }

private:
    CppSqlite sql_;
    rclcpp::Service<interface::srv::MyInfo>::SharedPtr srv_my_info_;
    std::future<void> mqtt_task_;
    std::unique_ptr<mqtt::async_client> mqtt_client_;

    // 注册的信息
    std::string device_info_ = R"({
        "ip": "",
        "uuid": "",
        "name": "",
        "description": "",
        "status": ""
        })";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotBase>();
    RCLCPP_INFO(node->get_logger(), "Hello key quadruped!");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
