#pragma once

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"

namespace dev_server {

/**
 * HTTP API 处理器
 * 提供设备信息查询等 REST API
 */
class HttpApiHandler {
public:
    explicit HttpApiHandler(rclcpp::Node::SharedPtr node);
    ~HttpApiHandler() = default;

    /**
     * 获取设备 UUID（JSON 格式）
     * @return JSON 字符串，格式: {"uuid": "xxx", "deviceName": "xxx", ...}
     */
    std::string getDeviceUUIDJson();

    /**
     * 获取完整设备信息（JSON 格式）
     * @return JSON 字符串
     */
    std::string getDeviceInfoJson();

private:
    rclcpp::Node::SharedPtr node_;
    std::string cachedUUID_;
    bool uuidFetched_;

    /**
     * 从 robot_base 服务获取 UUID
     * @return 成功返回 true
     */
    bool fetchUUIDFromRobotBase();

    /**
     * 从系统中获取 UUID（备用方案）
     * @return UUID 字符串
     */
    std::string getSystemUUID();
};

} // namespace dev_server
