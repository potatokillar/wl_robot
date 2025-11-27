#include "http_api.hpp"
#include <sstream>
#include <fstream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace dev_server {

HttpApiHandler::HttpApiHandler(rclcpp::Node::SharedPtr node)
    : node_(node), cachedUUID_(""), uuidFetched_(false) {
    RCLCPP_INFO(node_->get_logger(), "HttpApiHandler initialized");
}

std::string HttpApiHandler::getDeviceUUIDJson() {
    // 如果还没有获取过 UUID，尝试获取
    if (!uuidFetched_) {
        if (!fetchUUIDFromRobotBase()) {
            // 如果从 robot_base 获取失败，使用系统 UUID
            cachedUUID_ = getSystemUUID();
            uuidFetched_ = true;
        }
    }

    // 构建 JSON 响应
    std::ostringstream json;
    json << "{";
    json << "\"uuid\":\"" << cachedUUID_ << "\"";
    json << ",\"deviceName\":\"IIRI Robot\"";
    json << ",\"deviceType\":\"quadruped\"";
    json << ",\"timestamp\":" << std::chrono::system_clock::now().time_since_epoch().count();
    json << "}";

    return json.str();
}

std::string HttpApiHandler::getDeviceInfoJson() {
    // 与 getDeviceUUIDJson 相同，可扩展更多信息
    return getDeviceUUIDJson();
}

bool HttpApiHandler::fetchUUIDFromRobotBase() {
    try {
        // 创建 robot_base/my_info 服务客户端
        auto client = node_->create_client<std_srvs::srv::Trigger>(
            "/robot_base/my_info");

        // 等待服务可用（最多 1 秒）
        if (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(node_->get_logger(),
                "Service /robot_base/my_info not available");
            return false;
        }

        // 发送请求
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(request);

        // 等待响应（最多 1 秒）
        auto status = future.wait_for(std::chrono::seconds(1));
        if (status != std::future_status::ready) {
            RCLCPP_WARN(node_->get_logger(),
                "Timeout waiting for /robot_base/my_info response");
            return false;
        }

        auto response = future.get();
        if (response->success) {
            // 从响应中提取 UUID
            // 假设响应格式为 "UUID: xxxx" 或直接是 UUID
            std::string message = response->message;

            // 查找 "UUID:" 标记
            size_t uuidPos = message.find("UUID:");
            if (uuidPos != std::string::npos) {
                size_t start = uuidPos + 5;
                while (start < message.size() && std::isspace(message[start])) {
                    start++;
                }
                size_t end = start;
                while (end < message.size() &&
                       (std::isalnum(message[end]) || message[end] == '_' ||
                        message[end] == '-')) {
                    end++;
                }
                cachedUUID_ = message.substr(start, end - start);
            } else {
                // 如果没有找到 "UUID:" 标记，假设整个消息就是 UUID
                cachedUUID_ = message;
            }

            uuidFetched_ = true;
            RCLCPP_INFO(node_->get_logger(),
                "Device UUID obtained from robot_base: %s", cachedUUID_.c_str());
            return true;
        } else {
            RCLCPP_WARN(node_->get_logger(),
                "Failed to get device UUID from robot_base: %s",
                response->message.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
            "Exception while fetching UUID from robot_base: %s", e.what());
        return false;
    }
}

std::string HttpApiHandler::getSystemUUID() {
    // 尝试从 /etc/machine-id 获取系统 UUID
    std::ifstream machineIdFile("/etc/machine-id");
    if (machineIdFile.is_open()) {
        std::string machineId;
        std::getline(machineIdFile, machineId);
        machineIdFile.close();

        // 取前12位作为 UUID
        if (machineId.length() >= 12) {
            return machineId.substr(0, 12);
        }
        return machineId;
    }

    // 如果无法读取 machine-id，返回默认值
    RCLCPP_WARN(node_->get_logger(),
        "Cannot read /etc/machine-id, using default UUID");
    return "unknown_device";
}

} // namespace dev_server
