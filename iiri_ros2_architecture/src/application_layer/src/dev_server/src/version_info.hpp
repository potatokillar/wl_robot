#pragma once

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"

namespace dev_server {

using json = nlohmann::json;

/**
 * 版本信息处理器
 * 提供 ROS2 集群和 QR 控制进程的版本信息查询
 */
class VersionInfoHandler {
public:
    explicit VersionInfoHandler(rclcpp::Node::SharedPtr node);
    ~VersionInfoHandler() = default;

    /**
     * 获取 ROS2 集群版本信息（JSON 格式）
     * @return JSON 字符串，包含架构、版本号、构建时间等信息
     */
    std::string getROS2VersionJson();

    /**
     * 获取 QR 控制进程版本信息（JSON 格式）
     * @return JSON 字符串，包含版本信息（如果可用）
     */
    std::string getQRVersionJson();

    /**
     * 获取版本历史列表（JSON 格式）
     * @return JSON 字符串，包含所有已部署版本的列表
     */
    std::string getVersionHistoryJson();

    /**
     * 获取当前激活版本信息（JSON 格式）
     * @return JSON 字符串，包含当前符号链接指向的版本
     */
    std::string getCurrentVersionJson();

private:
    rclcpp::Node::SharedPtr node_;

    /**
     * 解析 VERSION.txt 文件
     * @param filepath VERSION.txt 文件路径
     * @return 解析后的 JSON 对象
     */
    json parseVersionFile(const std::string& filepath);

    /**
     * 获取符号链接的目标路径
     * @param symlinkPath 符号链接路径
     * @return 符号链接指向的实际路径
     */
    std::string getSymlinkTarget(const std::string& symlinkPath);

    /**
     * 扫描版本目录列表
     * @param baseDir 基础目录路径（如 /home/wl/autorun）
     * @param prefix 目录前缀（如 "iiri-ros-"）
     * @return 版本目录路径列表
     */
    std::vector<std::string> scanVersionDirs(const std::string& baseDir, const std::string& prefix);

    /**
     * 从目录名提取版本信息
     * @param dirname 目录名（如 "iiri-ros-arm-65e7322-dirty"）
     * @return 包含架构和版本的 JSON 对象
     */
    json extractVersionFromDirname(const std::string& dirname);
};

} // namespace dev_server
