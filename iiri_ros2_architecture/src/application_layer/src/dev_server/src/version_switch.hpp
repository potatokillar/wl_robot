/**
 * @Author: 唐文浩
 * @Date: 2025-11-04
 * @Description: 版本切换处理器 - 提供 ROS2 集群和 QR 控制进程的版本切换功能
 *
 * 功能特性：
 * - 自动备份当前配置
 * - 原子更新符号链接
 * - 自动重启服务
 * - 健康检查
 * - 失败自动回滚
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <memory>
#include "nlohmann/json.hpp"

namespace dev_server {

using json = nlohmann::json;

/**
 * @brief 版本切换结果
 */
struct SwitchResult {
    bool success;                    // 切换是否成功
    std::string message;             // 结果消息
    std::string previousVersion;     // 切换前版本
    std::string currentVersion;      // 切换后版本
    std::string backupPath;          // 配置备份路径
    std::string errorDetails;        // 错误详情（失败时）

    // 转换为 JSON
    json toJson() const {
        json j;
        j["success"] = success;
        j["message"] = message;
        j["previousVersion"] = previousVersion;
        j["currentVersion"] = currentVersion;
        j["backupPath"] = backupPath;
        if (!errorDetails.empty()) {
            j["errorDetails"] = errorDetails;
        }
        return j;
    }
};

/**
 * @brief 版本切换处理器
 *
 * 负责处理 ROS2 集群和 QR 控制进程的版本切换，包括：
 * 1. 配置备份
 * 2. 符号链接更新
 * 3. 服务重启
 * 4. 健康检查
 * 5. 失败回滚
 */
class VersionSwitchHandler {
public:
    /**
     * @brief 构造函数
     * @param node ROS2 节点指针（用于日志）
     */
    explicit VersionSwitchHandler(rclcpp::Node::SharedPtr node);

    /**
     * @brief 切换版本
     * @param type 类型："ros2" 或 "qr"
     * @param targetVersion 目标版本路径（如 "/home/wl/autorun/iiri-ros-arm-6e7a6f3-dirty"）
     * @return SwitchResult 切换结果
     *
     * 流程：
     * 1. 验证目标版本是否存在
     * 2. 获取当前版本
     * 3. 备份当前配置
     * 4. 更新符号链接
     * 5. 重启服务
     * 6. 健康检查
     * 7. 失败时自动回滚
     */
    SwitchResult switchVersion(const std::string& type, const std::string& targetVersion);

    /**
     * @brief 生成切换操作的 JSON 响应
     * @param result 切换结果
     * @return std::string JSON 字符串
     */
    std::string getJsonResponse(const SwitchResult& result);

private:
    rclcpp::Node::SharedPtr node_;   // ROS2 节点

    // === 配置常量 ===

    // 符号链接路径
    const std::string ROS2_SYMLINK = "/home/wl/autorun/iiri-ros";
    const std::string QR_SYMLINK = "/home/wl/autorun/iiri-qr";

    // 服务名称
    const std::string ROS2_SERVICE = "iiri-ros.service";
    const std::string QR_SERVICE = "iiri-qr.service";

    // 备份目录
    const std::string BACKUP_BASE_DIR = "/var/backups/iiri";

    // 健康检查超时（秒）
    const int HEALTH_CHECK_TIMEOUT = 30;

    // === 私有方法 ===

    /**
     * @brief 获取类型对应的符号链接路径
     * @param type "ros2" 或 "qr"
     * @return 符号链接路径
     */
    std::string getSymlinkPath(const std::string& type) const;

    /**
     * @brief 获取类型对应的服务名称
     * @param type "ros2" 或 "qr"
     * @return systemd 服务名称
     */
    std::string getServiceName(const std::string& type) const;

    /**
     * @brief 读取符号链接指向的目标路径
     * @param symlinkPath 符号链接路径
     * @return 目标路径（绝对路径）
     */
    std::string getSymlinkTarget(const std::string& symlinkPath);

    /**
     * @brief 验证目标版本路径是否存在
     * @param targetPath 目标路径
     * @return true 存在, false 不存在
     */
    bool validateTargetPath(const std::string& targetPath);

    /**
     * @brief 备份当前版本的配置文件
     * @param type "ros2" 或 "qr"
     * @param currentVersion 当前版本路径
     * @return 备份路径（失败返回空字符串）
     *
     * 备份内容：
     * - ROS2: install/STAR/share/STAR/config/STAR.yaml
     * - QR: 配置文件
     *
     * 备份路径格式：/var/backups/iiri/TYPE-TIMESTAMP/
     */
    std::string backupCurrentConfig(const std::string& type, const std::string& currentVersion);

    /**
     * @brief 创建备份目录
     * @param type "ros2" 或 "qr"
     * @return 备份目录路径
     */
    std::string createBackupDirectory(const std::string& type);

    /**
     * @brief 复制配置文件到备份目录
     * @param sourceDir 源目录
     * @param backupDir 备份目录
     * @return true 成功, false 失败
     */
    bool copyConfigFiles(const std::string& sourceDir, const std::string& backupDir);

    /**
     * @brief 更新符号链接（原子操作）
     * @param symlinkPath 符号链接路径
     * @param targetPath 新目标路径
     * @return true 成功, false 失败
     *
     * 使用 ln -snf 实现原子更新
     */
    bool updateSymlink(const std::string& symlinkPath, const std::string& targetPath);

    /**
     * @brief 重启 systemd 服务
     * @param serviceName 服务名称
     * @return true 成功, false 失败
     *
     * 执行：sudo systemctl restart {serviceName}
     */
    bool restartService(const std::string& serviceName);

    /**
     * @brief 检查服务健康状态
     * @param serviceName 服务名称
     * @param timeoutSeconds 超时时间（秒）
     * @return true 健康, false 不健康
     *
     * 检查方法：
     * 1. systemctl is-active {serviceName}
     * 2. 等待服务完全启动（循环检查）
     * 3. 超时则认为不健康
     */
    bool checkServiceHealth(const std::string& serviceName, int timeoutSeconds);

    /**
     * @brief 回滚到之前的版本
     * @param type "ros2" 或 "qr"
     * @param previousVersion 之前的版本路径
     * @return true 成功, false 失败
     */
    bool rollback(const std::string& type, const std::string& previousVersion);

    /**
     * @brief 执行系统命令并获取输出
     * @param command 命令字符串
     * @param output 输出字符串（引用）
     * @return 命令退出码
     */
    int executeCommand(const std::string& command, std::string& output);
};

} // namespace dev_server
