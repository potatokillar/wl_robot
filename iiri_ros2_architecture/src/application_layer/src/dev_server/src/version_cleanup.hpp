/**
 * @Author: 唐文浩
 * @Date: 2025-11-04
 * @Description: 版本清理处理器 - 自动清理旧版本，保留最近 N 个版本
 *
 * 功能特性：
 * - 扫描所有版本目录
 * - 按构建日期排序
 * - 保留最近 N 个版本
 * - 永远不删除当前激活版本
 * - 计算可节省的磁盘空间
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
 * @brief 版本信息
 */
struct VersionEntry {
    std::string dirName;          // 目录名（如 "iiri-ros-arm-6e7a6f3-dirty"）
    std::string fullPath;         // 完整路径
    std::string buildDate;        // 构建日期（从 VERSION.txt 读取）
    uint64_t size;                // 目录大小（字节）
    bool isActive;                // 是否为当前激活版本

    // 用于排序（按构建日期降序）
    bool operator<(const VersionEntry& other) const {
        return buildDate > other.buildDate;  // 降序：最新的在前
    }
};

/**
 * @brief 版本清理结果
 */
struct CleanupResult {
    bool success;                           // 清理是否成功
    std::string message;                    // 结果消息
    std::vector<std::string> deletedVersions;   // 已删除的版本
    std::vector<std::string> remainingVersions; // 保留的版本
    uint64_t freedSpace;                    // 释放的磁盘空间（字节）
    std::string errorDetails;               // 错误详情（失败时）

    // 转换为 JSON
    json toJson() const {
        json j;
        j["success"] = success;
        j["message"] = message;
        j["deletedVersions"] = deletedVersions;
        j["remainingVersions"] = remainingVersions;
        j["freedSpaceMB"] = freedSpace / (1024 * 1024);  // 转换为 MB
        if (!errorDetails.empty()) {
            j["errorDetails"] = errorDetails;
        }
        return j;
    }
};

/**
 * @brief 版本清理分析结果（不执行删除，仅分析）
 */
struct CleanupAnalysis {
    int totalVersions;                      // 总版本数
    int versionsToDelete;                   // 将要删除的版本数
    std::vector<std::string> versionList;   // 将要删除的版本列表
    uint64_t spaceToFree;                   // 将释放的磁盘空间（字节）
    std::string activeVersion;              // 当前激活版本

    // 转换为 JSON
    json toJson() const {
        json j;
        j["totalVersions"] = totalVersions;
        j["versionsToDelete"] = versionsToDelete;
        j["versionList"] = versionList;
        j["spaceToFreeMB"] = spaceToFree / (1024 * 1024);  // 转换为 MB
        j["activeVersion"] = activeVersion;
        return j;
    }
};

/**
 * @brief 版本清理处理器
 *
 * 负责清理旧版本，保留最近 N 个版本（默认 5 个）
 */
class VersionCleanupHandler {
public:
    /**
     * @brief 构造函数
     * @param node ROS2 节点指针（用于日志）
     */
    explicit VersionCleanupHandler(rclcpp::Node::SharedPtr node);

    /**
     * @brief 分析版本清理（不执行删除）
     * @param type 类型："ros2" 或 "qr"
     * @param keepCount 保留版本数量（默认 5）
     * @return CleanupAnalysis 分析结果
     *
     * 用于在删除前向用户展示将要删除的版本和释放的空间
     */
    CleanupAnalysis analyzeCleanup(const std::string& type, int keepCount = 5);

    /**
     * @brief 执行版本清理
     * @param type 类型："ros2" 或 "qr"
     * @param keepCount 保留版本数量（默认 5）
     * @return CleanupResult 清理结果
     *
     * 流程：
     * 1. 扫描所有版本目录
     * 2. 获取当前激活版本
     * 3. 按构建日期排序
     * 4. 标记要删除的版本（排除激活版本）
     * 5. 计算磁盘空间
     * 6. 执行删除
     */
    CleanupResult cleanupOldVersions(const std::string& type, int keepCount = 5);

    /**
     * @brief 删除指定版本
     * @param type 类型："ros2" 或 "qr"
     * @param versionPath 版本完整路径
     * @return CleanupResult 清理结果
     *
     * 用于手动删除单个版本
     */
    CleanupResult deleteVersion(const std::string& type, const std::string& versionPath);

    /**
     * @brief 生成清理操作的 JSON 响应
     * @param result 清理结果
     * @return std::string JSON 字符串
     */
    std::string getJsonResponse(const CleanupResult& result);

    /**
     * @brief 生成分析结果的 JSON 响应
     * @param analysis 分析结果
     * @return std::string JSON 字符串
     */
    std::string getAnalysisJson(const CleanupAnalysis& analysis);

private:
    rclcpp::Node::SharedPtr node_;   // ROS2 节点

    // === 配置常量 ===

    // 基础目录
    const std::string AUTORUN_DIR = "/home/wl/autorun";

    // 版本目录前缀
    const std::string ROS2_PREFIX = "iiri-ros-";
    const std::string QR_PREFIX = "iiri-qr-";

    // 符号链接路径
    const std::string ROS2_SYMLINK = "/home/wl/autorun/iiri-ros";
    const std::string QR_SYMLINK = "/home/wl/autorun/iiri-qr";

    // === 私有方法 ===

    /**
     * @brief 获取类型对应的目录前缀
     * @param type "ros2" 或 "qr"
     * @return 目录前缀
     */
    std::string getDirPrefix(const std::string& type) const;

    /**
     * @brief 获取类型对应的符号链接路径
     * @param type "ros2" 或 "qr"
     * @return 符号链接路径
     */
    std::string getSymlinkPath(const std::string& type) const;

    /**
     * @brief 扫描所有版本目录
     * @param baseDir 基础目录
     * @param prefix 目录前缀
     * @return 版本信息列表
     */
    std::vector<VersionEntry> scanVersions(const std::string& baseDir, const std::string& prefix);

    /**
     * @brief 读取版本目录的 VERSION.txt 获取构建日期
     * @param versionPath 版本路径
     * @return 构建日期字符串（失败返回空）
     */
    std::string getBuildDate(const std::string& versionPath);

    /**
     * @brief 计算目录大小
     * @param dirPath 目录路径
     * @return 大小（字节）
     */
    uint64_t calculateDirSize(const std::string& dirPath);

    /**
     * @brief 读取符号链接指向的目标路径
     * @param symlinkPath 符号链接路径
     * @return 目标路径（绝对路径）
     */
    std::string getSymlinkTarget(const std::string& symlinkPath);

    /**
     * @brief 选择要删除的版本
     * @param versions 所有版本列表（已排序）
     * @param activeVersion 当前激活版本路径
     * @param keepCount 保留数量
     * @return 要删除的版本列表
     */
    std::vector<VersionEntry> selectVersionsToDelete(
        const std::vector<VersionEntry>& versions,
        const std::string& activeVersion,
        int keepCount
    );

    /**
     * @brief 删除版本目录
     * @param dirPath 目录路径
     * @return true 成功, false 失败
     */
    bool deleteDirectory(const std::string& dirPath);

    /**
     * @brief 验证版本路径不是当前激活版本
     * @param versionPath 版本路径
     * @param type 类型
     * @return true 安全可删除, false 是激活版本不可删除
     */
    bool isSafeToDelete(const std::string& versionPath, const std::string& type);
};

} // namespace dev_server
