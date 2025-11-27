/*
 * @Author: 唐文浩
 * @Date: 2025-11-03
 * @Description: OTA 更新服务 - 负责管理 update_manager 进程和状态监控
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#pragma once

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <mutex>
#include <functional>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

namespace dev_server {

/**
 * @brief 更新任务信息
 */
struct UpdateTask {
    std::string taskId;              // 任务ID
    std::string packagePath;         // 更新包路径
    std::string sha256Path;          // SHA256 校验文件路径
    std::string appType;             // 应用类型（ros2/qr）
    std::string originalFilename;    // 原始文件名（用于验证）
    pid_t pid = -1;                  // update_manager 进程ID
    std::string statusFile = "/var/run/update_status.json";  // 状态文件路径
    std::atomic<bool> monitoring{false};  // 是否正在监控
    std::thread monitorThread;       // 监控线程
};

/**
 * @brief OTA 更新服务类
 *
 * 功能：
 * 1. 处理文件上传（更新包 + SHA256 校验文件）
 * 2. 启动 update_manager 进程（fork + execv）
 * 3. 监控状态文件变化并通过 WebSocket 推送
 * 4. 取消更新任务
 */
class UpdateService {
public:
    /**
     * @brief 构造函数
     * @param node ROS2 节点指针
     */
    explicit UpdateService(rclcpp::Node::SharedPtr node);

    /**
     * @brief 析构函数 - 确保所有线程正常退出
     */
    ~UpdateService();

    /**
     * @brief 处理文件上传
     * @param packageData 更新包二进制数据
     * @param sha256Data SHA256 校验文件数据
     * @param appType 应用类型（ros2 或 qr）
     * @param packageFilename 更新包文件名（用于验证）
     * @return 任务ID，如果验证失败则返回空字符串
     */
    std::string handleUpload(
        const std::vector<uint8_t>& packageData,
        const std::vector<uint8_t>& sha256Data,
        const std::string& appType,
        const std::string& packageFilename
    );

    /**
     * @brief 启动更新进程
     * @param taskId 任务ID
     * @param daemon 是否以守护进程模式运行
     * @return 成功返回 true
     */
    bool startUpdate(const std::string& taskId, bool daemon = true);

    /**
     * @brief 查询更新状态
     * @param taskId 任务ID
     * @return 状态 JSON 对象
     */
    json getStatus(const std::string& taskId);

    /**
     * @brief 取消更新任务
     * @param taskId 任务ID
     * @return 成功返回 true
     */
    bool cancelUpdate(const std::string& taskId);

    /**
     * @brief 设置状态变化回调函数（用于 WebSocket 推送）
     * @param callback 回调函数，参数为 (taskId, statusJson)
     */
    void setStatusCallback(
        std::function<void(const std::string& taskId, const json& status)> callback
    );

private:
    /**
     * @brief 生成唯一任务ID
     * @return 格式: update-YYYYMMDD-HHMMSS-随机值
     */
    std::string generateTaskId();

    /**
     * @brief 保存上传的文件
     * @param taskId 任务ID
     * @param data 文件数据
     * @param extension 文件扩展名（如 .tar.gz）
     * @return 保存后的文件路径
     */
    std::string saveUploadedFile(
        const std::string& taskId,
        const std::vector<uint8_t>& data,
        const std::string& extension
    );

    /**
     * @brief 启动 update_manager 进程（使用 systemd-run）
     * @param taskId 任务ID
     * @param packagePath 更新包路径
     * @param appType 应用类型
     * @param originalFilename 原始文件名（用于验证）
     * @param daemon 是否守护进程模式
     * @return 进程ID，失败返回 -1
     */
    pid_t launchUpdateManager(
        const std::string& taskId,
        const std::string& packagePath,
        const std::string& appType,
        const std::string& originalFilename,
        bool daemon
    );

    /**
     * @brief 监控状态文件变化（独立线程中运行）
     * @param taskId 任务ID
     */
    void monitorStatusFile(const std::string& taskId);

    /**
     * @brief 解析状态文件
     * @param filePath 状态文件路径
     * @return JSON 对象
     */
    json parseStatusFile(const std::string& filePath);

    // === 成员变量 ===

    /// 任务映射表（taskId -> UpdateTask）
    std::unordered_map<std::string, UpdateTask> tasks_;

    /// 任务表互斥锁
    std::mutex tasksMutex_;

    /// 状态变化回调函数（WebSocket 推送）
    std::function<void(const std::string& taskId, const json& status)> statusCallback_;

    /// ROS2 日志器
    rclcpp::Logger logger_;

    /// ROS2 节点指针
    rclcpp::Node::SharedPtr node_;

    // === 配置项 ===

    /// update_manager 可执行文件路径
    std::string updateManagerPath_ = "/home/wl/autorun/update_manager/update_manager";

    /// 上传文件存储目录
    std::string uploadDir_ = "/tmp/updates";
};

} // namespace dev_server
