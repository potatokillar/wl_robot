/*
 * @Author: 唐文浩
 * @Date: 2025-11-03
 * @Description: OTA 更新服务实现
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#include "update_service.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <filesystem>
#include <random>
#include <algorithm>

namespace dev_server {

/**
 * @brief 执行系统命令并获取输出
 */
static std::pair<int, std::string> executeCommand(const std::string& command) {
    std::string output;
    char buffer[256];
    
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        return {-1, "Failed to execute command"};
    }
    
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        output += buffer;
    }
    
    int exitCode = pclose(pipe);
    return {WEXITSTATUS(exitCode), output};
}

UpdateService::UpdateService(rclcpp::Node::SharedPtr node)
    : logger_(rclcpp::get_logger("UpdateService")),
      node_(node) {
    // 创建上传目录
    std::filesystem::create_directories(uploadDir_);
    RCLCPP_INFO(logger_, "UpdateService initialized, upload directory: %s", uploadDir_.c_str());
}

UpdateService::~UpdateService() {
    // 停止所有监控线程
    for (auto& [taskId, task] : tasks_) {
        task.monitoring = false;
        if (task.monitorThread.joinable()) {
            task.monitorThread.join();
        }
    }
    RCLCPP_INFO(logger_, "UpdateService destroyed");
}

std::string UpdateService::generateTaskId() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    // 生成随机数（6位十六进制）
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 0xFFFFFF);
    unsigned int random_val = dis(gen);

    std::stringstream ss;
    ss << "update-"
       << std::put_time(std::localtime(&time_t), "%Y%m%d-%H%M%S")
       << "-" << std::hex << std::setw(6) << std::setfill('0') << random_val;
    return ss.str();
}

std::string UpdateService::saveUploadedFile(
    const std::string& taskId,
    const std::vector<uint8_t>& data,
    const std::string& extension
) {
    std::string filePath = uploadDir_ + "/" + taskId + extension;

    try {
        std::ofstream file(filePath, std::ios::binary);
        if (!file.is_open()) {
            RCLCPP_ERROR(logger_, "Failed to create file: %s", filePath.c_str());
            return "";
        }

        file.write(reinterpret_cast<const char*>(data.data()), data.size());
        file.close();

        RCLCPP_INFO(logger_, "Saved uploaded file: %s (size: %zu bytes)",
                    filePath.c_str(), data.size());
        return filePath;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to save file %s: %s", filePath.c_str(), e.what());
        return "";
    }
}

std::string UpdateService::handleUpload(
    const std::vector<uint8_t>& packageData,
    const std::vector<uint8_t>& sha256Data,
    const std::string& appType,
    const std::string& packageFilename
) {
    // 验证文件名是否匹配 appType
    bool filenameValid = false;
    if (appType == "ros2") {
        // ROS2 更新包必须包含 "ros" 或 "ROS"
        if (packageFilename.find("ros") != std::string::npos ||
            packageFilename.find("ROS") != std::string::npos) {
            filenameValid = true;
        }
    } else if (appType == "qr") {
        // QR 更新包必须包含 "qr" 或 "QR"
        if (packageFilename.find("qr") != std::string::npos ||
            packageFilename.find("QR") != std::string::npos) {
            filenameValid = true;
        }
    }

    if (!filenameValid) {
        RCLCPP_ERROR(logger_, "Package filename validation failed: appType=%s, filename=%s",
                     appType.c_str(), packageFilename.c_str());
        RCLCPP_ERROR(logger_, "Requirement: ROS2 packages must contain 'ros' or 'ROS', QR packages must contain 'qr' or 'QR'");
        return "";
    }

    RCLCPP_INFO(logger_, "Package filename validated: appType=%s, filename=%s",
                appType.c_str(), packageFilename.c_str());

    std::string taskId = generateTaskId();

    // 保存更新包文件
    std::string packagePath = saveUploadedFile(taskId, packageData, ".tar.gz");
    if (packagePath.empty()) {
        return "";
    }

    // 保存 SHA256 校验文件
    std::string sha256Path = saveUploadedFile(taskId, sha256Data, ".tar.gz.sha256");
    if (sha256Path.empty()) {
        // 删除已保存的更新包
        std::filesystem::remove(packagePath);
        return "";
    }

    // 记录任务
    {
        std::lock_guard<std::mutex> lock(tasksMutex_);
        // 直接在 map 中构造，避免 atomic 和 thread 的移动问题
        UpdateTask& task = tasks_[taskId];
        task.taskId = taskId;
        task.packagePath = packagePath;
        task.sha256Path = sha256Path;
        task.appType = appType;
        task.originalFilename = packageFilename;  // 保存原始文件名用于验证
    }

    RCLCPP_INFO(logger_, "Upload completed: taskId=%s, appType=%s, package=%zu bytes, sha256=%zu bytes",
                taskId.c_str(), appType.c_str(), packageData.size(), sha256Data.size());
    return taskId;
}

pid_t UpdateService::launchUpdateManager(
    const std::string& taskId,
    const std::string& packagePath,
    const std::string& appType,
    const std::string& originalFilename,
    bool daemon
) {
    // 检查 update_manager 可执行文件是否存在
    if (!std::filesystem::exists(updateManagerPath_)) {
        RCLCPP_ERROR(logger_, "update_manager not found at: %s", updateManagerPath_.c_str());
        return -1;
    }

    // 使用 systemd-run 启动 update_manager，确保进程独立于 iiri-ros.service
    // 这样当停止 iiri-ros.service 时，update_manager 不会被杀死
    std::string unitName = "update-manager-" + taskId;
    std::string logFile = uploadDir_ + "/" + taskId + ".log";

    // 构建 systemd-run 命令（使用 sudo 以获得必要权限）
    std::stringstream cmd;
    cmd << "sudo systemd-run ";  // 添加 sudo
    cmd << "--scope ";
    cmd << "--unit=" << unitName << " ";
    cmd << "--description=\"OTA Update Manager for " << appType << "\" ";
    cmd << "--slice=system.slice ";
    cmd << "bash -c '";
    cmd << updateManagerPath_;
    cmd << " --task-id " << taskId;
    cmd << " --package " << packagePath;
    cmd << " --app-type " << appType;
    cmd << " --original-filename \"" << originalFilename << "\"";  // 传递原始文件名用于验证
    if (daemon) {
        cmd << " --daemon";
    }
    cmd << " > " << logFile << " 2>&1";
    cmd << "'";

    RCLCPP_INFO(logger_, "Launching update_manager with systemd-run: %s", cmd.str().c_str());

    // 执行命令
    auto [exitCode, output] = executeCommand(cmd.str());

    if (exitCode != 0) {
        RCLCPP_ERROR(logger_, "Failed to launch update_manager with systemd-run");
        RCLCPP_ERROR(logger_, "Exit code: %d, Output: %s", exitCode, output.c_str());
        return -1;
    }

    RCLCPP_INFO(logger_, "systemd-run output: %s", output.c_str());

    // 等待一下让 systemd 创建 unit
    sleep(1);

    // 通过 systemctl show 获取 MainPID
    std::string showCmd = "systemctl show " + unitName + ".scope --property=MainPID --value";
    auto [showExitCode, pidStr] = executeCommand(showCmd);

    if (showExitCode != 0 || pidStr.empty() || pidStr == "0") {
        RCLCPP_WARN(logger_, "Failed to get PID from systemd, but process may have started");
        RCLCPP_WARN(logger_, "You can check with: systemctl status %s.scope", unitName.c_str());
        return 0;  // 返回 0 表示启动成功但 PID 未知
    }

    // 移除换行符并转换为整数
    pidStr.erase(std::remove(pidStr.begin(), pidStr.end(), '\n'), pidStr.end());
    pid_t pid = std::stoi(pidStr);

    RCLCPP_INFO(logger_, "Launched update_manager: pid=%d, taskId=%s, unit=%s.scope",
                pid, taskId.c_str(), unitName.c_str());
    return pid;
}

bool UpdateService::startUpdate(const std::string& taskId, bool daemon) {
    UpdateTask* task = nullptr;

    // 查找任务
    {
        std::lock_guard<std::mutex> lock(tasksMutex_);
        auto it = tasks_.find(taskId);
        if (it == tasks_.end()) {
            RCLCPP_ERROR(logger_, "Task not found: %s", taskId.c_str());
            return false;
        }
        task = &it->second;
    }

    // 再次验证文件名与 appType 是否匹配（防止前端篡改）
    bool filenameValid = false;
    if (task->appType == "ros2") {
        if (task->originalFilename.find("ros") != std::string::npos ||
            task->originalFilename.find("ROS") != std::string::npos) {
            filenameValid = true;
        }
    } else if (task->appType == "qr") {
        if (task->originalFilename.find("qr") != std::string::npos ||
            task->originalFilename.find("QR") != std::string::npos) {
            filenameValid = true;
        }
    }

    if (!filenameValid) {
        RCLCPP_ERROR(logger_, "Package filename validation failed at startUpdate: appType=%s, filename=%s",
                     task->appType.c_str(), task->originalFilename.c_str());
        RCLCPP_ERROR(logger_, "Requirement: ROS2 packages must contain 'ros' or 'ROS', QR packages must contain 'qr' or 'QR'");
        return false;
    }

    RCLCPP_INFO(logger_, "StartUpdate validation passed: appType=%s, filename=%s",
                task->appType.c_str(), task->originalFilename.c_str());

    // 清空状态文件，避免监控线程读取到旧数据
    try {
        std::ofstream ofs(task->statusFile, std::ios::trunc);
        if (ofs.is_open()) {
            ofs << "{}";  // 写入空的 JSON 对象
            ofs.close();
            RCLCPP_DEBUG(logger_, "Cleared status file: %s", task->statusFile.c_str());
        } else {
            RCLCPP_WARN(logger_, "Failed to clear status file: %s", task->statusFile.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(logger_, "Exception while clearing status file: %s", e.what());
    }

    // 启动 update_manager 进程
    pid_t pid = launchUpdateManager(
        task->taskId,
        task->packagePath,
        task->appType,
        task->originalFilename,  // 传递原始文件名用于验证
        daemon
    );

    if (pid < 0) {
        RCLCPP_ERROR(logger_, "Failed to launch update_manager for task: %s", taskId.c_str());
        return false;
    }

    // 记录 PID
    task->pid = pid;

    // 启动状态监控线程
    task->monitoring = true;
    task->monitorThread = std::thread([this, taskId]() {
        this->monitorStatusFile(taskId);
    });

    RCLCPP_INFO(logger_, "Update started: taskId=%s, pid=%d", taskId.c_str(), pid);
    return true;
}

void UpdateService::monitorStatusFile(const std::string& taskId) {
    RCLCPP_INFO(logger_, "Starting status monitor for task: %s", taskId.c_str());

    UpdateTask* task = nullptr;
    {
        std::lock_guard<std::mutex> lock(tasksMutex_);
        auto it = tasks_.find(taskId);
        if (it == tasks_.end()) {
            RCLCPP_ERROR(logger_, "Task not found in monitor thread: %s", taskId.c_str());
            return;
        }
        task = &it->second;
    }

    const std::string& statusFile = task->statusFile;
    json lastStatus;

    // 轮询监控状态文件（每秒检查一次）
    while (task->monitoring) {
        if (std::filesystem::exists(statusFile)) {
            json currentStatus = parseStatusFile(statusFile);

            // 检查是否有变化
            if (currentStatus != lastStatus) {
                lastStatus = currentStatus;

                // 通过回调推送状态更新
                if (statusCallback_) {
                    try {
                        statusCallback_(taskId, currentStatus);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(logger_, "Status callback exception: %s", e.what());
                    }
                }

                // 检查是否完成（必须确保 taskId 匹配，避免读取旧数据）
                if (currentStatus.contains("state") && currentStatus.contains("taskId")) {
                    std::string statusTaskId = currentStatus["taskId"];
                    std::string state = currentStatus["state"];

                    // 只有当 taskId 匹配时才判断是否完成
                    if (statusTaskId == taskId) {
                        if (state == "SUCCESS" || state == "FAILED" ||
                            state == "ROLLBACK_SUCCESS" || state == "ROLLBACK_FAILED" ||
                            state == "CANCELLED") {
                            RCLCPP_INFO(logger_, "Update finished: taskId=%s, state=%s",
                                       taskId.c_str(), state.c_str());
                            task->monitoring = false;
                            break;
                        }
                    } else {
                        RCLCPP_DEBUG(logger_, "Ignoring status for different task: expected=%s, got=%s",
                                    taskId.c_str(), statusTaskId.c_str());
                    }
                }
            }
        } else {
            RCLCPP_DEBUG(logger_, "Status file not yet created: %s", statusFile.c_str());
        }

        // 睡眠 1 秒
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(logger_, "Status monitor stopped for task: %s", taskId.c_str());
}

json UpdateService::parseStatusFile(const std::string& filePath) {
    try {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            return json{{"error", "Cannot open status file"}};
        }

        json status;
        file >> status;
        file.close();
        return status;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to parse status file %s: %s",
                     filePath.c_str(), e.what());
        return json{{"error", e.what()}};
    }
}

json UpdateService::getStatus(const std::string& taskId) {
    std::lock_guard<std::mutex> lock(tasksMutex_);
    auto it = tasks_.find(taskId);
    if (it == tasks_.end()) {
        return json{
            {"error", "Task not found"},
            {"taskId", taskId}
        };
    }

    return parseStatusFile(it->second.statusFile);
}

bool UpdateService::cancelUpdate(const std::string& taskId) {
    std::lock_guard<std::mutex> lock(tasksMutex_);
    auto it = tasks_.find(taskId);
    if (it == tasks_.end()) {
        RCLCPP_ERROR(logger_, "Cannot cancel: task not found: %s", taskId.c_str());
        return false;
    }

    UpdateTask& task = it->second;
    if (task.pid > 0) {
        // 发送 SIGTERM 信号到 update_manager 进程
        if (kill(task.pid, SIGTERM) == 0) {
            task.monitoring = false;
            RCLCPP_INFO(logger_, "Cancelled update: taskId=%s, pid=%d",
                       taskId.c_str(), task.pid);
            return true;
        } else {
            RCLCPP_ERROR(logger_, "Failed to kill process %d: %s",
                        task.pid, strerror(errno));
            return false;
        }
    }

    RCLCPP_WARN(logger_, "Cannot cancel: no running process for task: %s", taskId.c_str());
    return false;
}

void UpdateService::setStatusCallback(
    std::function<void(const std::string& taskId, const json& status)> callback
) {
    statusCallback_ = callback;
    RCLCPP_INFO(logger_, "Status callback registered");
}

} // namespace dev_server
