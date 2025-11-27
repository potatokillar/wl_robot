/**
 * @Author: 唐文浩
 * @Date: 2025-11-04
 * @Description: 版本切换处理器实现
 */

#include "version_switch.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <ctime>
#include <chrono>
#include <thread>
#include <array>
#include <memory>
#include <cstdio>

namespace fs = std::filesystem;

namespace dev_server {

// ========================
// 构造函数
// ========================

VersionSwitchHandler::VersionSwitchHandler(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] VersionSwitchHandler initialized");
}

// ========================
// 公共方法
// ========================

SwitchResult VersionSwitchHandler::switchVersion(const std::string& type, const std::string& targetVersion) {
    SwitchResult result;
    result.success = false;

    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Starting version switch: type=%s, target=%s",
                type.c_str(), targetVersion.c_str());

    // 1. 验证类型
    if (type != "ros2" && type != "qr") {
        result.message = "Invalid type. Must be 'ros2' or 'qr'";
        result.errorDetails = "Received type: " + type;
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] %s", result.message.c_str());
        return result;
    }

    // 2. 验证目标版本路径是否存在
    if (!validateTargetPath(targetVersion)) {
        result.message = "Target version path does not exist";
        result.errorDetails = "Path: " + targetVersion;
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] %s", result.message.c_str());
        return result;
    }

    // 3. 获取当前版本
    std::string symlinkPath = getSymlinkPath(type);
    std::string currentVersion = getSymlinkTarget(symlinkPath);
    result.previousVersion = currentVersion;

    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Current version: %s", currentVersion.c_str());

    // 4. 检查是否已经是目标版本
    if (currentVersion == targetVersion) {
        result.success = true;
        result.message = "Already on target version";
        result.currentVersion = targetVersion;
        RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Already on target version");
        return result;
    }

    // 5. 备份当前配置
    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Backing up current configuration...");
    std::string backupPath = backupCurrentConfig(type, currentVersion);
    if (backupPath.empty()) {
        result.message = "Failed to backup current configuration";
        result.errorDetails = "Check permissions and disk space";
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Backup failed");
        return result;
    }
    result.backupPath = backupPath;
    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Backup created: %s", backupPath.c_str());

    // 6. 更新符号链接
    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Updating symlink...");
    if (!updateSymlink(symlinkPath, targetVersion)) {
        result.message = "Failed to update symlink";
        result.errorDetails = "Symlink path: " + symlinkPath;
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Symlink update failed");
        return result;
    }
    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Symlink updated successfully");

    // 7. 重启服务
    std::string serviceName = getServiceName(type);
    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Restarting service: %s", serviceName.c_str());
    if (!restartService(serviceName)) {
        result.message = "Failed to restart service";
        result.errorDetails = "Service: " + serviceName;
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Service restart failed");

        // 尝试回滚
        RCLCPP_WARN(node_->get_logger(), "[VERSION_SWITCH] Attempting rollback...");
        if (rollback(type, currentVersion)) {
            result.message += " (rolled back to previous version)";
            RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Rollback successful");
        } else {
            result.message += " (rollback also failed!)";
            RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Rollback failed!");
        }
        return result;
    }

    // 8. 健康检查
    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Performing health check (timeout: %d seconds)...",
                HEALTH_CHECK_TIMEOUT);
    if (!checkServiceHealth(serviceName, HEALTH_CHECK_TIMEOUT)) {
        result.message = "Service health check failed";
        result.errorDetails = "Service did not start properly within timeout";
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Health check failed");

        // 尝试回滚
        RCLCPP_WARN(node_->get_logger(), "[VERSION_SWITCH] Attempting rollback...");
        if (rollback(type, currentVersion)) {
            result.message += " (rolled back to previous version)";
            RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Rollback successful");
        } else {
            result.message += " (rollback also failed!)";
            RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Rollback failed!");
        }
        return result;
    }

    // 9. 成功
    result.success = true;
    result.message = "Version switched successfully";
    result.currentVersion = targetVersion;

    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Version switch completed successfully");
    return result;
}

std::string VersionSwitchHandler::getJsonResponse(const SwitchResult& result) {
    return result.toJson().dump();
}

// ========================
// 私有方法
// ========================

std::string VersionSwitchHandler::getSymlinkPath(const std::string& type) const {
    return (type == "ros2") ? ROS2_SYMLINK : QR_SYMLINK;
}

std::string VersionSwitchHandler::getServiceName(const std::string& type) const {
    return (type == "ros2") ? ROS2_SERVICE : QR_SERVICE;
}

std::string VersionSwitchHandler::getSymlinkTarget(const std::string& symlinkPath) {
    try {
        if (!fs::is_symlink(symlinkPath)) {
            RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Path is not a symlink: %s",
                        symlinkPath.c_str());
            return "";
        }

        fs::path target = fs::read_symlink(symlinkPath);

        // 转换为绝对路径
        if (target.is_relative()) {
            fs::path symlinkDir = fs::path(symlinkPath).parent_path();
            target = fs::canonical(symlinkDir / target);
        }

        return target.string();
    } catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Failed to read symlink: %s", e.what());
        return "";
    }
}

bool VersionSwitchHandler::validateTargetPath(const std::string& targetPath) {
    try {
        return fs::exists(targetPath) && fs::is_directory(targetPath);
    } catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Failed to validate path: %s", e.what());
        return false;
    }
}

std::string VersionSwitchHandler::createBackupDirectory(const std::string& type) {
    // 生成时间戳：YYYYMMDD-HHMMSS
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::localtime(&time_t_now);

    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d-%H%M%S", &tm_now);

    // 备份路径：/var/backups/iiri/{type}-{timestamp}/
    std::string backupDir = BACKUP_BASE_DIR + "/" + type + "-" + timestamp;

    try {
        // 创建备份目录（递归创建）
        fs::create_directories(backupDir);
        RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Created backup directory: %s",
                   backupDir.c_str());
        return backupDir;
    } catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Failed to create backup directory: %s",
                    e.what());
        return "";
    }
}

bool VersionSwitchHandler::copyConfigFiles(const std::string& sourceDir, const std::string& backupDir) {
    std::string command = "rsync -av --include='*.yaml' --include='*.json' --include='*.conf' "
                          "--include='*/' --exclude='*' " + sourceDir + "/ " + backupDir + "/ 2>&1";

    std::string output;
    int exitCode = executeCommand(command, output);

    if (exitCode != 0 && exitCode != 23) {  // rsync exit code 23 is partial transfer (acceptable)
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Failed to copy config files: %s",
                    output.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Config files copied successfully");
    return true;
}

std::string VersionSwitchHandler::backupCurrentConfig(const std::string& type,
                                                       const std::string& currentVersion) {
    // 创建备份目录
    std::string backupDir = createBackupDirectory(type);
    if (backupDir.empty()) {
        return "";
    }

    // 复制配置文件
    if (!copyConfigFiles(currentVersion, backupDir)) {
        return "";
    }

    // 保存元数据
    json metadata;
    metadata["type"] = type;
    metadata["sourceVersion"] = currentVersion;
    metadata["timestamp"] = std::time(nullptr);

    std::string metadataFile = backupDir + "/backup_metadata.json";
    std::ofstream ofs(metadataFile);
    if (ofs.is_open()) {
        ofs << metadata.dump(2);
        ofs.close();
    }

    return backupDir;
}

bool VersionSwitchHandler::updateSymlink(const std::string& symlinkPath, const std::string& targetPath) {
    // 使用 ln -snf 实现原子更新
    std::string command = "ln -snf " + targetPath + " " + symlinkPath + " 2>&1";
    std::string output;
    int exitCode = executeCommand(command, output);

    if (exitCode != 0) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Failed to update symlink: %s",
                    output.c_str());
        return false;
    }

    return true;
}

bool VersionSwitchHandler::restartService(const std::string& serviceName) {
    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Scheduling service restart via 'at' command");
    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Service will restart in 1 minute");

    // 使用 at 命令延迟执行重启，完全脱离当前进程控制
    // 这避免了"自己重启自己"的循环等待问题
    // 注意：at 命令不支持 seconds，只支持 minutes/hours/days
    std::stringstream cmd;
    cmd << "echo 'systemctl restart " << serviceName << "' | ";
    cmd << "sudo at now + 1 minute 2>&1";

    std::string output;
    int exitCode = executeCommand(cmd.str(), output);

    if (exitCode != 0) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Failed to schedule restart: %s",
                    output.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Service restart scheduled successfully");
    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] at command output: %s", output.c_str());

    // 不需要等待，API 响应会在服务重启前发送给客户端
    // 让 at 守护进程在后台执行重启，完全独立于当前服务
    return true;
}

bool VersionSwitchHandler::checkServiceHealth(const std::string& serviceName, int timeoutSeconds) {
    int elapsed = 0;
    const int checkInterval = 2;  // 每 2 秒检查一次

    while (elapsed < timeoutSeconds) {
        std::string command = "systemctl is-active " + serviceName + " 2>&1";
        std::string output;
        int exitCode = executeCommand(command, output);

        // 去除输出末尾的换行符
        if (!output.empty() && output.back() == '\n') {
            output.pop_back();
        }

        if (exitCode == 0 && output == "active") {
            RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Service is active: %s",
                       serviceName.c_str());
            return true;
        }

        RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Waiting for service to become active... (%d/%d seconds)",
                   elapsed, timeoutSeconds);

        std::this_thread::sleep_for(std::chrono::seconds(checkInterval));
        elapsed += checkInterval;
    }

    RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Service health check timeout: %s",
                serviceName.c_str());
    return false;
}

bool VersionSwitchHandler::rollback(const std::string& type, const std::string& previousVersion) {
    RCLCPP_WARN(node_->get_logger(), "[VERSION_SWITCH] Rolling back to: %s", previousVersion.c_str());

    std::string symlinkPath = getSymlinkPath(type);
    std::string serviceName = getServiceName(type);

    // 1. 恢复符号链接
    if (!updateSymlink(symlinkPath, previousVersion)) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Rollback: Failed to restore symlink");
        return false;
    }

    // 2. 重启服务
    if (!restartService(serviceName)) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Rollback: Failed to restart service");
        return false;
    }

    // 3. 健康检查
    if (!checkServiceHealth(serviceName, HEALTH_CHECK_TIMEOUT)) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Rollback: Health check failed");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "[VERSION_SWITCH] Rollback completed successfully");
    return true;
}

int VersionSwitchHandler::executeCommand(const std::string& command, std::string& output) {
    std::array<char, 128> buffer;
    std::string result;

    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_SWITCH] Failed to execute command: %s",
                    command.c_str());
        return -1;
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }

    output = result;
    return pclose(pipe.release());
}

} // namespace dev_server
