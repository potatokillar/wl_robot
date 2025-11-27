/**
 * @Author: 唐文浩
 * @Date: 2025-11-04
 * @Description: 版本清理处理器实现
 */

#include "version_cleanup.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <regex>

namespace fs = std::filesystem;

namespace dev_server {

// ========================
// 构造函数
// ========================

VersionCleanupHandler::VersionCleanupHandler(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] VersionCleanupHandler initialized");
}

// ========================
// 公共方法
// ========================

CleanupAnalysis VersionCleanupHandler::analyzeCleanup(const std::string& type, int keepCount) {
    CleanupAnalysis analysis;
    analysis.totalVersions = 0;
    analysis.versionsToDelete = 0;
    analysis.spaceToFree = 0;

    RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Analyzing cleanup: type=%s, keepCount=%d",
                type.c_str(), keepCount);

    // 1. 验证类型
    if (type != "ros2" && type != "qr") {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] Invalid type: %s", type.c_str());
        return analysis;
    }

    // 2. 扫描所有版本
    std::string prefix = getDirPrefix(type);
    std::vector<VersionEntry> versions = scanVersions(AUTORUN_DIR, prefix);
    analysis.totalVersions = versions.size();

    if (versions.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[VERSION_CLEANUP] No versions found");
        return analysis;
    }

    // 3. 获取当前激活版本
    std::string symlinkPath = getSymlinkPath(type);
    std::string activeVersion = getSymlinkTarget(symlinkPath);
    analysis.activeVersion = activeVersion;

    // 4. 按构建日期排序（最新的在前）
    std::sort(versions.begin(), versions.end());

    // 5. 选择要删除的版本
    std::vector<VersionEntry> toDelete = selectVersionsToDelete(versions, activeVersion, keepCount);
    analysis.versionsToDelete = toDelete.size();

    // 6. 计算释放空间并生成列表
    for (const auto& entry : toDelete) {
        analysis.versionList.push_back(entry.dirName);
        analysis.spaceToFree += entry.size;
    }

    RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Analysis: %d versions total, %d to delete, %.2f MB to free",
                analysis.totalVersions, analysis.versionsToDelete,
                analysis.spaceToFree / (1024.0 * 1024.0));

    return analysis;
}

CleanupResult VersionCleanupHandler::cleanupOldVersions(const std::string& type, int keepCount) {
    CleanupResult result;
    result.success = false;
    result.freedSpace = 0;

    RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Starting cleanup: type=%s, keepCount=%d",
                type.c_str(), keepCount);

    // 1. 验证类型
    if (type != "ros2" && type != "qr") {
        result.message = "Invalid type. Must be 'ros2' or 'qr'";
        result.errorDetails = "Received type: " + type;
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] %s", result.message.c_str());
        return result;
    }

    // 2. 扫描所有版本
    std::string prefix = getDirPrefix(type);
    std::vector<VersionEntry> versions = scanVersions(AUTORUN_DIR, prefix);

    if (versions.empty()) {
        result.success = true;
        result.message = "No versions found to clean up";
        RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] No versions found");
        return result;
    }

    RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Found %zu versions", versions.size());

    // 3. 获取当前激活版本
    std::string symlinkPath = getSymlinkPath(type);
    std::string activeVersion = getSymlinkTarget(symlinkPath);
    RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Active version: %s", activeVersion.c_str());

    // 4. 按构建日期排序（最新的在前）
    std::sort(versions.begin(), versions.end());

    // 5. 记录保留的版本
    int keepIndex = 0;
    for (const auto& entry : versions) {
        if (entry.fullPath == activeVersion || keepIndex < keepCount) {
            result.remainingVersions.push_back(entry.dirName);
            keepIndex++;
        }
    }

    // 6. 选择要删除的版本
    std::vector<VersionEntry> toDelete = selectVersionsToDelete(versions, activeVersion, keepCount);

    if (toDelete.empty()) {
        result.success = true;
        result.message = "No old versions to delete";
        RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] No old versions to delete");
        return result;
    }

    RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Will delete %zu versions", toDelete.size());

    // 7. 执行删除
    int deleteCount = 0;
    int failCount = 0;

    for (const auto& entry : toDelete) {
        RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Deleting: %s (%.2f MB)",
                   entry.dirName.c_str(), entry.size / (1024.0 * 1024.0));

        if (deleteDirectory(entry.fullPath)) {
            result.deletedVersions.push_back(entry.dirName);
            result.freedSpace += entry.size;
            deleteCount++;
            RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Deleted: %s", entry.dirName.c_str());
        } else {
            failCount++;
            RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] Failed to delete: %s", entry.dirName.c_str());
        }
    }

    // 8. 汇总结果
    if (failCount == 0) {
        result.success = true;
        result.message = "Cleanup completed successfully";
    } else {
        result.success = (deleteCount > 0);
        result.message = "Cleanup completed with some failures";
        result.errorDetails = std::to_string(failCount) + " versions failed to delete";
    }

    RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Cleanup completed: deleted=%d, failed=%d, freed=%.2f MB",
                deleteCount, failCount, result.freedSpace / (1024.0 * 1024.0));

    return result;
}

CleanupResult VersionCleanupHandler::deleteVersion(const std::string& type, const std::string& versionPath) {
    CleanupResult result;
    result.success = false;

    RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Deleting single version: %s", versionPath.c_str());

    // 1. 验证类型
    if (type != "ros2" && type != "qr") {
        result.message = "Invalid type. Must be 'ros2' or 'qr'";
        result.errorDetails = "Received type: " + type;
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] %s", result.message.c_str());
        return result;
    }

    // 2. 验证路径存在
    if (!fs::exists(versionPath) || !fs::is_directory(versionPath)) {
        result.message = "Version path does not exist or is not a directory";
        result.errorDetails = "Path: " + versionPath;
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] %s", result.message.c_str());
        return result;
    }

    // 3. 安全检查：不删除激活版本
    if (!isSafeToDelete(versionPath, type)) {
        result.message = "Cannot delete active version";
        result.errorDetails = "Version is currently active: " + versionPath;
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] %s", result.message.c_str());
        return result;
    }

    // 4. 计算大小
    uint64_t size = calculateDirSize(versionPath);

    // 5. 执行删除
    if (deleteDirectory(versionPath)) {
        result.success = true;
        result.message = "Version deleted successfully";
        result.deletedVersions.push_back(fs::path(versionPath).filename().string());
        result.freedSpace = size;
        RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Version deleted: %s (%.2f MB)",
                   versionPath.c_str(), size / (1024.0 * 1024.0));
    } else {
        result.message = "Failed to delete version";
        result.errorDetails = "Path: " + versionPath;
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] Failed to delete: %s", versionPath.c_str());
    }

    return result;
}

std::string VersionCleanupHandler::getJsonResponse(const CleanupResult& result) {
    return result.toJson().dump();
}

std::string VersionCleanupHandler::getAnalysisJson(const CleanupAnalysis& analysis) {
    return analysis.toJson().dump();
}

// ========================
// 私有方法
// ========================

std::string VersionCleanupHandler::getDirPrefix(const std::string& type) const {
    return (type == "ros2") ? ROS2_PREFIX : QR_PREFIX;
}

std::string VersionCleanupHandler::getSymlinkPath(const std::string& type) const {
    return (type == "ros2") ? ROS2_SYMLINK : QR_SYMLINK;
}

std::vector<VersionEntry> VersionCleanupHandler::scanVersions(const std::string& baseDir,
                                                                const std::string& prefix) {
    std::vector<VersionEntry> versions;

    try {
        for (const auto& entry : fs::directory_iterator(baseDir)) {
            if (!entry.is_directory()) {
                continue;
            }

            std::string dirName = entry.path().filename().string();

            // 检查目录名前缀
            if (dirName.find(prefix) != 0) {
                continue;
            }

            VersionEntry versionEntry;
            versionEntry.dirName = dirName;
            versionEntry.fullPath = entry.path().string();
            versionEntry.buildDate = getBuildDate(versionEntry.fullPath);
            versionEntry.size = calculateDirSize(versionEntry.fullPath);
            versionEntry.isActive = false;  // 稍后设置

            versions.push_back(versionEntry);

            RCLCPP_DEBUG(node_->get_logger(), "[VERSION_CLEANUP] Found version: %s (date=%s, size=%.2f MB)",
                        dirName.c_str(), versionEntry.buildDate.c_str(),
                        versionEntry.size / (1024.0 * 1024.0));
        }
    } catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] Failed to scan directory: %s", e.what());
    }

    return versions;
}

std::string VersionCleanupHandler::getBuildDate(const std::string& versionPath) {
    std::string versionFile = versionPath + "/VERSION.txt";

    if (!fs::exists(versionFile)) {
        // 如果 VERSION.txt 不存在，使用目录修改时间
        try {
            auto ftime = fs::last_write_time(versionPath);
            auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                ftime - fs::file_time_type::clock::now() + std::chrono::system_clock::now()
            );
            auto time_t_val = std::chrono::system_clock::to_time_t(sctp);
            std::tm tm_val = *std::localtime(&time_t_val);

            char buffer[32];
            std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_val);
            return std::string(buffer);
        } catch (const std::exception& e) {
            return "1970-01-01 00:00:00";  // 默认最早日期
        }
    }

    std::ifstream file(versionFile);
    if (!file.is_open()) {
        return "1970-01-01 00:00:00";
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.find("BUILD_DATE:") == 0) {
            size_t colonPos = line.find(':');
            if (colonPos != std::string::npos) {
                std::string date = line.substr(colonPos + 1);
                // 去除前后空格
                date.erase(0, date.find_first_not_of(" \t\r\n"));
                date.erase(date.find_last_not_of(" \t\r\n") + 1);
                return date;
            }
        }
    }

    return "1970-01-01 00:00:00";
}

uint64_t VersionCleanupHandler::calculateDirSize(const std::string& dirPath) {
    uint64_t totalSize = 0;

    try {
        for (const auto& entry : fs::recursive_directory_iterator(dirPath)) {
            if (entry.is_regular_file()) {
                totalSize += entry.file_size();
            }
        }
    } catch (const fs::filesystem_error& e) {
        RCLCPP_WARN(node_->get_logger(), "[VERSION_CLEANUP] Failed to calculate size for %s: %s",
                   dirPath.c_str(), e.what());
    }

    return totalSize;
}

std::string VersionCleanupHandler::getSymlinkTarget(const std::string& symlinkPath) {
    try {
        if (!fs::is_symlink(symlinkPath)) {
            RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] Path is not a symlink: %s",
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
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] Failed to read symlink: %s", e.what());
        return "";
    }
}

std::vector<VersionEntry> VersionCleanupHandler::selectVersionsToDelete(
    const std::vector<VersionEntry>& versions,
    const std::string& activeVersion,
    int keepCount) {

    std::vector<VersionEntry> toDelete;
    int keptCount = 0;

    for (const auto& entry : versions) {
        // 永远不删除激活版本
        if (entry.fullPath == activeVersion) {
            RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Keeping active version: %s",
                       entry.dirName.c_str());
            continue;
        }

        // 保留最近的 N 个版本
        if (keptCount < keepCount) {
            RCLCPP_INFO(node_->get_logger(), "[VERSION_CLEANUP] Keeping recent version: %s",
                       entry.dirName.c_str());
            keptCount++;
            continue;
        }

        // 其余的加入删除列表
        toDelete.push_back(entry);
    }

    return toDelete;
}

bool VersionCleanupHandler::deleteDirectory(const std::string& dirPath) {
    try {
        // 使用系统命令删除（更可靠）
        // 使用 sudo 以解决 root 用户创建的日志文件权限问题
        std::string command = "sudo rm -rf " + dirPath + " 2>&1";
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            return false;
        }

        int exitCode = pclose(pipe);
        return (exitCode == 0);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] Failed to delete directory: %s", e.what());
        return false;
    }
}

bool VersionCleanupHandler::isSafeToDelete(const std::string& versionPath, const std::string& type) {
    std::string symlinkPath = getSymlinkPath(type);
    std::string activeVersion = getSymlinkTarget(symlinkPath);

    // 规范化路径进行比较
    try {
        fs::path versionPathCanonical = fs::canonical(versionPath);
        fs::path activeVersionCanonical = fs::canonical(activeVersion);

        if (versionPathCanonical == activeVersionCanonical) {
            RCLCPP_WARN(node_->get_logger(), "[VERSION_CLEANUP] Cannot delete active version: %s",
                       versionPath.c_str());
            return false;
        }
    } catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(node_->get_logger(), "[VERSION_CLEANUP] Failed to check path safety: %s", e.what());
        return false;
    }

    return true;
}

} // namespace dev_server
