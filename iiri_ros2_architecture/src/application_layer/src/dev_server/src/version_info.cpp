#include "version_info.hpp"
#include <fstream>
#include <filesystem>
#include <sstream>
#include <algorithm>
#include <regex>

namespace fs = std::filesystem;

namespace dev_server {

VersionInfoHandler::VersionInfoHandler(rclcpp::Node::SharedPtr node)
    : node_(node) {
    RCLCPP_INFO(node_->get_logger(), "VersionInfoHandler initialized");
}

std::string VersionInfoHandler::getROS2VersionJson() {
    try {
        // 获取当前激活版本的符号链接目标
        std::string symlinkPath = "/home/wl/autorun/iiri-ros";

        if (!fs::exists(symlinkPath)) {
            RCLCPP_WARN(node_->get_logger(), "Symlink %s does not exist", symlinkPath.c_str());
            return R"({"error": "ROS2 installation not found"})";
        }

        std::string currentPath;
        if (fs::is_symlink(symlinkPath)) {
            currentPath = getSymlinkTarget(symlinkPath);
        } else {
            // 如果不是符号链接，直接使用该路径
            currentPath = symlinkPath;
        }

        std::string versionFile = currentPath + "/VERSION.txt";

        if (!fs::exists(versionFile)) {
            RCLCPP_WARN(node_->get_logger(), "VERSION.txt not found at %s", versionFile.c_str());
            // 尝试从目录名提取版本信息
            json versionData = extractVersionFromDirname(currentPath);
            versionData["isActive"] = true;
            versionData["deployPath"] = currentPath;
            versionData["source"] = "dirname";
            return versionData.dump();
        }

        json versionData = parseVersionFile(versionFile);
        versionData["isActive"] = true;
        versionData["deployPath"] = currentPath;
        versionData["source"] = "VERSION.txt";

        return versionData.dump();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get ROS2 version: %s", e.what());
        return R"({"error": "Failed to read ROS2 version info"})";
    }
}

std::string VersionInfoHandler::getQRVersionJson() {
    try {
        std::string qrPath = "/home/wl/autorun/iiri-qr";

        if (!fs::exists(qrPath)) {
            RCLCPP_DEBUG(node_->get_logger(), "QR directory not found at %s", qrPath.c_str());
            return R"({"version": "unknown", "note": "QR installation not found"})";
        }

        // 如果是符号链接，获取目标路径
        std::string currentPath;
        if (fs::is_symlink(qrPath)) {
            currentPath = getSymlinkTarget(qrPath);
        } else {
            currentPath = qrPath;
        }

        std::string versionFile = currentPath + "/VERSION.txt";

        if (!fs::exists(versionFile)) {
            RCLCPP_DEBUG(node_->get_logger(), "QR VERSION.txt not found");
            // 尝试从目录名提取版本信息
            json versionData = extractVersionFromDirname(currentPath);
            versionData["isActive"] = true;
            versionData["deployPath"] = currentPath;
            versionData["source"] = "dirname";
            return versionData.dump();
        }

        json versionData = parseVersionFile(versionFile);
        versionData["isActive"] = true;
        versionData["deployPath"] = currentPath;
        versionData["source"] = "VERSION.txt";

        return versionData.dump();
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to get QR version: %s", e.what());
        return R"({"version": "unknown", "note": "Failed to read QR version info"})";
    }
}

std::string VersionInfoHandler::getVersionHistoryJson() {
    try {
        json result;

        // 获取当前符号链接目标
        std::string symlinkPath = "/home/wl/autorun/iiri-ros";
        std::string currentTarget = "";

        if (fs::exists(symlinkPath) && fs::is_symlink(symlinkPath)) {
            try {
                currentTarget = getSymlinkTarget(symlinkPath);
            } catch (...) {
                currentTarget = "";
            }
        }

        result["currentSymlink"] = currentTarget;

        // 扫描 ROS2 版本目录
        auto ros2VersionDirs = scanVersionDirs("/home/wl/autorun", "iiri-ros-");
        json ros2Versions = json::array();

        for (const auto& dir : ros2VersionDirs) {
            std::string versionFile = dir + "/VERSION.txt";
            json versionData;

            if (fs::exists(versionFile)) {
                versionData = parseVersionFile(versionFile);
            } else {
                versionData = extractVersionFromDirname(dir);
            }

            versionData["isActive"] = (dir == currentTarget);
            versionData["deployPath"] = dir;
            versionData["type"] = "ros2";
            ros2Versions.push_back(versionData);
        }

        // 扫描 QR 版本目录
        auto qrVersionDirs = scanVersionDirs("/home/wl/autorun", "iiri-qr-");
        json qrVersions = json::array();

        for (const auto& dir : qrVersionDirs) {
            std::string versionFile = dir + "/VERSION.txt";
            json versionData;

            if (fs::exists(versionFile)) {
                versionData = parseVersionFile(versionFile);
            } else {
                versionData = extractVersionFromDirname(dir);
            }

            // 检查 QR 符号链接
            std::string qrSymlink = "/home/wl/autorun/iiri-qr";
            if (fs::exists(qrSymlink) && fs::is_symlink(qrSymlink)) {
                try {
                    versionData["isActive"] = (dir == getSymlinkTarget(qrSymlink));
                } catch (...) {
                    versionData["isActive"] = false;
                }
            } else {
                versionData["isActive"] = false;
            }

            versionData["deployPath"] = dir;
            versionData["type"] = "qr";
            qrVersions.push_back(versionData);
        }

        result["ros2Versions"] = ros2Versions;
        result["qrVersions"] = qrVersions;
        result["totalCount"] = ros2Versions.size() + qrVersions.size();

        return result.dump();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get version history: %s", e.what());
        return R"({"error": "Failed to read version history"})";
    }
}

std::string VersionInfoHandler::getCurrentVersionJson() {
    try {
        json result;

        // ROS2 当前版本
        std::string ros2Symlink = "/home/wl/autorun/iiri-ros";
        if (fs::exists(ros2Symlink) && fs::is_symlink(ros2Symlink)) {
            result["ros2"]["symlinkPath"] = ros2Symlink;
            result["ros2"]["targetPath"] = getSymlinkTarget(ros2Symlink);
        } else {
            result["ros2"]["symlinkPath"] = ros2Symlink;
            result["ros2"]["targetPath"] = "not a symlink";
        }

        // QR 当前版本
        std::string qrSymlink = "/home/wl/autorun/iiri-qr";
        if (fs::exists(qrSymlink) && fs::is_symlink(qrSymlink)) {
            result["qr"]["symlinkPath"] = qrSymlink;
            result["qr"]["targetPath"] = getSymlinkTarget(qrSymlink);
        } else {
            result["qr"]["symlinkPath"] = qrSymlink;
            result["qr"]["targetPath"] = "not found or not a symlink";
        }

        return result.dump();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to get current version: %s", e.what());
        return R"({"error": "Failed to read current version"})";
    }
}

json VersionInfoHandler::parseVersionFile(const std::string& filepath) {
    json result;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        throw std::runtime_error("Cannot open " + filepath);
    }

    std::string line;
    while (std::getline(file, line)) {
        // 跳过空行和分隔线
        if (line.empty() || line.find("====") != std::string::npos) {
            continue;
        }

        // 解析格式: "Key: Value"
        size_t colonPos = line.find(':');
        if (colonPos != std::string::npos) {
            std::string key = line.substr(0, colonPos);
            std::string value = line.substr(colonPos + 1);

            // 去除前后空格
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);

            // 转换为 camelCase 并存储
            if (key == "Architecture") {
                result["architecture"] = value;
            } else if (key == "Version") {
                result["version"] = value;
            } else if (key == "Build Date") {
                result["buildDate"] = value;
            } else if (key == "Git Branch") {
                result["gitBranch"] = value;
            } else if (key == "Git Commit") {
                result["gitCommit"] = value;
            } else if (key == "Build Host") {
                result["buildHost"] = value;
            }
        }
    }

    file.close();

    // 如果没有解析到任何字段，返回错误
    if (result.empty()) {
        result["error"] = "Failed to parse VERSION.txt";
        result["version"] = "unknown";
    }

    return result;
}

std::string VersionInfoHandler::getSymlinkTarget(const std::string& symlinkPath) {
    if (!fs::is_symlink(symlinkPath)) {
        throw std::runtime_error(symlinkPath + " is not a symlink");
    }

    // 读取符号链接
    fs::path target = fs::read_symlink(symlinkPath);

    // 如果是相对路径，转换为绝对路径
    if (target.is_relative()) {
        fs::path symlinkDir = fs::path(symlinkPath).parent_path();
        target = fs::canonical(symlinkDir / target);
    }

    return target.string();
}

std::vector<std::string> VersionInfoHandler::scanVersionDirs(
    const std::string& baseDir, const std::string& prefix) {
    std::vector<std::string> versionDirs;

    if (!fs::exists(baseDir) || !fs::is_directory(baseDir)) {
        RCLCPP_WARN(node_->get_logger(), "Base directory %s does not exist", baseDir.c_str());
        return versionDirs;
    }

    try {
        for (const auto& entry : fs::directory_iterator(baseDir)) {
            if (entry.is_directory()) {
                std::string dirname = entry.path().filename().string();
                // 匹配前缀（如 "iiri-ros-" 或 "iiri-qr-"）
                if (dirname.find(prefix) == 0) {
                    versionDirs.push_back(entry.path().string());
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "Failed to scan directory %s: %s",
                    baseDir.c_str(), e.what());
    }

    // 按名称排序（最新的版本通常在后面）
    std::sort(versionDirs.begin(), versionDirs.end(), std::greater<>());

    return versionDirs;
}

json VersionInfoHandler::extractVersionFromDirname(const std::string& dirname) {
    json result;

    // 从完整路径提取目录名
    fs::path p(dirname);
    std::string name = p.filename().string();

    // 尝试匹配格式: iiri-ros-{arch}-{version}
    // 或: iiri-qr-{arch}-{version}
    std::regex pattern(R"((iiri-ros|iiri-qr)-(\w+)-([\w\-\.]+))");
    std::smatch matches;

    if (std::regex_search(name, matches, pattern)) {
        result["architecture"] = matches[2].str();  // arm / x86
        result["version"] = matches[3].str();       // 版本号
        result["source"] = "dirname";
    } else {
        result["version"] = name;  // 如果无法解析，使用整个目录名
        result["architecture"] = "unknown";
        result["source"] = "dirname";
    }

    return result;
}

} // namespace dev_server
