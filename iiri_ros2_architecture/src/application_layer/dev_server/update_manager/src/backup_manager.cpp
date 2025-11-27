/**
 * @file backup_manager.cpp
 * @brief OTA Update Manager - Backup Management Implementation
 * @author 唐文浩
 * @date 2025-11-05
 *
 * 修复内容：
 * - 使用绝对路径创建符号链接（修复 "Backup directory does not exist after backup" 错误）
 * - 添加详细的日志输出
 * - 验证备份链接的有效性
 */

#include "backup_manager.hpp"
#include <filesystem>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iostream>

namespace fs = std::filesystem;

namespace iiri {
namespace ota {

BackupManager::BackupManager(StatusManager* status_mgr, const std::string& backup_dir)
    : status_mgr_(status_mgr), backup_dir_(backup_dir) {
    // Ensure backup directory exists
    if (!fs::exists(backup_dir_)) {
        fs::create_directories(backup_dir_);
    }
}

BackupManager::~BackupManager() {
}

std::string BackupManager::createBackup(const std::string& source_path, const std::string& task_id) {
    try {
        if (status_mgr_) {
            status_mgr_->setMessage("Backing up current version...");
        }

        std::cout << "[BackupManager] Backing up current version..." << std::endl;
        std::cout << "[BackupManager] From: " << source_path << std::endl;

        // Resolve source path (handle symlinks)
        fs::path source(source_path);
        if (!fs::exists(source)) {
            std::cerr << "[BackupManager] ERROR: Source path does not exist: " << source_path << std::endl;
            return "";
        }

        // Get the real target path (resolve symlink if source is a symlink)
        fs::path target;
        if (fs::is_symlink(source)) {
            target = fs::read_symlink(source);
            // If relative path, make it absolute
            if (target.is_relative()) {
                target = source.parent_path() / target;
            }
            target = fs::canonical(target);  // Resolve to absolute path
            std::cout << "[BackupManager] Source is symlink, resolved to: " << target << std::endl;
        } else {
            target = fs::canonical(source);
            std::cout << "[BackupManager] Source is directory: " << target << std::endl;
        }

        // Create backup symlink name
        std::string app_type = source.filename().string().find("ros") != std::string::npos ? "ros2" : "qr";
        std::string backup_name = app_type + "_backup_" + task_id;
        fs::path backup_path = fs::path(backup_dir_) / backup_name;

        std::cout << "[BackupManager] To:   " << backup_path << std::endl;
        std::cout << "[BackupManager] Target (absolute path): " << target << std::endl;

        // Remove existing backup if exists
        if (fs::exists(backup_path)) {
            fs::remove(backup_path);
            std::cout << "[BackupManager] Removed existing backup: " << backup_path << std::endl;
        }

        // **CRITICAL FIX**: Use ABSOLUTE path for symlink target
        // Before (BUG): fs::create_symlink(target.filename(), backup_path);  // Relative path!
        // After (FIXED): fs::create_symlink(target, backup_path);             // Absolute path!
        fs::create_symlink(target, backup_path);

        std::cout << "[BackupManager] Created symlink: " << backup_path << " -> " << target << std::endl;

        // Verify backup was created successfully
        if (!verifyBackup(backup_path.string())) {
            std::cerr << "[BackupManager] ERROR: Backup directory does not exist after backup" << std::endl;
            return "";
        }

        if (status_mgr_) {
            status_mgr_->setMessage("Backup completed successfully");
        }

        std::cout << "[BackupManager] ✓ Backup completed successfully" << std::endl;
        return backup_path.string();

    } catch (const std::exception& e) {
        std::cerr << "[BackupManager] Exception during backup: " << e.what() << std::endl;
        return "";
    }
}

bool BackupManager::verifyBackup(const std::string& backup_path) {
    std::cout << "[BackupManager] Verifying backup: " << backup_path << std::endl;

    // Check if backup symlink exists
    if (!fs::exists(backup_path)) {
        std::cerr << "[BackupManager] Backup path does not exist: " << backup_path << std::endl;
        return false;
    }

    // Check if it's a symlink
    if (!fs::is_symlink(backup_path)) {
        std::cerr << "[BackupManager] Backup path is not a symlink: " << backup_path << std::endl;
        return false;
    }

    // Check if symlink target exists
    fs::path target = fs::read_symlink(backup_path);

    // Resolve relative path if needed
    if (target.is_relative()) {
        target = fs::path(backup_path).parent_path() / target;
    }

    if (!fs::exists(target)) {
        std::cerr << "[BackupManager] Backup symlink target does not exist: " << target << std::endl;
        std::cerr << "[BackupManager]   Symlink: " << backup_path << std::endl;
        std::cerr << "[BackupManager]   Target:  " << target << std::endl;
        return false;
    }

    std::cout << "[BackupManager] ✓ Backup verified successfully" << std::endl;
    std::cout << "[BackupManager]   Symlink: " << backup_path << std::endl;
    std::cout << "[BackupManager]   Target:  " << target << std::endl;
    return true;
}

void BackupManager::cleanupOldBackups(int max_backups) {
    if (max_backups <= 0) {
        return;
    }

    try {
        std::vector<fs::directory_entry> backups;
        for (const auto& entry : fs::directory_iterator(backup_dir_)) {
            if (entry.is_symlink() || entry.is_directory()) {
                backups.push_back(entry);
            }
        }

        // Sort by modification time (oldest first)
        std::sort(backups.begin(), backups.end(),
            [](const fs::directory_entry& a, const fs::directory_entry& b) {
                return fs::last_write_time(a) < fs::last_write_time(b);
            });

        // Remove oldest backups if exceeding limit
        int to_remove = static_cast<int>(backups.size()) - max_backups;
        for (int i = 0; i < to_remove; ++i) {
            std::cout << "[BackupManager] Removing old backup: " << backups[i].path() << std::endl;
            fs::remove(backups[i].path());
        }

    } catch (const std::exception& e) {
        std::cerr << "[BackupManager] Exception during cleanup: " << e.what() << std::endl;
    }
}

bool BackupManager::restoreBackup(const std::string& backup_path, const std::string& dest_path) {
    try {
        if (!fs::exists(backup_path)) {
            std::cerr << "[BackupManager] Backup does not exist: " << backup_path << std::endl;
            return false;
        }

        // Get backup target
        fs::path target = fs::read_symlink(backup_path);
        if (target.is_relative()) {
            target = fs::path(backup_path).parent_path() / target;
        }

        // Remove dest if exists
        if (fs::exists(dest_path)) {
            fs::remove(dest_path);
        }

        // Create symlink to restored version
        fs::create_symlink(fs::canonical(target), dest_path);

        std::cout << "[BackupManager] Restored backup from: " << target << std::endl;
        std::cout << "[BackupManager] To: " << dest_path << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[BackupManager] Exception during restore: " << e.what() << std::endl;
        return false;
    }
}

bool BackupManager::removeBackup(const std::string& backup_path) {
    try {
        if (fs::exists(backup_path)) {
            fs::remove(backup_path);
            std::cout << "[BackupManager] Removed backup: " << backup_path << std::endl;
            return true;
        } else {
            std::cerr << "[BackupManager] Backup directory does not exist, nothing to remove" << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "[BackupManager] Exception during remove: " << e.what() << std::endl;
        return false;
    }
}

size_t BackupManager::getBackupSize(const std::string& backup_path) {
    try {
        if (!fs::exists(backup_path)) {
            return 0;
        }

        fs::path target = fs::read_symlink(backup_path);
        if (target.is_relative()) {
            target = fs::path(backup_path).parent_path() / target;
        }

        size_t total_size = 0;
        for (const auto& entry : fs::recursive_directory_iterator(target)) {
            if (fs::is_regular_file(entry)) {
                total_size += fs::file_size(entry);
            }
        }
        return total_size;

    } catch (const std::exception& e) {
        std::cerr << "[BackupManager] Exception getting backup size: " << e.what() << std::endl;
        return 0;
    }
}

std::string BackupManager::executeCommand(const std::string& cmd) {
    return executor_.execute(cmd);
}

} // namespace ota
} // namespace iiri
