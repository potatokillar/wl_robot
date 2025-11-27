#pragma once

#include "status_manager.hpp"
#include "command_executor.hpp"
#include <string>
#include <filesystem>

namespace iiri {
namespace ota {

class BackupManager {
public:
    BackupManager(StatusManager* status_mgr, const std::string& backup_dir);
    ~BackupManager();

    // Create backup of current installation
    // Returns backup path on success, empty string on failure
    std::string createBackup(const std::string& source_path, const std::string& task_id);

    // Verify backup exists and is valid
    bool verifyBackup(const std::string& backup_path);

    // Remove old backups (keep only max_backups)
    void cleanupOldBackups(int max_backups);

    // Restore from backup
    bool restoreBackup(const std::string& backup_path, const std::string& dest_path);

    // Remove specific backup
    bool removeBackup(const std::string& backup_path);

    // Get backup size
    size_t getBackupSize(const std::string& backup_path);

private:
    StatusManager* status_mgr_;
    std::string backup_dir_;
    CommandExecutor executor_;

    std::string executeCommand(const std::string& cmd);
};

} // namespace ota
} // namespace iiri
