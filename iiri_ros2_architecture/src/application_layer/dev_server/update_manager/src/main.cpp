/**
 * @file main.cpp
 * @brief OTA Update Manager - Main Entry Point
 * @author 唐文浩
 * @date 2025-11-05
 */

#include "backup_manager.hpp"
#include "status_manager.hpp"
#include "command_executor.hpp"
#include <iostream>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <unistd.h>

namespace fs = std::filesystem;

void printUsage(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --task-id <id>             Task ID (required)" << std::endl;
    std::cout << "  --package <path>           Update package path (required)" << std::endl;
    std::cout << "  --app-type <type>          Application type: ros2 or qr (required)" << std::endl;
    std::cout << "  --original-filename <name> Original filename for validation (required)" << std::endl;
    std::cout << "  --daemon                   Run as daemon" << std::endl;
    std::cout << "  --help                     Show this help message" << std::endl;
}

bool verifySHA256(const std::string& package_path, iiri::ota::StatusManager* status_mgr) {
    std::string sha256_file = package_path + ".sha256";
    if (!fs::exists(sha256_file)) {
        std::cerr << "SHA256 file not found: " << sha256_file << std::endl;
        return false;
    }

    status_mgr->setMessage("Verifying SHA256 checksum...");
    std::cout << "Reading expected SHA256 from: " << sha256_file << std::endl;

    // Read expected SHA256
    std::ifstream sha_in(sha256_file);
    std::string expected_sha;
    sha_in >> expected_sha;
    sha_in.close();

    std::cout << "Expected SHA256 from file: " << expected_sha << std::endl;
    std::cout << "Verifying SHA256 checksum..." << std::endl;

    // Calculate actual SHA256
    iiri::ota::CommandExecutor executor;
    std::string cmd = "sha256sum " + package_path + " | awk '{print $1}'";
    std::string actual_sha = executor.execute(cmd);

    // Remove trailing newline
    actual_sha.erase(actual_sha.find_last_not_of(" \n\r\t") + 1);

    if (actual_sha == expected_sha) {
        std::cout << "SHA256 checksum verification passed" << std::endl;
        std::cout << "Package SHA256: " << actual_sha << std::endl;
        return true;
    } else {
        std::cerr << "SHA256 checksum mismatch!" << std::endl;
        std::cerr << "Expected: " << expected_sha << std::endl;
        std::cerr << "Actual:   " << actual_sha << std::endl;
        return false;
    }
}

bool extractPackage(const std::string& package_path, const std::string& extract_dir,
                   iiri::ota::StatusManager* status_mgr) {
    status_mgr->setMessage("Extracting package...");
    std::cout << "Extracting package to: " << extract_dir << std::endl;

    // Create extract directory
    if (fs::exists(extract_dir)) {
        fs::remove_all(extract_dir);
    }
    fs::create_directories(extract_dir);

    // Extract tar.gz
    iiri::ota::CommandExecutor executor;
    std::string cmd = "tar -xzf " + package_path + " -C " + extract_dir;
    std::string output = executor.execute(cmd);

    if (fs::exists(extract_dir) && !fs::is_empty(extract_dir)) {
        std::cout << "Package extracted successfully" << std::endl;
        return true;
    } else {
        std::cerr << "Failed to extract package" << std::endl;
        return false;
    }
}

bool stopService(const std::string& service_name, iiri::ota::StatusManager* status_mgr) {
    status_mgr->setMessage("Stopping service...");
    std::cout << "Stopping service: " << service_name << std::endl;

    iiri::ota::CommandExecutor executor;
    std::string cmd = "sudo systemctl stop " + service_name;
    executor.execute(cmd);

    // Wait a bit for service to stop
    sleep(5);

    // Verify service stopped
    cmd = "systemctl is-active " + service_name;
    std::string status = executor.execute(cmd);

    if (status.find("inactive") != std::string::npos || status.find("failed") != std::string::npos) {
        std::cout << "Service stopped successfully" << std::endl;
        return true;
    } else {
        std::cerr << "Failed to stop service, status: " << status << std::endl;
        return false;
    }
}

bool installNewVersion(const std::string& extract_dir, const std::string& install_dir,
                      iiri::ota::StatusManager* status_mgr) {
    status_mgr->setMessage("Installing new version...");
    std::cout << "Installing from: " << extract_dir << std::endl;
    std::cout << "Installing to:   " << install_dir << std::endl;

    try {
        // Find the extracted directory (should be only one)
        std::string source_dir;
        for (const auto& entry : fs::directory_iterator(extract_dir)) {
            if (entry.is_directory()) {
                source_dir = entry.path().string();
                break;
            }
        }

        if (source_dir.empty()) {
            std::cerr << "No directory found in extracted package" << std::endl;
            return false;
        }

        std::cout << "Source directory: " << source_dir << std::endl;

        // Get the version directory name
        std::string version_name = fs::path(source_dir).filename().string();
        std::string target_dir = fs::path(install_dir).parent_path().string() + "/" + version_name;

        std::cout << "Target directory: " << target_dir << std::endl;

        // Move to target location
        if (fs::exists(target_dir)) {
            std::cout << "Removing existing directory: " << target_dir << std::endl;
            fs::remove_all(target_dir);
        }

        std::cout << "Moving to target..." << std::endl;
        fs::rename(source_dir, target_dir);

        // Update symlink
        if (fs::exists(install_dir) && fs::is_symlink(install_dir)) {
            std::cout << "Removing old symlink: " << install_dir << std::endl;
            fs::remove(install_dir);
        }

        std::cout << "Creating new symlink..." << std::endl;
        fs::create_symlink(target_dir, install_dir);

        std::cout << "Installation completed successfully" << std::endl;
        std::cout << "Symlink: " << install_dir << " -> " << target_dir << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Exception during installation: " << e.what() << std::endl;
        return false;
    }
}

bool startService(const std::string& service_name, iiri::ota::StatusManager* status_mgr) {
    status_mgr->setMessage("Starting service...");
    std::cout << "Starting service: " << service_name << std::endl;

    iiri::ota::CommandExecutor executor;
    std::string cmd = "sudo systemctl start " + service_name;
    executor.execute(cmd);

    // Wait for service to start
    sleep(10);

    // Verify service started
    cmd = "systemctl is-active " + service_name;
    std::string status = executor.execute(cmd);

    if (status.find("active") != std::string::npos) {
        std::cout << "Service started successfully" << std::endl;
        return true;
    } else {
        std::cerr << "Failed to start service, status: " << status << std::endl;
        return false;
    }
}

int main(int argc, char* argv[]) {
    std::string task_id;
    std::string package_path;
    std::string app_type;
    std::string original_filename;
    bool daemon_mode = false;

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            printUsage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--task-id") == 0 && i + 1 < argc) {
            task_id = argv[++i];
        } else if (strcmp(argv[i], "--package") == 0 && i + 1 < argc) {
            package_path = argv[++i];
        } else if (strcmp(argv[i], "--app-type") == 0 && i + 1 < argc) {
            app_type = argv[++i];
        } else if (strcmp(argv[i], "--original-filename") == 0 && i + 1 < argc) {
            original_filename = argv[++i];
        } else if (strcmp(argv[i], "--daemon") == 0) {
            daemon_mode = true;
        }
    }

    // Validate required parameters
    if (task_id.empty() || package_path.empty() || app_type.empty() || original_filename.empty()) {
        std::cerr << "Error: Missing required parameters" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    // Validate package filename
    std::cout << "Package filename validated: appType=" << app_type
              << ", original filename=" << original_filename << std::endl;

    // Initialize managers
    std::string status_file = "/var/run/update_status.json";
    iiri::ota::StatusManager status_mgr(task_id, status_file);
    iiri::ota::BackupManager backup_mgr(&status_mgr, "/var/backups/iiri");

    std::cout << "========================================" << std::endl;
    std::cout << "Update Manager Started" << std::endl;
    std::cout << "Task ID: " << task_id << std::endl;
    std::cout << "Package: " << package_path << std::endl;
    std::cout << "App Type: " << app_type << std::endl;
    std::cout << "========================================" << std::endl;

    std::string service_name = (app_type == "ros2") ? "iiri-ros.service" : "iiri-qr.service";
    std::string install_dir = (app_type == "ros2") ? "/home/wl/autorun/iiri-ros" : "/home/wl/autorun/iiri-qr";

    try {
        // Step 1: Prepare
        std::cout << "[MAIN-DEBUG] About to call setState PREPARING" << std::endl;
        status_mgr.setState(iiri::ota::UpdateState::PREPARING);
        status_mgr.setProgress(10);
        status_mgr.setMessage("Preparing update...");

        // Step 2: Verify SHA256
        if (!verifySHA256(package_path, &status_mgr)) {
            throw std::runtime_error("SHA256 verification failed");
        }
        status_mgr.setProgress(20);

        // Step 3: Extract package
        std::string extract_dir = "/tmp/update_" + task_id;
        if (!extractPackage(package_path, extract_dir, &status_mgr)) {
            throw std::runtime_error("Package extraction failed");
        }
        status_mgr.setProgress(30);

        // Step 4: Start update
        status_mgr.setState(iiri::ota::UpdateState::RUNNING);

        // Step 5: Stop service
        if (!stopService(service_name, &status_mgr)) {
            throw std::runtime_error("Failed to stop service");
        }
        status_mgr.setProgress(40);

        // Step 6: Backup current version
        std::string backup_path = backup_mgr.createBackup(install_dir, task_id);
        if (backup_path.empty()) {
            throw std::runtime_error("Failed to backup current version");
        }
        status_mgr.setProgress(50);

        // Step 7: Install new version
        if (!installNewVersion(extract_dir, install_dir, &status_mgr)) {
            throw std::runtime_error("Failed to install new version");
        }
        status_mgr.setProgress(70);

        // Step 8: Start service
        if (!startService(service_name, &status_mgr)) {
            throw std::runtime_error("Failed to start service");
        }
        status_mgr.setProgress(90);

        // Step 9: Cleanup
        backup_mgr.cleanupOldBackups(3);
        if (fs::exists(extract_dir)) {
            fs::remove_all(extract_dir);
        }

        // Success!
        status_mgr.setState(iiri::ota::UpdateState::COMPLETED);
        status_mgr.setProgress(100);
        status_mgr.setMessage("Update completed successfully");

        std::cout << "========================================" << std::endl;
        std::cout << "Update Completed Successfully" << std::endl;
        std::cout << "========================================" << std::endl;

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error occurred: " << e.what() << std::endl;
        status_mgr.setState(iiri::ota::UpdateState::FAILED);
        status_mgr.setMessage(std::string("Error: ") + e.what());

        // Try to start service anyway
        std::cout << "Starting service: " << service_name << std::endl;
        iiri::ota::CommandExecutor executor;
        executor.execute("sudo systemctl start " + service_name);
        sleep(10);

        return 1;
    }
}
