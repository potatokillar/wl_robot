#include "status_manager.hpp"
#include <fstream>
#include <iostream>
#include <sys/stat.h>

namespace iiri {
namespace ota {

StatusManager::StatusManager(const std::string& task_id, const std::string& status_file)
    : task_id_(task_id), status_file_(status_file), state_(UpdateState::IDLE), progress_(0) {
    std::cout << "[" << task_id_ << "] StatusManager initialized for task: " << task_id_ << std::endl;
}

StatusManager::~StatusManager() {
    std::cout << "[" << task_id_ << "] StatusManager shutting down" << std::endl;
}

void StatusManager::setState(UpdateState state) {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
    std::string state_str;
    switch(state) {
        case UpdateState::IDLE: state_str = "IDLE"; break;
        case UpdateState::PREPARING: state_str = "PREPARING"; break;
        case UpdateState::RUNNING: state_str = "RUNNING"; break;
        case UpdateState::COMPLETED: state_str = "SUCCESS"; break;  // ✅ Match dev_server expectation
        case UpdateState::FAILED: state_str = "FAILED"; break;
        case UpdateState::CANCELLED: state_str = "CANCELLED"; break;
    }
    std::cout << "[" << task_id_ << "] State changed to: " << state_str << std::endl;
    save();
}

void StatusManager::setProgress(int progress) {
    std::lock_guard<std::mutex> lock(mutex_);
    progress_ = progress;
    save();
}

void StatusManager::setMessage(const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    message_ = message;
    std::cout << "[" << task_id_ << "] Status message: " << message << std::endl;
    save();
}

UpdateState StatusManager::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

int StatusManager::getProgress() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return progress_;
}

std::string StatusManager::getMessage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return message_;
}

void StatusManager::save() {
    // Convert state to string
    std::string state_str;
    switch(state_) {
        case UpdateState::IDLE: state_str = "IDLE"; break;
        case UpdateState::PREPARING: state_str = "PREPARING"; break;
        case UpdateState::RUNNING: state_str = "RUNNING"; break;
        case UpdateState::COMPLETED: state_str = "SUCCESS"; break;  // ✅ Match dev_server expectation
        case UpdateState::FAILED: state_str = "FAILED"; break;
        case UpdateState::CANCELLED: state_str = "CANCELLED"; break;
    }

    // Write JSON file (simple format without external library)
    std::ofstream ofs(status_file_);
    if (ofs.is_open()) {
        ofs << "{\n";
        ofs << "  \"taskId\": \"" << task_id_ << "\",\n";  // ✅ Camel case to match dev_server
        ofs << "  \"state\": \"" << state_str << "\",\n";
        ofs << "  \"progress\": " << progress_ << ",\n";
        ofs << "  \"message\": \"" << message_ << "\"\n";
        ofs << "}\n";
        ofs.close();

        // Set permissions so dev_server can read it
        chmod(status_file_.c_str(), 0666);
    } else {
        std::cerr << "[" << task_id_ << "] Failed to write status file: " << status_file_ << std::endl;
    }
}

void StatusManager::load() {
    // Placeholder - implement JSON deserialization if needed
}

} // namespace ota
} // namespace iiri
