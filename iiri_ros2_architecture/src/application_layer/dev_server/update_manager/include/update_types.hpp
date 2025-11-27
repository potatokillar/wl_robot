#pragma once

#include <string>

namespace iiri {
namespace ota {

enum class UpdateState {
    IDLE,
    PREPARING,
    RUNNING,
    COMPLETED,
    FAILED,
    CANCELLED
};

struct UpdateTask {
    std::string task_id;
    std::string package_path;
    std::string app_type;  // "ros2" or "qr"
    std::string original_filename;
    UpdateState state;
    int progress;
    std::string message;
};

} // namespace ota
} // namespace iiri
