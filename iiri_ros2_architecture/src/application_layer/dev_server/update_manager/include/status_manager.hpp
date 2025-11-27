#pragma once

#include "update_types.hpp"
#include <string>
#include <mutex>

namespace iiri {
namespace ota {

class StatusManager {
public:
    StatusManager(const std::string& task_id, const std::string& status_file);
    ~StatusManager();

    void setState(UpdateState state);
    void setProgress(int progress);
    void setMessage(const std::string& message);

    UpdateState getState() const;
    int getProgress() const;
    std::string getMessage() const;

    void save();
    void load();

private:
    std::string task_id_;
    std::string status_file_;
    UpdateState state_;
    int progress_;
    std::string message_;
    mutable std::mutex mutex_;
};

} // namespace ota
} // namespace iiri
