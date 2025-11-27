#pragma once

#include <string>
#include <vector>

namespace iiri {
namespace ota {

class CommandExecutor {
public:
    CommandExecutor();
    ~CommandExecutor();

    // Execute shell command and return output
    std::string execute(const std::string& command);

    // Execute command with timeout (seconds)
    std::string executeWithTimeout(const std::string& command, int timeout_seconds);

    // Check if command succeeded
    bool checkCommandSuccess(const std::string& command);

private:
    std::string runCommand(const std::string& cmd);
};

} // namespace ota
} // namespace iiri
