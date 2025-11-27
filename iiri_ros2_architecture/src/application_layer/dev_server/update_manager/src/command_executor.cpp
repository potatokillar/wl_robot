#include "command_executor.hpp"
#include <array>
#include <memory>
#include <stdexcept>
#include <iostream>

namespace iiri {
namespace ota {

CommandExecutor::CommandExecutor() {
}

CommandExecutor::~CommandExecutor() {
}

std::string CommandExecutor::execute(const std::string& command) {
    return runCommand(command);
}

std::string CommandExecutor::executeWithTimeout(const std::string& command, int timeout_seconds) {
    std::string cmd_with_timeout = "timeout " + std::to_string(timeout_seconds) + " " + command;
    return runCommand(cmd_with_timeout);
}

bool CommandExecutor::checkCommandSuccess(const std::string& command) {
    int result = system((command + " > /dev/null 2>&1").c_str());
    return result == 0;
}

std::string CommandExecutor::runCommand(const std::string& cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    
    return result;
}

} // namespace ota
} // namespace iiri
