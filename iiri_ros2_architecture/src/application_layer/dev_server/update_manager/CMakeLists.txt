cmake_minimum_required(VERSION 3.10)
project(update_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/include
)

# Source files
set(SOURCES
    src/main.cpp
    src/backup_manager.cpp
    src/status_manager.cpp
    src/command_executor.cpp
)

# Executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    stdc++fs  # For std::filesystem
    pthread   # For std::mutex
)

# Install
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)
