# CLAUDE.md

This file provides guidance to AI assistants when working with code in this repository.

## Project Overview

This is an **IIRI ROS2 Layered Architecture** project using ROS2 Humble with a five-layer design pattern managed by vcstool. The architecture separates concerns into distinct layers, each maintained as an independent Git repository.

### Five-Layer Architecture

The layers follow a strict dependency hierarchy (lower layers are dependencies of upper layers):

1. **core_layer** - Base layer with message definitions (`interface`) and third-party tools (`backward_ros`)
   - Depends on: ROS2 Humble base environment
   - Repository: `http://192.168.1.55/ontology/iiri-core-layer.git`

2. **hardware_layer** - Hardware drivers and low-level interfaces
   - Depends on: core_layer
   - Contains: motion_control, robot_base, sensor drivers
   - Repository: `http://192.168.1.55/ontology/iiri-hardware-layer.git`

3. **perception_layer** - Perception data processing
   - Depends on: hardware_layer
   - Contains: camera_ptz, speaker, speech_recognition, tts
   - Repository: `http://192.168.1.55/ontology/iiri-perception-layer.git`

4. **intelligence_layer** - Intelligent decision-making and navigation
   - Depends on: perception_layer
   - Contains: navigation, bt_manager, smart_follow, path_tracker (with optional Ceres optimization)
   - Repository: `http://192.168.1.55/ontology/iiri-intelligence-layer.git`

5. **application_layer** - Application logic and communication
   - Depends on: intelligence_layer
   - Contains: bringup, dev_server, key_control, record, remote_ctrl
   - Repository: `http://192.168.1.55/ontology/iiri-application-layer.git`

### ROS2 Overlay Mechanism

Each layer uses ROS2 overlay to build upon previous layers:
- core_layer sources `/opt/ros/humble/setup.bash`
- Each subsequent layer sources its own `setup.bash` which chains all previous layers
- The final environment at application_layer includes all five layers

## Build Commands

### Quick Build (Recommended for New Users)

```bash
# Build all layers for current architecture
./build.sh

# Build with architecture specification
./build.sh x86          # For x86_64
./build.sh arm          # For ARM/aarch64

# Clean build (removes all cached build artifacts)
./build.sh -c
./build.sh -c x86
```

### Layered Build (For Fine-Grained Control)

```bash
# Build specific layer and its dependencies
./build_layered.sh core_layer              # Only core layer
./build_layered.sh hardware_layer          # core + hardware
./build_layered.sh perception_layer        # core + hardware + perception
./build_layered.sh intelligence_layer      # Up to intelligence layer
./build_layered.sh application_layer       # All layers (default)

# With architecture specification
./build_layered.sh x86 intelligence_layer
./build_layered.sh arm application_layer

# Clean build specific layer
./build_layered.sh -c core_layer
./build_layered.sh -c x86 perception_layer

# Enable Ceres optimization (for intelligence_layer's path_tracker)
./build_layered.sh --ceres intelligence_layer
./build_layered.sh -c --ceres x86 application_layer
```

**Important**: The `--ceres` flag enables Ceres optimization specifically for the `path_tracker` package in intelligence_layer. This provides higher precision path tracking at the cost of longer build times.

### Build Inside Docker Container

```bash
# Enter Docker environment
./docker.sh run

# Inside container, use the internal build script
./script/build_layered.sh [-c] [--ceres] [layer_name]
```

### Docker Images

- **x86**: `192.168.1.93/iiri/build_x86_ros2:v1.4.3`
- **ARM**: `192.168.1.93/iiri/build_arm_ros2:v1.4.2`

## Code Management with vcstool

This project uses **vcstool** to manage multiple Git repositories as a unified workspace.

### Initial Setup

```bash
# One-time installation (recommended for new users)
./install.sh                    # Install dependencies and import all layers
./install.sh --config devel     # Use development branches
./install.sh --no-deps          # Skip dependency installation

# Manual import (if needed)
./sync.sh import
```

### Daily Workflow Commands

```bash
# Check status of all layer repositories
./sync.sh status

# Pull updates from all repositories
./sync.sh pull

# Switch between configurations
./sync.sh switch main           # Production branches
./sync.sh switch devel          # Development branches
./sync.sh switch stable         # Stable release branches

# Clean all layers (removes src/ directory)
./sync.sh clean
```

### Working on a Specific Layer

```bash
# Navigate to the layer you want to work on
cd src/intelligence_layer

# Create a feature branch
git checkout -b feature/new-navigation

# Make changes, commit, and push
git add .
git commit -m "feat: implement new navigation algorithm"
git push origin feature/new-navigation

# Return to workspace root and test build
cd ../..
./build_layered.sh intelligence_layer
```

### Version Management

```bash
# Validate release readiness
./release.sh validate

# View version status of all layers
./release.sh status

# Create a new version release
./release.sh create v1.2.0

# List all version tags
./release.sh list
```

## Build Architecture Details

### Build Directory Structure

Builds are organized by architecture and layer:

```
build_x86_shared/                # x86 shared build directory
├── core_layer/
│   ├── build/                   # Build artifacts
│   └── log/                     # Build logs
├── hardware_layer/
│   ├── build/
│   └── log/
├── install/                     # Shared install space for all layers
...
```

For ARM: `build_arm_shared/`

### Build Types

- **x86**: `CMAKE_BUILD_TYPE=Debug` (with GPU support)
- **ARM**: `CMAKE_BUILD_TYPE=Release` (with NVIDIA runtime on Jetson platforms)

## OTA Update Manager (update_manager)

**IMPORTANT**: `update_manager` is a **standalone component** that must be compiled and deployed **independently** from the ROS2 cluster. It is NOT part of the ROS2 build system.

### Why Independent Deployment?

The `update_manager` is responsible for updating the ROS2 cluster itself. If it were packaged inside the ROS2 installation directory, it would:
1. **Self-destruct**: Get deleted/replaced while updating the very directory it's running from
2. **Version conflicts**: Unclear which version (old/new) is being used during updates
3. **Race conditions**: Binary file could be overwritten while still executing

### Architecture

```
/home/wl/autorun/
├── update_manager/              # ✅ Independent deployment
│   ├── update_manager           # Binary (181KB)
│   ├── config.conf              # Configuration
│   └── config/                  # Config directory
└── iiri-ros/                    # ✅ ROS2 cluster (can be updated)
    └── install/
        └── dev_server/
            └── lib/dev_server/dev_server_node
```

### Build Process

**Location**: `src/application_layer/dev_server/update_manager/`

#### Step 1: Build in Docker

```bash
cd src/application_layer/dev_server/update_manager
./build_in_docker.sh arm   # For ARM architecture
./build_in_docker.sh x86   # For x86 architecture
```

This script:
1. Uses the ARM/x86 Docker image
2. Installs dependencies (OpenSSL, nlohmann-json, CMake, rsync)
3. Runs CMake configuration
4. Compiles with `make -j$(nproc)`
5. Outputs binary to `build/update_manager` (~181KB for ARM)

#### Step 2: Deploy to Remote Server

```bash
cd src/application_layer/dev_server/update_manager
./deploy_update_manager.sh 192.168.1.54   # Or any target host
```

This script:
1. Creates deployment package (tar.gz)
2. Uploads to remote server via scp
3. Extracts to `/home/wl/autorun/update_manager/`
4. Sets proper permissions
5. Creates configuration file
6. Verifies deployment with `--help`

### Verification

After deployment, verify on the remote server:

```bash
ssh wl@192.168.1.54
/home/wl/autorun/update_manager/update_manager --help

# Expected output includes:
#   --task-id <id>
#   --package <path>
#   --app-type <type>
#   --original-filename <name>  # ✅ Required for validation
#   --daemon
```

### Complete System Architecture

#### Overview

The OTA update system consists of three main components:

1. **Web Frontend** (Vue3) - User interface for uploading packages and monitoring progress
2. **dev_server** (ROS2 Node) - Backend service handling file uploads and process management
3. **update_manager** (Standalone C++ binary) - Executes the actual update operations

#### Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                            用户浏览器 (Web UI)                        │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌──────────────┐ │
│  │ 上传组件    │  │ 进度展示   │  │ 日志查看   │  │ 版本管理     │ │
│  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘  └──────┬───────┘ │
└────────┼───────────────┼──────────────┼──────────────────┼─────────┘
         │               │              │                  │
    HTTP POST      WebSocket      HTTP GET           HTTP GET
    /api/upload     /ws/update    /api/status      /api/versions
         │               │              │                  │
         ▼               ▼              ▼                  ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        dev_server (端口 8080)                        │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────────┐│
│  │UpdateService │  │WebSocketMgr  │  │StatusMonitor               ││
│  │ - 文件管理    │  │ - 实时推送   │  │ - 状态查询                 ││
│  │ - 任务调度    │  │ - 进度同步   │  │ - 日志聚合                 ││
│  └──────┬───────┘  └──────┬───────┘  └────────┬───────────────────┘│
│         │                  │                    │                    │
│         └──────────────────┼────────────────────┘                    │
│                            │                                         │
│              sudo systemd-run --scope                                │
│                            │                                         │
└────────────────────────────┼─────────────────────────────────────────┘
                             ▼
                    ┌─────────────────┐
                    │  update_manager │ (独立进程)
                    │  (与 iiri-ros   │
                    │   无父子关系)    │
                    └────────┬────────┘
                             │
                ┌────────────┴───────────┐
                ▼                        ▼
        ┌──────────────┐        ┌──────────────┐
        │ ROS2 Updater │        │ QR Updater   │
        └──────┬───────┘        └──────┬───────┘
               │                        │
        systemctl 操作            pkill/start 操作
               │                        │
        ┌──────▼───────┐        ┌──────▼───────┐
        │ iiri-ros     │        │ iiri-qr      │
        └──────────────┘        └──────────────┘
```

#### Update State Machine

```
                    ┌─────────┐
                    │  IDLE   │ 初始状态
                    └────┬────┘
                         │ handleUpload()
                         ▼
                    ┌─────────┐
                    │PREPARING│ 验证文件名、SHA256
                    └────┬────┘
                         │ startUpdate()
                         ▼
                    ┌─────────┐
               ┌────│ RUNNING │────┐
               │    └────┬────┘    │
               │         │         │
     用户取消  │    正常完成│      │ 执行失败
               │         ▼         │
               │    ┌─────────┐    │
               │    │COMPLETED│    │
               │    └─────────┘    │
               ▼                   ▼
          ┌──────────┐        ┌────────┐
          │CANCELLED │        │ FAILED │
          └──────────┘        └────────┘
```

#### Detailed Update Workflow

**Phase 1: File Upload (dev_server/UpdateService)**

1. User uploads package via Web UI (POST /api/upload)
   - Required files: `package.tar.gz` + `package.tar.gz.sha256`
   - Required params: `appType` (ros2/qr), `packageFilename`

2. `UpdateService::handleUpload()` validates filename:
   - ROS2 packages must contain "ros" or "ROS" in filename
   - QR packages must contain "qr" or "QR" in filename
   - Prevents user from uploading wrong package for wrong app type

3. Files saved to `/tmp/updates/{taskId}.tar.gz` and `/tmp/updates/{taskId}.tar.gz.sha256`

4. Task created with unique ID: `update-YYYYMMDD-HHMMSS-random`

**Phase 2: Launch Update Process (dev_server → update_manager)**

1. User clicks "Start Update" (POST /api/update/start)

2. `UpdateService::startUpdate()` executes:
   ```bash
   sudo systemd-run --scope \
     --unit=update-manager-{taskId} \
     --description="OTA Update Manager for {appType}" \
     --slice=system.slice \
     bash -c '/home/wl/autorun/update_manager/update_manager \
       --task-id {taskId} \
       --package /tmp/updates/{taskId}.tar.gz \
       --app-type {appType} \
       --original-filename "{filename}" \
       --daemon \
       > /tmp/updates/{taskId}.log 2>&1'
   ```

3. Why `systemd-run --scope`?
   - Creates a new systemd scope unit independent of iiri-ros.service
   - When iiri-ros.service stops during update, update_manager continues running
   - Prevents parent-child process relationship that would kill update_manager

**Phase 3: Update Execution (update_manager)**

1. **Verify SHA256** (10% progress)
   - Read expected checksum from `.sha256` file
   - Calculate actual checksum: `sha256sum package.tar.gz`
   - Abort if mismatch

2. **Extract Package** (20-30% progress)
   ```bash
   mkdir -p /tmp/update_{taskId}
   tar -xzf package.tar.gz -C /tmp/update_{taskId}/
   ```

3. **Stop Service** (40% progress)
   ```bash
   sudo systemctl stop iiri-ros.service  # or iiri-qr.service
   ```
   Wait 5 seconds and verify with `systemctl is-active`

4. **Backup Current Version** (50% progress)
   ```cpp
   // Resolve symlink to actual directory
   fs::path source = "/home/wl/autorun/iiri-ros";  // or iiri-qr
   fs::path target = fs::canonical(fs::read_symlink(source));

   // Create backup symlink pointing to current version
   fs::path backup = "/var/backups/iiri/ros2_backup_{taskId}";
   fs::create_symlink(target, backup);  // ✅ ABSOLUTE PATH (bug fixed)
   ```

5. **Install New Version** (70% progress)
   ```cpp
   // Extract produces: /tmp/update_{taskId}/iiri-ros-arm-{version}/
   std::string version_dir = find_extracted_directory();
   std::string target_dir = "/home/wl/autorun/" + version_dir;

   // Move extracted directory to autorun/
   fs::rename(extract_dir + "/" + version_dir, target_dir);

   // Update symlink to point to new version
   fs::remove("/home/wl/autorun/iiri-ros");
   fs::create_symlink(target_dir, "/home/wl/autorun/iiri-ros");
   ```

6. **Start Service** (90% progress)
   ```bash
   sudo systemctl start iiri-ros.service
   ```
   Wait 10 seconds and verify with `systemctl is-active`

7. **Cleanup** (100% progress)
   - Delete `/tmp/update_{taskId}/`
   - Keep last 3 backups, delete older ones

**Phase 4: Status Monitoring (dev_server → Web UI)**

1. `UpdateService::monitorStatusFile()` runs in separate thread

2. Polls `/var/run/update_status.json` every 1 second:
   ```json
   {
     "task_id": "update-20251105-143000-abc123",
     "state": "RUNNING",
     "progress": 50,
     "message": "Backing up current version..."
   }
   ```

3. On status change, invoke callback → WebSocket push to frontend

4. Frontend updates progress bar and status message in real-time

### Integration with dev_server

The `dev_server` node calls `update_manager` via:
- **Path**: `/home/wl/autorun/update_manager/update_manager` (hardcoded in `update_service.hpp:178`)
- **Method**: `systemd-run --scope` (ensures process isolation from iiri-ros.service)
- **Communication**: Shared status file at `/var/run/update_status.json`

**Key Implementation Files**:
- `src/application_layer/src/dev_server/src/update_service.hpp` - UpdateService class definition
- `src/application_layer/src/dev_server/src/update_service.cpp` - File upload, process launching, status monitoring
- `src/application_layer/dev_server/update_manager/src/main.cpp` - Update execution logic
- `src/application_layer/dev_server/update_manager/src/status_manager.cpp` - JSON status file writing

### Key Features

1. **Process Isolation**: Launched via `systemd-run --scope` to survive ROS2 service restart
2. **Filename Validation**: Double validation (upload + start) ensures package matches app type
3. **SHA256 Verification**: Validates package integrity before extraction
4. **Symlink-Based Versioning**: Multiple versions can coexist, quick rollback via symlink
5. **Atomic Updates**: Backup → Stop → Update → Start → Cleanup (transactional approach)
6. **Status Tracking**: Real-time progress updates via shared JSON file + WebSocket push
7. **Error Recovery**: Automatic service restart on failure, manual rollback via symlink

### Configuration File

Location: `/home/wl/autorun/update_manager/config.conf`

```ini
# Log configuration
log_dir = /var/log/update_manager
log_level = INFO

# Backup configuration
backup_dir = /var/backups/iiri
max_backups = 3

# Status file
status_file = /tmp/update_manager_status.json

# Service configuration
ros2_service = iiri-ros.service
qr_service = iiri-qr.service

# Install paths
ros2_install_dir = /home/wl/autorun/iiri-ros
qr_install_dir = /home/wl/autorun/iiri-qr
```

### Common Workflow

```bash
# 1. Modify update_manager source code
vim src/application_layer/dev_server/update_manager/src/main.cpp

# 2. Rebuild
cd src/application_layer/dev_server/update_manager
./build_in_docker.sh arm

# 3. Deploy
./deploy_update_manager.sh 192.168.1.54

# 4. Verify
ssh wl@192.168.1.54 "/home/wl/autorun/update_manager/update_manager --help"

# 5. Test update flow via web UI
# Open http://192.168.1.54:8080/#/ota-update
```

### Exclusion from colcon

The update_manager directory contains a **`.colconignore`** file to prevent colcon from building it:

```bash
src/application_layer/dev_server/update_manager/.colconignore
```

This ensures:
- ✅ Only one build method exists (independent via `build_in_docker.sh`)
- ✅ No confusion between colcon-built (2.8M) vs independent-built (181KB) versions
- ✅ Faster ROS2 compilation (one less package)
- ✅ Clear separation of concerns

**Verification**: After adding `.colconignore`, `colcon list` in `src/application_layer` should NOT show `update_manager`.

### DO NOT

❌ **DO NOT** remove `.colconignore` (it prevents accidental colcon builds)
❌ **DO NOT** include `update_manager` in ROS2 package builds (`colcon build`)
❌ **DO NOT** deploy `update_manager` from `iiri-ros/install/update_manager/`
❌ **DO NOT** expect `update_manager` to update itself (it's separate for a reason)

### Related Documentation

- **Complete Design Document**: `/home/wl/twh/workspace/OTA/OTA_UPDATE_SYSTEM_DESIGN.md` (Production-grade design with advanced features)
- **Architecture Diagrams**: `/home/wl/twh/workspace/OTA/OTA_*.dot` (Graphviz source files)
- **Leadership Presentation**: `/home/wl/twh/workspace/OTA/OTA_FLOWCHARTS_FOR_LEADERSHIP.md`
- **Frontend Implementation**: `src/application_layer/src/dev_server/frontend_src/OTA_FRONTEND_IMPLEMENTATION.md`
- **Testing Guide**: `OTA_TEST_GUIDE.md`
- **Bug Fixes**: `OTA_BUGFIX_SUMMARY.md`
- **Status Tracking**: `OTA_STATUS_TRACKING_SOLUTION.md`

**Note**: The complete design document (`OTA_UPDATE_SYSTEM_DESIGN.md`) contains production-grade features not yet implemented in the current MVP version, including:
- Version compatibility checking (VersionManager)
- Advanced health checks (HealthChecker)
- Process locking (flock) to prevent concurrent updates
- Signal handling for graceful shutdown
- GPG signature verification
- Disaster recovery mechanisms
- Automatic retry with exponential backoff
- Power failure recovery

## Common Issues and Solutions

### CMake Cache Path Mismatch

**Symptom**: Error like "The current CMakeCache.txt directory is different than the directory where CMakeCache.txt was created"

**Cause**: Project was moved to a different path, but CMakeCache.txt still references the old path.

**Solution**:
```bash
# Clean build the affected layer
./build_layered.sh -c core_layer

# Or manually remove the build cache
rm -rf build_x86_shared/core_layer/build
# Then rebuild
./build_layered.sh core_layer
```

**Prevention**: Always use clean build (`-c` flag) after moving the project directory.

### Build Failures

1. **Check Docker connectivity**: Ensure you can pull from `192.168.1.93`
2. **Verify layer dependencies**: Build layers in order (core → hardware → perception → intelligence → application)
3. **Clean build**: Use `-c` flag to remove stale build artifacts
4. **Check logs**: Build logs are in `build_*/[layer]/log/`

### vcstool Import Failures

**Symptom**: Cannot import repositories

**Solution**:
```bash
# Check network connectivity
ping 192.168.1.55

# Verify Git credentials
git config --global credential.helper store

# Re-run import
./sync.sh import
```

## Environment Setup

### ROS2 Environment Sourcing

After building, source the appropriate layer's setup file:

```bash
# For application layer (includes all layers)
cd src/application_layer
source setup.bash

# For testing a specific layer
cd src/intelligence_layer
source setup.bash
```

### Environment Variables

Key environment variables set by `iiri_env.sh`:
- `DOCKER_IMG_X86`: x86 Docker image
- `DOCKER_IMG_ARM`: ARM Docker image
- `DOCKER_RUN_FLAG`: Docker runtime flags (GPU, volumes, network)

## Development Best Practices

### When Developing in a Specific Layer

1. Always build from the workspace root, not from within a layer
2. Use layered build to test only affected layers and their dependencies
3. Clean build (`-c`) when switching branches or after significant changes
4. Commit and push layer changes to their respective repositories
5. Update main workspace `.repos` file only when changing layer versions

### Before Creating a Pull Request

1. Ensure clean build succeeds: `./build_layered.sh -c [your_layer]`
2. Test the full stack: `./build_layered.sh application_layer`
3. Verify no uncommitted changes: `./sync.sh status`
4. Run validation: `./release.sh validate`

### Ceres Optimization Guidelines

- Only enable `--ceres` when working on or testing `path_tracker` in intelligence_layer
- Keep Ceres settings consistent between intelligence_layer and application_layer builds
- Default builds (without `--ceres`) are faster and sufficient for most development

## Project Configuration Files

- `.repos` - Current vcstool configuration (main branches by default)
- `.repos.main` - Production branch configuration
- `.repos.devel` - Development branch configuration
- `.repos.stable` - Stable release configuration
- `iiri_env.sh` - Docker environment configuration
- `env.tmp` - Runtime environment variables (generated dynamically)

## System Bringup Package (core_layer)

**Key Innovation**: The `system_bringup` package provides layered system startup capabilities and is located in **core_layer** instead of application_layer.

### Why This Matters

**Problem Solved**:
- Old approach: bringup in application_layer → must compile all 5 layers
- New approach: system_bringup in core_layer → only need 3 layers to run full system

**Practical Benefits**:
- **Raspberry Pi**: Build up to intelligence_layer (3 layers) → full system available
- **Jetson Orin**: Build up to application_layer (4 layers) → all features available
- **Development**: Build only core_layer (1 layer) → fast iteration on launch logic

### Launch Files

Located at: `src/core_layer/src/system_bringup/launch/`

| Launch File | Layer | Purpose |
|-------------|-------|---------|
| `1_hardware.launch.py` | Hardware | Motion control, robot base |
| `2_perception.launch.py` | Perception | Camera, TTS, speech recognition |
| `3_intelligence.launch.py` | Intelligence | Navigation, behavior trees, smart follow |
| `4_application.launch.py` | Application | Dev server, recording, remote control |
| `full_system.launch.py` | All | Complete system startup |

### Platform-Specific Configuration

Each launch file supports platform selection via `platform` argument:

```bash
# For Jetson Orin
ros2 launch system_bringup 2_perception.launch.py platform:=orin

# For Raspberry Pi (default)
ros2 launch system_bringup 2_perception.launch.py platform:=pi

# Simulation mode
ros2 launch system_bringup 1_hardware.launch.py simulation_mode:=true
```

Configuration files are located at: `src/[layer]/config/[platform]/`

### Testing

Comprehensive test suite available:
```bash
# Run all layer tests
./test_system_bringup.sh

# Test in Docker
./test_in_docker.sh

# Check specific layer
ros2 launch system_bringup 3_intelligence.launch.py
```

Results documented in: `TEST_RESULTS.md`

## Remote Debugging with MCP Servers

### Available MCP Tools

**Docker MCP** (requires Python 3.12):
- Installed at: `/home/wl/.local/bin/docker-mcp`
- Container management, image operations, network inspection
- Usage: Natural language queries about Docker containers

**SSH MCP** (Raspberry Pi at 192.168.1.54):
- Target: `wl@192.168.1.54` (password: 123456)
- Capabilities: Remote command execution, file upload/download
- Use for: Remote debugging on Raspberry Pi hardware

**Jenkins MCP** (CI/CD Integration):
- Server: http://192.168.1.59:8081
- Functions: Build monitoring, log analysis, test result retrieval
- Automatically configured for this project

### Python Environment Notes

**Critical**: ROS2 Humble requires Python 3.10
- System default: Python 3.10.12 (DO NOT change)
- Docker MCP uses: Python 3.12.12 (parallel installation)
- Both versions coexist safely via `/usr/bin/python3.12`

## Deployment (Systemd Services)

### Service Deployment Script

**Location**: `iiri-ros/deploy_systemd_services.sh`

**Key Features**:
- Automatic symlink management to `/home/wl/autorun/iiri-ros`
- Version-specific deployment with rollback capability
- Systemd service installation for `iiri-qr.service` and `iiri-ros.service`

**Usage**:
```bash
sudo ./iiri-ros/deploy_systemd_services.sh install   # Install and enable
sudo ./iiri-ros/deploy_systemd_services.sh start     # Start services
sudo ./iiri-ros/deploy_systemd_services.sh status    # Check status
sudo ./iiri-ros/deploy_systemd_services.sh stop      # Stop services
```

**Symlink Strategy**:
- Services use fixed path: `/home/wl/autorun/iiri-ros`
- Symlink points to version-specific deployment
- Allows easy version switching without service reconfiguration

## Jenkins CI/CD Integration

### Automated Build Pipeline

**Configuration**: `Jenkinsfile`
- Triggers: Every 2 hours (H */2 * * *)
- Docker environment: Uses project's Docker images
- Build artifacts: Archived to `/var/jenkins_home/workspace/iiri-layered-build-ci/build_x86_shared/install/`

### Build Process

1. **Code Import**: Uses vcstool to import all layer repositories
2. **Layered Build**: Compiles each layer sequentially with dependency chaining
3. **Version Generation**: Automatically generates version info with Git commit hash
4. **Artifact Archiving**: Stores build results for deployment

### Monitoring

Use Jenkins MCP to check build status:
```bash
# In conversation with AI assistant:
"Check the latest Jenkins build status"
"Show me the build logs for the last failed build"
"What tests failed in build #123?"
```

## Authorship and Attribution

**All documentation and code in this project is authored by: 唐文浩**

### Git Commit Format

```
<type>(<scope>): <subject>

<body>

📝 作者：唐文浩

Co-Authored-By: 唐文浩 <twh@example.com>
```

### Documentation Format

```markdown
---
**作者**: 唐文浩
**日期**: 2025-10-11
**版本**: v1.0.0
```

**Note**: Historical commits (before 2025-10-11) may contain different attribution, but all current and future work uses the above format.

## Recent Improvements

### 2025-10-21: ROS2 日志系统优化和调试增强

**主仓库 Commit**: `255170c` - "chore: 更新子模块指针和部署配置"

**子模块 Commits**:
- **core_layer** (`24c7b89`): "feat(system_bringup): 禁用 rcl_logging_spdlog 的外部库日志"
- **hardware_layer** (`969ff6c`): "feat(motion_control): 增强 SetRunState 日志输出"
- **application_layer** (`d32309c`, `f20cb5b`, `1f3a6a4`): 多项改进

#### 问题背景

在远程环境 (192.168.1.54) 上运行 ROS2 系统时，发现日志管理存在以下问题：
1. 每个节点会生成空的 rcl_logging_spdlog 日志文件（0 字节），影响日志判断
2. 实际有用的日志输出在 `/tmp/ros2_logs/YYYY-MM-DD-*/node-N-stderr.log` 中
3. 空日志文件干扰运维人员快速定位问题
4. 缺乏详细的调试日志，难以追踪状态机和远程控制问题

#### 解决方案

##### 1. 禁用 rcl_logging_spdlog 外部库日志 (core_layer)

**修改文件**:
- `src/system_bringup/launch/layer/1_hardware.launch.py`
- `src/system_bringup/launch/layer/2_perception.launch.py`
- `src/system_bringup/launch/layer/3_intelligence.launch.py`
- `src/system_bringup/launch/layer/4_application.launch.py`

**实施细节**:
为所有 13 个节点添加 `--disable-external-lib-logs` 参数：

```python
# 硬件层（2个节点）
motion_control_node = Node(
    package='motion_control',
    executable='motion_control_node',
    name='motion_control',
    output='own_log',
    arguments=['--ros-args', '--disable-external-lib-logs'],  # 新增
    parameters=motion_params
)
```

涉及节点：
- 硬件层：motion_control, robot_base
- 感知层：camera_ptz, speaker, tts, speech_recognition
- 智能层：bt_manager, xiaozhi, smart_follow
- 应用层：dev_server, remote_ctrl, record, key_control

**效果对比**:

修改前：
```bash
/tmp/ros2_logs/
├── bt_manager_node_8539_xxx.log        (0 字节 ❌)
├── camera_ptz_node_8530_xxx.log        (0 字节 ❌)
├── dev_server_node_8563_xxx.log        (0 字节 ❌)
├── ... (11个空文件)
└── 2025-10-21-10-02-36-xxx/
    ├── robot_base_node-2-stderr.log    (有内容 ✅)
    └── launch.log                       (有内容 ✅)
```

修改后：
```bash
/tmp/ros2_logs/
└── 2025-10-21-10-16-33-xxx/
    ├── robot_base_node-2-stderr.log    (1.6K ✅)
    ├── bt_manager_node-7-stderr.log    (1.7K ✅)
    ├── launch.log                       (1.8K ✅)
    └── ... (只有有用的日志文件)
```

##### 2. 增强运动控制调试日志 (hardware_layer)

**修改文件**: `src/motion_control/src/qr.cpp`

**改进内容**:
- 添加状态名称显示（STAND/WALK/LIE）
- 记录状态切换前后对比
- 输出 SDK SetRunState 调用结果
- 所有日志添加 `[MOTION_CTRL]` 前缀

**代码示例**:
```cpp
RCLCPP_INFO(this->get_logger(),
    "[MOTION_CTRL] ROS2 received run_state command: %s (value=%d)",
    state_name, msg->value);

if (tmp != runState_) {
    RCLCPP_INFO(this->get_logger(),
        "[MOTION_CTRL] State change detected, calling SDK SetRunState...");
    auto ret = quadruped_->SetRunState(tmp);
    if (ret == iiri::RetState::ok) {
        RCLCPP_INFO(this->get_logger(),
            "[MOTION_CTRL] SDK SetRunState SUCCESS: %s", state_name);
    } else {
        RCLCPP_ERROR(this->get_logger(),
            "[MOTION_CTRL] SDK SetRunState FAILED: %s", state_name);
    }
}
```

##### 3. 增强远程控制调试日志 (application_layer)

**修改文件**: `src/remote_ctrl/src/remote_ctrl_node.cpp`

**改进内容**:
- 记录接收到的 WebSocket 消息（path 和 payload）
- 显示发布的 ROS2 run_state 消息及其值
- 警告未知的 WebSocket 路径
- 所有日志添加 `[REMOTE_CTRL]` 前缀

**代码示例**:
```cpp
RCLCPP_INFO(this->get_logger(),
    "[REMOTE_CTRL] WS received: path=%s, payload=%s",
    path.c_str(), payload.substr(0, 200).c_str());

RCLCPP_INFO(this->get_logger(),
    "[REMOTE_CTRL] ROS2 published run_state=%s (value=%d)",
    json_param.at("run_state").get<std::string>().c_str(),
    run_state.value);
```

##### 4. 架构重构：移除 application_layer/bringup (application_layer)

**背景**:
原 bringup 包混合了所有层级的启动逻辑，难以维护。新架构已将启动逻辑迁移到 core_layer/system_bringup。

**删除文件**:
- `src/bringup/CMakeLists.txt`
- `src/bringup/package.xml`
- `src/bringup/launch/*.launch.py` (14个文件)

**新的启动方式**:
```bash
# 使用 core_layer 的分层 launch 文件
ros2 launch system_bringup 1_hardware.launch.py
ros2 launch system_bringup 2_perception.launch.py
ros2 launch system_bringup 3_intelligence.launch.py
ros2 launch system_bringup 4_application.launch.py
ros2 launch system_bringup qr_orin.launch.py  # 完整系统
```

##### 5. dev_server Vue3 前端集成 (application_layer)

**新增功能**:

1. **HTTP API 服务** (`http_api.cpp/hpp`)
   - 集成 CrowCpp 轻量级 HTTP 框架
   - 支持 RESTful 接口
   - 静态文件服务（/static/）

2. **Vue3 前端应用** (`frontend_src/`)
   - Vue3 + Vite + TypeScript 技术栈
   - 组件化设计：
     * ConnectionPanel: WebSocket 连接管理
     * ControlPanel: 机器人控制面板
     * SystemInfo: 系统信息显示
     * VideoPlayer: 视频流播放
   - Pinia 状态管理
   - 响应式设计，支持移动端

3. **构建和部署**
   - `build_frontend.sh`: 自动构建脚本
   - 构建产物输出到 `static/` 目录
   - `.gitignore`: 忽略 node_modules 和构建产物
   - `INTEGRATION_GUIDE.md`: 集成指南文档

##### 6. 主仓库配置优化

**新增文件**:
- `deploy_dev_server_frontend.sh`: 前端部署脚本

**修改文件**:
- `iiri-ros/start_ros2_iiri_start.sh`:
  ```bash
  export RCUTILS_COLORIZED_OUTPUT=0               # 禁用彩色输出
  export RCUTILS_LOGGING_USE_STDOUT=0             # 日志写入文件
  export RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR=1  # 优化刷新
  ```

- `iiri-ros/iiri-ros.service`: 更新 systemd 服务配置

#### 技术原理

**rcl_logging_spdlog 行为**:
- ROS2 默认使用 rcl_logging_spdlog 为每个节点创建单独的日志文件
- 这些文件通过 ROS logging API（RCLCPP_INFO等）写入
- 如果节点主要使用 std::cout/std::cerr，这些文件会一直是空的
- `--disable-external-lib-logs` 参数禁用此行为

**日志输出位置**:
- `output='own_log'` 将日志输出到 launch 日志目录
- 每个节点生成独立的 `-stdout.log` 和 `-stderr.log`
- 位于 `/tmp/ros2_logs/YYYY-MM-DD-HH-MM-SS-xxx-hostname-pid/` 目录

#### 部署流程

```bash
# 1. 构建
./build_layered.sh -c arm core_layer
./build_layered.sh arm application_layer

# 2. 打包
./deploy_package.sh arm deploy_packages

# 3. 上传到远程服务器
scp deploy_packages/iiri-ros-arm-*.tar.gz wl@192.168.1.54:/home/wl/autorun/

# 4. 远程部署
ssh wl@192.168.1.54
cd /home/wl/autorun
tar -xzf iiri-ros-arm-*.tar.gz
ln -snf iiri-ros-arm-<version> iiri-ros
sudo systemctl restart iiri-ros.service
```

#### 验证方法

```bash
# 查看所有日志
ls -lh /tmp/ros2_logs/2025-*/

# 查看特定节点的错误输出
tail -f /tmp/ros2_logs/2025-*/robot_base_node-*-stderr.log

# 查看 launch 总日志
tail -f /tmp/ros2_logs/2025-*/launch.log

# 使用日志前缀过滤
journalctl -u iiri-ros.service | grep '\[MOTION_CTRL\]'
journalctl -u iiri-ros.service | grep '\[REMOTE_CTRL\]'
```

#### 影响和收益

**日志管理**:
- ✅ 清爽整洁：没有干扰的空文件
- ✅ 易于判断：一眼就能看到哪些日志有内容
- ✅ 性能优化：减少不必要的文件 I/O 操作
- ✅ 保持功能：launch 日志和节点 stderr/stdout 都正常输出

**调试能力**:
- ✅ 完整的状态机追踪：从 WebSocket → ROS2 → SDK 的完整链路
- ✅ 日志前缀过滤：快速定位特定模块的问题
- ✅ 详细的错误信息：明确指出失败的具体环节

**用户体验**:
- ✅ Web 控制界面：Vue3 响应式设计，支持移动端
- ✅ 实时监控：WebSocket 连接状态、系统信息显示
- ✅ 便捷操作：无需 SSH，浏览器即可控制机器人

#### 相关文档

- [ROS2 Logging Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Logging.html)
- [rcl_logging_spdlog 配置](https://github.com/ros2/rcl_logging)
- [system_bringup 测试报告](docs/testing/test-reports/)

### 2025-10-14: sync.sh Push Command Enhancement

**Commit**: `ef7b7c9` - "feat: 增强 sync.sh，添加智能 push 命令"

#### Background
Previously, pushing changes to multiple layer repositories required manual steps:
1. `vcs push src/` - Push all subrepositories
2. `git add src/` - Stage submodule pointer changes
3. `git commit` - Commit pointer updates
4. `git push` - Push main repository

This was tedious and error-prone, especially when forgetting to update submodule pointers after pushing layer changes.

#### New Features

**1. Interactive Push Command**
```bash
./sync.sh push
```
- Automatically pushes all subrepositories using `vcs push`
- Detects submodule pointer changes in main repository
- Interactively asks user whether to update main repository pointers
- Shows list of changed submodules
- Provides manual operation hints if user chooses to skip

**2. Automated Push Command**
```bash
./sync.sh push --update-main
```
- Pushes all subrepositories
- Automatically updates main repository submodule pointers
- Generates standardized commit message with changed module list
- Includes author attribution (唐文浩)
- Suitable for CI/CD automation workflows

#### Implementation Details

**New Functions**:
- `push_changes()` (sync.sh:157-223): Main push logic with submodule pointer detection
- `update_main_repo()` (sync.sh:225-261): Automatic main repository update

**User Workflow**:
```bash
# Scenario 1: Daily development
cd src/intelligence_layer
git commit -m "feat: optimize navigation algorithm"
cd ../..
./sync.sh push                    # Interactive confirmation

# Scenario 2: CI/CD automation
./sync.sh push --update-main      # One command does everything
```

#### Benefits
- **Efficiency**: Reduces 4 manual steps to 1 command
- **Safety**: Automatic detection prevents forgotten pointer updates
- **Consistency**: Standardized commit messages with author attribution
- **Flexibility**: Interactive and automated modes for different use cases

#### Related Changes
- Updated `README.md` with detailed `sync.sh push` documentation
- Added usage examples and scenarios
- Updated help text and command descriptions

#### Files Modified
- `sync.sh`: Added push command functionality
- `README.md`: Added push command documentation section

### 2025-10-14: bt_manager Behavior Tree Node Optimization

**Commits**:
- `8abbf30` - "refactor: 优化 BtRemoteCtrl 和 BtSmartFollow 超时和容错机制"
- `18521f1` - "fix: 修复 BtSmartFollow 的第二个构造函数未初始化 node_ 的问题"

#### Background
The behavior tree nodes `BtRemoteCtrl` and `BtSmartFollow` had several design issues:
- Used `SyncActionNode` (blocking main thread)
- 30-second timeout (mismatched with 3-second detection cycles)
- Shutdown on service unavailable (no fault tolerance)

#### Changes Made

**1. Node Type Change**
- Changed from `BT::SyncActionNode` to `BT::ThreadedAction`
- Enables non-blocking asynchronous execution
- Prevents blocking the behavior tree main thread

**2. Timeout Optimization**
- Reduced timeout from 30 seconds to 3 seconds
- Matches the detection cycle of remote_ctrl service
- Faster failure recovery and state transitions

**3. Fault Tolerance**
- Added `service_available_` flag
- Constructor now warns instead of shutting down on service unavailable
- Node returns `FAILURE` gracefully when service unavailable
- System continues running even if optional services are down

#### Technical Details

**Modified Files**:
- `src/intelligence_layer/src/bt_manager/include/bt_remote_ctrl.hpp`
- `src/intelligence_layer/src/bt_manager/src/bt_remote_ctrl.cpp`
- `src/intelligence_layer/src/bt_manager/include/bt_smart_follow.hpp`
- `src/intelligence_layer/src/bt_manager/src/bt_smart_follow.cpp`

**Key Changes**:
```cpp
// Before: Blocking synchronous node
class BtRemoteCtrl : public BT::SyncActionNode

// After: Non-blocking threaded node
class BtRemoteCtrl : public BT::ThreadedAction
{
    bool service_available_ = true;  // Fault tolerance flag
};

// Timeout reduction
ret_future.wait_for(std::chrono::seconds(3));  // Was 30s

// Graceful degradation
if (!service_available_ || !cli_trigger_->service_is_ready()) {
    return BT::NodeStatus::FAILURE;  // Don't crash
}
```

#### Impact
- **Responsiveness**: System reacts faster to remote control input
- **Robustness**: Optional services don't crash the system
- **User Experience**: Smoother behavior tree execution
- **Debugging**: Clearer log messages for service status

### 2025-10-11: Previous Improvements

#### 1. System Bringup Refactoring
- Moved bringup from application_layer to core_layer
- Reduced compilation requirements from 5 layers to 3 layers
- Added platform-specific configuration support
- Created comprehensive test suite

#### 2. CI/CD Enhancement
- Configured Jenkins automated build pipeline
- Implemented artifact archiving strategy
- Optimized build parallelism (single job to avoid memory issues)
- Added version tracking with Git commit hash

#### 3. Deployment Improvements
- Created intelligent symlink management in deployment script
- Added rollback capability through version-specific paths
- Improved error handling and user feedback

#### 4. Development Tools
- Integrated Docker MCP for container management
- Added SSH MCP for Raspberry Pi remote debugging
- Configured Jenkins MCP for CI/CD monitoring
- Established parallel Python 3.10/3.12 environment

#### 5. Documentation Updates
- Unified all authorship to 唐文浩
- Updated configuration and deployment guides
- Added troubleshooting sections
- Created quick reference materials

## Documentation Structure

The project documentation is organized into themed directories for easy navigation:

### 📚 Main Documentation (`docs/`)

All project documentation is centralized in the `docs/` directory with the following structure:

- **[docs/README.md](docs/README.md)** - Documentation navigation center
- **getting-started/** - Quick start guides for new users
- **architecture/** - System design and architecture documentation
  - [layered-design.md](docs/architecture/layered-design.md) - Five-layer architecture
  - [path-tracker.md](docs/architecture/path-tracker.md) - Path tracker component design
- **development/** - Development guides and best practices
  - [build-system.md](docs/development/build-system.md) - Build system explained
  - [vcstool-guide.md](docs/development/vcstool-guide.md) - vcstool commands and usage
  - [vcstool-team-guide.md](docs/development/vcstool-team-guide.md) - Team collaboration practices
- **deployment/** - Production deployment guides
  - [deployment-guide.md](docs/deployment/deployment-guide.md) - Complete deployment workflow
  - [systemd-services.md](docs/deployment/systemd-services.md) - Service configuration
- **ci-cd/** - CI/CD configuration and usage
  - [jenkins-setup.md](docs/ci-cd/jenkins-setup.md) - Jenkins configuration
  - [jenkins-usage.md](docs/ci-cd/jenkins-usage.md) - Daily Jenkins operations
- **infrastructure/** - Infrastructure services
  - [harbor-registry.md](docs/infrastructure/harbor-registry.md) - Docker image registry
- **testing/** - Testing strategies and reports
  - [testing-guide.md](docs/testing/testing-guide.md) - Testing methodology
  - test-reports/ - Historical test reports
- **reference/** - API references and troubleshooting
  - [jenkins-troubleshooting.md](docs/reference/jenkins-troubleshooting.md) - Jenkins issues
  - [troubleshooting.md](docs/reference/troubleshooting.md) - Common issues
  - [faq.md](docs/reference/faq.md) - Frequently asked questions

### 📖 Root Documentation

- **[README.md](README.md)** - Project overview and quick links
- **[CLAUDE.md](CLAUDE.md)** - AI assistant development guide (this file)
- **[CHANGELOG.md](CHANGELOG.md)** - Version history and changes

### 🔧 Tool Directories

- **jenkins/** - Jenkins CI scripts and tools ([README](jenkins/README.md))
- **harbor/** - Harbor registry scripts and tools ([README](harbor/README.md))
- **script/** - Build and utility scripts

### 📦 Archive

- **archive/** - Historical documents for reference only

## Recent Session: 2025-10-22 Frontend Fix & build_frontend.sh Improvement

### Background - Frontend Deployment Issue

During the velocity command fix debugging session, discovered that the web frontend was not loading properly (blank page at http://192.168.1.54:8080/). Investigation revealed:

**Problem**: The frontend static files (dist/) were not being completely copied to the static/ directory
- Local static/ directory had: index.html, favicon.ico, but missing JS/CSS assets
- Result: HTML loaded but JavaScript files returned 404 errors
- Root cause: build_frontend.sh script was incomplete or running before npm build finished

**Root Cause Analysis**:
The build_frontend.sh script had a flawed copy logic:
1. Used `find ... -delete` to clear static/ but didn't guarantee complete removal
2. Didn't verify file counts before/after copying
3. Silent failures - script continued even if copy failed
4. No validation that critical files were actually copied

### Solution Implemented

**Commit**: `e217d8a - improve(build_frontend): 完善静态文件复制和验证逻辑`

#### Enhanced build_frontend.sh Features

```bash
# 1. Source file counting
DIST_FILE_COUNT=$(find "$DIST_DIR" -type f | wc -l)

# 2. Complete directory cleanup
rm -rf "$STATIC_DIR"
mkdir -p "$STATIC_DIR"

# 3. Verbose copy with error checking
cp -rv "$DIST_DIR"/* "$STATIC_DIR/" 2>&1 | head -20
CP_EXIT_CODE=$?

# 4. Destination verification
STATIC_FILE_COUNT=$(find "$STATIC_DIR" -type f | wc -l)

# 5. File count validation
if [ "$STATIC_FILE_COUNT" -lt "$DIST_FILE_COUNT" ]; then
    echo "Error: Files mismatch!"
    exit 1
fi

# 6. Critical file validation
for file in "index.html" "assets/index-*.js" "assets/index-*.css" "favicon.ico"; do
    # Check each file exists
done
```

#### Improvements Made

| Feature | Before | After |
|---------|--------|-------|
| Directory cleanup | `find -delete` (partial) | `rm -rf` (complete) |
| Copy verification | None | Count-based validation |
| Error handling | Silent failures | Immediate exit on error |
| File validation | No | Verify critical assets |
| Log output | Minimal | Detailed with color |

#### Key Changes

**Step-by-step validation**:
1. Count files in dist/ (source)
2. Completely delete static/ directory
3. Recreate empty static/ directory
4. Copy all files from dist/ with verbose output
5. Count files in static/ (destination)
6. Compare source vs destination file counts
7. Validate critical files exist (index.html, JS, CSS, favicon)
8. Display summary statistics

**Error handling**:
- Exit code checking on each operation
- File count mismatch detection
- Missing critical file detection
- Clear error messages with color coding

#### Example Output

```
[3/3] 复制构建文件到 static 目录...
Source (dist): 7 文件
清空 /static 目录...
复制文件中...
'dist/assets' -> 'static/assets'
[...more files...]

✓ 文件复制完成

构建结果统计:
  - 源文件数量 (dist):   7
  - 目标文件数量 (static): 7
  - 总大小:           1.4M

关键文件验证:
  ✓ index.html
  ✓ index-iYiMz3Es.js (1009K)
  ✓ index-u9KE-Qh3.css (347K)
  ✓ favicon.ico
```

### Files Modified

**Commit**: `e217d8a` in application_layer
**File**: `src/dev_server/frontend_src/build_frontend.sh`
**Changes**: +65 lines, -18 lines

- Replaced lines 64-92 with improved copy logic
- Added validation and error checking
- Enhanced logging and progress reporting

### Testing & Validation

After deploying with improved build_frontend.sh:
1. Manually verified all JS/CSS files present in static/
2. Confirmed file counts match (7 files in both dist/ and static/)
3. Web interface loaded successfully at http://192.168.1.54:8080/
4. All UI components rendered without 404 errors

### Impact

**Prevents future occurrences**:
- Script now guarantees complete file copy or fails loudly
- File count validation catches partial copies
- Critical file verification ensures no essential assets missing
- Clear error messages help diagnose copy failures

**Operational benefits**:
- Faster failure detection in build pipeline
- Better visibility into what files were copied
- Prevents deployment of incomplete frontend packages

---

## Previous Session: 2025-10-22 Debug Session & Git Batch Push

### Background
In the previous context session, the developer identified a critical bug where `SetRunState()` was returning SUCCESS despite all 12 motors being in error/timeout state. This session focused on:
1. Continuing from the previous implementation (hardware health checks in StateManager)
2. Understanding the state machine transition logic
3. Pushing all accumulated changes to remote repositories

### Changes Made

#### 1. Hardware Layer Enhancements (hardware_layer: 258eae4)

**Commit**: `258eae4 - feat(motion_control): 增强 SetRunState 调用的日志记录和错误处理`

**Modified Files**:
- `src/motion_control/src/qr.cpp`
- `src/motion_control/third-party/iiri-sdk/CMakeLists.txt`
- `src/motion_control/third-party/iiri-sdk/src/apiDevice.cpp/hpp`
- `src/motion_control/third-party/iiri-sdk/src/sdkProtocolClient.cpp/hpp`
- `src/motion_control/third-party/iiri-sdk/src/udpClient.cpp/hpp`

**Key Changes in qr.cpp**:

1. **Enhanced State Query Logic** (lines 213-238):
```cpp
// Query actual state from QR instead of relying on cached state
auto [get_ret, actual_state] = quadruped_->GetRunState();

std::string actual_state_name = "UNKNOWN";
if (get_ret == iiri::RetState::ok) {
    switch (actual_state) {
        case iiri::qr::RunState::stand:
            actual_state_name = "STAND";
            break;
        case iiri::qr::RunState::walk:
            actual_state_name = "WALK";
            break;
        case iiri::qr::RunState::lie:
            actual_state_name = "LIE";
            break;
        default:
            actual_state_name = "UNKNOWN";
    }
    RCLCPP_DEBUG(this->get_logger(), "[MOTION_CTRL] Current QR state: %s, Requested: %s",
                 actual_state_name.c_str(), state_name);
} else {
    RCLCPP_WARN(this->get_logger(), "[MOTION_CTRL] Failed to query current state: RetState=%d",
                static_cast<int>(get_ret));
}
```

2. **State Transition Logging** (lines 240-250):
```cpp
// Compare with desired state and execute if different
if (tmp != actual_state) {
    RCLCPP_INFO(this->get_logger(), "[MOTION_CTRL] State change detected: %s -> %s, calling SDK SetRunState...",
               actual_state_name.c_str(), state_name);
    auto ret = quadruped_->SetRunState(tmp);
    if (ret == iiri::RetState::ok) {
        runState_ = tmp;  // Update cache for subscribers
        RCLCPP_INFO(this->get_logger(), "[MOTION_CTRL] SDK SetRunState SUCCESS: %s", state_name);
    }
```

3. **Comprehensive Error Code Mapping** (lines 254-278):
```cpp
std::string error_desc = "UNKNOWN";
switch (ret) {
    case iiri::RetState::netErr:
        error_desc = "netErr (网络错误)";
        break;
    case iiri::RetState::outRange:
        error_desc = "outRange (超范围)";
        break;
    case iiri::RetState::timeout:
        error_desc = "timeout (超时)";
        break;
    case iiri::RetState::noSupport:
        error_desc = "noSupport (不支持)";
        break;
    case iiri::RetState::parseErr:
        error_desc = "parseErr (解析错误)";
        break;
    case iiri::RetState::busy:
        error_desc = "busy (设备忙)";
        break;
    case iiri::RetState::interrupt:
        error_desc = "interrupt (任务中断)";
        break;
    case iiri::RetState::error:
        error_desc = "error (设备状态错误/电机故障)";
        break;
    default:
        error_desc = "other error";
}
RCLCPP_ERROR(this->get_logger(), "[MOTION_CTRL] SDK SetRunState FAILED: %s -> %s (RetState=%d)",
            state_name, error_desc.c_str(), static_cast<int>(ret));
```

4. **Optimized Logging Levels** (line 284):
```cpp
// Changed from INFO to DEBUG for no-change case
RCLCPP_DEBUG(this->get_logger(), "[MOTION_CTRL] Already in %s state, no change needed", state_name);
```

**Changes in iiri-sdk**:
- CMakeLists.txt: Simplified build configuration
- apiDevice.cpp/hpp: Code cleanup and optimization
- sdkProtocolClient.cpp/hpp: Refactored protocol handling (44 insertions, 44 deletions)
- udpClient.cpp/hpp: Optimized UDP communication logic (32 insertions, 32 deletions)

**Statistics**: 8 files changed, 148 insertions(+), 115 deletions(-)

#### 2. Core Layer Pointer Update (core_layer: 28122ad)

**Commit**: `28122ad - refactor(system_bringup): 更新 motion_control 节点可执行文件名`

This was pushed to origin/main without new commits (already pushed in previous session).

#### 3. Main Repository Changes (master: c118d41)

**Commit**: `c118d41 - refactor: 恢复 setup-and-build.sh 到根目录并更新子模块指针`

**Modified Files**:
- `archive/README.md` - Updated documentation
- `archive/setup-and-build.sh` → `setup-and-build.sh` (moved from archive to root)
- `src/hardware_layer` - Updated submodule pointer to 258eae4
- `src/core_layer` - Updated submodule pointer to 28122ad

**Changes**:
- Removed setup-and-build.sh from archive documentation (9 lines deleted)
- Restored setup-and-build.sh to root directory for easier access
- Updated archive/README.md to reflect the change

**Statistics**: 3 files changed, 1 insertion(+), 10 deletions(-)

### Push Summary

| Repository | Commit | Branch | Status |
|---|---|---|---|
| hardware_layer | 258eae4 | main | ✅ Pushed to origin/main |
| core_layer | 28122ad | main | ✅ Pushed to origin/main |
| perception_layer | 522a395 | main | ✅ Already synced |
| intelligence_layer | 18521f1 | main | ✅ Already synced |
| application_layer | 1f3a6a4 | main | ✅ Already synced |
| Main repo | c118d41 | master | ✅ Pushed to origin/master |

### Technical Insights

#### State Machine Transition Checking Code Pattern
The refactored code uses a clean separation of concerns:

```cpp
// Step 1: Check state machine rules
bool transitionAllowed = false;
for (auto var : canSwitch_) {
    if ((var.from == curState_) && (var.to == query_)) {
        transitionAllowed = true;
        break;
    }
}

// Step 2: Check robot global state health
if (!CheckRobotStateOk()) {
    return RetState::error;
}

// Step 3: Check motor health
if (!CheckMotorsHealthy()) {
    return RetState::error;
}

// Only if all checks pass
resp = RetState::ok;
```

This three-layer validation prevents invalid state transitions due to hardware failures.

#### Logging Improvements Impact
The enhanced logging in motion_control node provides:
- **Traceability**: Complete ROS2 → SDK → QR state transition path
- **Debugging**: Detailed error codes help identify failure root causes
- **Monitoring**: [MOTION_CTRL] prefix enables easy filtering in systemd journals
- **Performance**: Log levels appropriately tuned (INFO for changes, DEBUG for stable states)

### Working Directory Status After Push

All repositories are synchronized with remote:
```
✅ Main repo: branch master synced with origin/master
✅ core_layer: branch main synced with origin/main
✅ hardware_layer: branch main synced with origin/main
✅ perception_layer: branch main synced with origin/main
✅ intelligence_layer: branch main synced with origin/main
✅ application_layer: branch main synced with origin/main
```

Remaining untracked files (not pushed - temporary/backup files):
- `env.tmp` - Temporary environment variables
- `iiri-sdk.backup_websocket/` - SDK backup directory
- `*.backup` files - Backup copies

### Related Issues Addressed

1. **Bug Fix**: `SetRunState()` success validation now includes hardware health checks (from previous session)
2. **Logging Enhancement**: Detailed state transition tracking for remote debugging
3. **Code Quality**: SDK optimization and cleanup
4. **File Organization**: Restored setup-and-build.sh to root for better discoverability

## Additional Resources

- User tools: `~/.claude/QUICK_REF.md` - Common commands and MCP setup
- Configuration: `~/.claude/config.yaml` - Claude assistant configuration
- External: [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
