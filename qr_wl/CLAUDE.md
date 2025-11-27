# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**qr_wl** is a universal robot controller for quadruped robots and robotic arms, running on ARM (Raspberry Pi) and x86 platforms. It uses a Docker-based cross-compilation environment and implements real-time motor control, sensor integration, and network communication.

## Build System

### Cross-Platform Compilation

The project uses Docker for cross-compilation to support both x86 and ARM architectures:

```bash
# Build both x86 and ARM (requires Docker)
./build.sh

# Build specific architecture
./build.sh x86      # Development/testing on PC
./build.sh arm      # Production deployment on Raspberry Pi

# Clean build
./build.sh -c       # Clean all
./build.sh -c x86   # Clean x86 only
./build.sh -c arm   # Clean ARM only
```

**Output locations:**
- x86: `build/x64/output/qr`
- ARM: `build/arm64/output/qr`

### Docker Environment

- Image: `registry.cn-hangzhou.aliyuncs.com/iiri/build_x86_arm_ros1`
- Configuration: `script/env.sh` (sets up user permissions and mounts)
- Entry point: `build.sh` → `script/build.sh` (actual build logic)
- Note: Docker runs with `-i` flag (non-interactive) to support automated builds

### Version Management

The project uses **git-based automatic versioning**:

- **Development branches** (dev, feature/*, bugfix/*): Version = `branch_YYYYMMDD_HHMMSS`
- **Main branch**: Version = latest git tag from remote (e.g., `v2.0.0`)

```bash
# Check version status
./release.sh

# Version info appears in logs on startup:
# [time][info][robot] software version:dev_20251015_103000, sdk version:v1.16.0
# [time][info][robot] git info: dev_a1c8fb3_48e2
```

Git info format: `branch_commitShort_programMD5Last5`

See `VERSION_MANAGEMENT.md` for complete details.

## Architecture Overview

### High-Level Structure

```
qr_wl/
├── app/              # Application layer (business logic)
│   ├── motion/       # Motor control (MIT motors, SPI-CAN, quadruped motors)
│   ├── driver/       # Hardware drivers (IMU, gamepad, sensors)
│   ├── network/      # Network/SDK communication (UDP protocol)
│   ├── robot/        # Robot state management, custom parameters
│   ├── user/         # User-level business logic (quad, arm, device)
│   └── config/       # Configuration and device definitions
├── baseline/         # Core framework (message passing, utilities, types)
├── control/          # Control algorithms (quadruped locomotion, arm kinematics)
├── project/          # Main entry points (real, sim, vrep variants)
├── third-party/      # External dependencies (IIRI SDK, etc.)
└── script/           # Build and deployment scripts
```

### Key Design Patterns

#### 1. Message Passing Architecture

The codebase uses a publish-subscribe message system (`baseline/pub/impl/msgCenter.hpp`):

```cpp
// Publishing data
MsgTrySend("qr::motor_ret", motorData);

// Subscribing to data
auto cmd = MsgTryRecv<msg::qr::motor_cmd>("qr::motor_cmd", this);
```

**Common message channels:**
- `qr::motor_cmd` / `qr::motor_ret` - Quadruped motor command/feedback
- `wheel::motor_cmd` / `wheel::motor_ret` - Wheeled motor command/feedback
- `arm::motor_cmd` / `arm::motor_ret` - Arm motor command/feedback
- `imu::data` - IMU sensor data
- `gamepad::cmd` - Gamepad input

Message types are defined in `baseline/pub/impl/msgType.hpp`.

#### 2. Motor Control Data Flow

**Quadruped motor control chain:**

```
Control Algorithm (control/)
    ↓ (msg::qr::motor_cmd)
QrMotorMitV3::Run() (app/motion/qrMotor/)
    ↓ BlockSend() - applies bias, chirality
Transfer() - converts to CAN protocol
    ↓ TransformMitMotor::CmdToCanV3()
SpiToCanV3::Transfer() (app/motion/hardware/)
    ↓ SPI communication with hardware
SpiToCanV3::GetMessage()
    ↓ CanToDataV3() - parses response
CheckMotorState() - timeout detection (300ms threshold)
    ↓ (msg::qr::motor_ret)
BlockRecv() - removes bias, applies chirality
    ↓
Control Algorithm receives feedback
```

**Critical implementation details:**
- Motor state tracking: `MotorState` enum (ok, disable, enable, error, timeout)
- Timeout detection runs in `CheckMotorState()` - requires CAN parsing results even when invalid
- SPI checksum errors clear `rxCan_` buffer to prevent stale data from falsely indicating motor online
- Each motor has a `qBias` field for zero-position calibration

#### 3. Motor State Lifecycle

```cpp
// Enable sequence (qrMotorMitV3.cpp:Enable())
1. SendEnable() - send enable command via SPI
2. BlockSend() - send zero-torque commands (4 iterations)
3. Read initial positions and check bias initialization
4. CheckInit() - save/load init positions from database

// Disable sequence (qrMotorMitV3.cpp:Disable())
1. BlockSend() - send zero-torque command
2. SendDisable() - send disable command (twice for reliability)
3. ClearSpiData() - clear buffers
4. Update motor states to 'disable'
```

#### 4. Network Protocol Layer

**Data flow to external clients (e.g., qr_chart):**

```
App layer (motor_ret, imu_data, etc.)
    ↓
QuadDebugServer::GetData() (app/user/userQuad/)
    ↓ SaveMotorRet(), SaveImuData() - populate packet
quad_debug_packet_ (includes motor status, position, velocity, torque)
    ↓ Struct2Vector()
SdkProtocolServer (app/network/)
    ↓ UDP port 20333
External client (qr_chart Qt application)
```

**Important:** When adding new fields to network protocol:
1. Update message type in `baseline/pub/impl/msgType.hpp`
2. Populate field in motor layer (e.g., `BlockRecv()`)
3. **Critical:** Copy field in network layer (e.g., `quadDebugServer.cpp::SaveMotorRet()`)
4. Update SDK structures if using IIRI SDK

## Development Workflow

### Local Development

```bash
# Start on development machine (x86)
./start.sh

# Or run directly
cd build/x64/output
./qr config.toml
```

### Deployment to Raspberry Pi

**Target device:** Raspberry Pi at 192.168.1.54
- Username: `wl`
- Password: `123456`
- Working directory: `/home/wl/autorun/iiri-qr`

```bash
# Build ARM version
./build.sh arm

# Deploy to Raspberry Pi
sshpass -p '123456' scp build/arm64/output/qr wl@192.168.1.54:/home/wl/autorun/iiri-qr/qr.new
sshpass -p '123456' ssh wl@192.168.1.54 "cd /home/wl/autorun/iiri-qr && cp qr qr.backup_$(date +%Y%m%d) && mv qr.new qr"

# Restart on Pi
sshpass -p '123456' ssh wl@192.168.1.54 "pkill qr && cd /home/wl/autorun/iiri-qr && ./qr_start.sh qr-linkV2-3.toml &"
```

Or use the automated deployment script:
```bash
./deploy_package_qr.sh
```

### Configuration Files

Configuration files are in TOML format and specify:
- Robot type (quadruped, arm, wheeled)
- Motor configurations (ID, model, chirality, direction)
- Network settings
- Sensor parameters

Example: `config/qr-linkV2-3.toml`

## Common Patterns and Conventions

### Motor Configuration

Each motor has:
- `id`: CAN ID (1-indexed)
- `model`: Motor model (haitai, damiao, etc.) - defines torque/velocity limits
- `chiral`: Handedness flag (affects sign of position/torque/velocity)
- `dir`: Direction multiplier

### Periodic Tasks

Use `PeriodicMemberFunction` for real-time loops:

```cpp
task_ = make_unique<PeriodicMemberFunction<ClassName>>(
    "taskName",
    0.002,  // 2ms period (500Hz)
    this,
    &ClassName::Run,
    true  // real-time priority
);
task_->Start();
```

### Database for Runtime Parameters

`deviceCustomParam.hpp` provides persistent storage:

```cpp
// Read motor init positions
auto positions = GetDevCustomParam().ReadMotorInitPos("qrMotorInitPos");

// Write motor init positions
GetDevCustomParam().WriteMotorInitPos("qrMotorInitPos", positions);

// Init flag (0 = normal, 1 = request re-initialization)
GetDevCustomParam().WriteInitFlag(0);
```

### Robot State Management

Global robot state (`robotState.hpp`):
- `standby` - Initial state
- `running` - Normal operation
- `jointCtrl` - Joint-level control mode
- `error` - Error state

```cpp
if (GetRobotCurState() == RobotState::running) {
    // Execute control
}
```

## Troubleshooting

### Motor Timeout Issues

If motors show "enabled" when disconnected:
1. Check `CheckMotorState()` is being called in `Transfer()` even with invalid data
2. Verify `rxCan_` is cleared on SPI checksum errors (spi2canV3.cpp:172)
3. Confirm timeout threshold (300ms) in `motorMitV3.cpp:34`

### SPI Communication

- SPI checksum uses XOR of 32-bit words plus 0xAA
- Packet format: `0xAA [counts] [data...] [checksum 4 bytes] 0xBB`
- Error threshold: 5 checksum errors in last 10 transfers triggers shutdown

### Build Issues

- **Docker TTY error**: Ensure `script/env.sh` uses `-i` not `-it`
- **Missing dependencies**: Check Docker image is up to date
- **Permission issues**: Docker runs as current user via env.tmp file

## Related Projects

- **qr_chart**: Qt-based visualization tool (separate repository at http://192.168.1.55/hepHephaestushaestus/qr-chart-linux.git)
  - Receives data via UDP on port 20333
  - Displays motor status, real-time plots, and robot state
  - Must be updated when network protocol changes

## Code Documentation

Refer to `claude.md` for detailed development history and specific feature implementation notes (e.g., motor status monitoring feature added 2025-10-15).
