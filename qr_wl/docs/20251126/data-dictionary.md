# 数据字典 (Data Dictionary)

## 1. 核心数据结构

### 1.1 RobotState (系统状态)
**类型**: Enum (枚举)
**描述**: 定义机器人的全局生命周期状态。

| 值 | 描述 |
| :--- | :--- |
| `initing` | 系统正在初始化，禁止电机输出。 |
| `standby` | 待机状态，电机上电但无动作，等待指令。 |
| `running` | 正常运行状态，允许执行运动控制算法。 |
| `error` | 故障状态，系统检测到严重错误，触发保护机制。 |
| `jointCtrl` | 关节控制模式，用于底层调试或校准。 |

### 1.2 MotorCmd (电机指令)
**类型**: Struct (结构体)
**描述**: 发送给底层电机的控制参数。

| 字段 | 类型 | 单位 | 描述 |
| :--- | :--- | :--- | :--- |
| `q` | float | rad | 目标关节角度 |
| `dq` | float | rad/s | 目标关节角速度 |
| `kp` | float | - | 位置刚度系数 |
| `kd` | float | - | 速度阻尼系数 |
| `tau` | float | Nm | 前馈扭矩 |

### 1.3 MotorData (电机反馈)
**类型**: Struct (结构体)
**描述**: 从电机回传的实时状态数据。

| 字段 | 类型 | 单位 | 描述 |
| :--- | :--- | :--- | :--- |
| `q` | float | rad | 当前关节角度 |
| `dq` | float | rad/s | 当前关节角速度 |
| `tau` | float | Nm | 当前输出扭矩 |
| `temp` | int | °C | 电机线圈温度 |
| `error` | int | - | 错误码 (过热、过流等) |

### 1.4 ImuData (IMU数据)
**类型**: Struct (结构体)
**描述**: 惯性测量单元的传感器数据。

| 字段 | 类型 | 单位 | 描述 |
| :--- | :--- | :--- | :--- |
| `acc` | Vector3 | m/s² | 三轴加速度 |
| `gyro` | Vector3 | rad/s | 三轴角速度 |
| `quat` | Vector4 | - | 姿态四元数 (w, x, y, z) |
| `rpy` | Vector3 | rad | 欧拉角 (Roll, Pitch, Yaw) |

### 1.5 BoardInfo (板卡信息)
**类型**: Struct (结构体)
**描述**: 硬件板卡的身份和版本信息。

| 字段 | 类型 | 描述 |
| :--- | :--- | :--- |
| `canId` | uint8_t | CAN总线ID |
| `version` | uint32_t | 固件版本号 |
| `type` | Enum | 板卡类型 (Driver, BMS, Sensor) |

## 2. 消息主题 (Topics)

系统内部通过消息总线通信，以下是主要的消息主题：

| 主题名称 | 数据类型 | 发布者 | 订阅者 | 描述 |
| :--- | :--- | :--- | :--- | :--- |
| `motor_cmd` | `MotorCmd` | UserNode | CanBridge | 期望的电机控制指令 |
| `motor_state` | `MotorData` | CanBridge | UserNode | 实际的电机状态反馈 |
| `imu_data` | `ImuData` | ImuDrv | UserNode | IMU姿态数据 |
| `voice_cmd` | `string` | OfflineVoice | UserNode | 语音识别结果 |
