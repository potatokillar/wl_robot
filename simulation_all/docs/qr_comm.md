# QR 通信机制概览

- 命名空间：`/qr_gazebo`，用于隔离 QR 机器人在 Gazebo 中的所有话题与控制器
- 启动入口：通过 `gazebo_qr.launch` 加载模型、控制器与状态发布器并完成命名空间重映射
- 控制方式：每个关节对应一个控制器 `robot_joint_controller`，订阅该关节的 `command`，发布该关节的 `state`
- 消息类型：自定义 `robot_msgs/MotorCommand` 与 `robot_msgs/MotorState`

## 启动与命名空间

- 载入世界与模型：`src/simulation_all/launch/gazebo_qr.launch:15–22`、`src/simulation_all/launch/gazebo_qr.launch:30`
- 控制器命名空间：`src/simulation_all/launch/gazebo_qr.launch:39`、`src/simulation_all/launch/gazebo_qr.launch:49`
- 重映射关节状态：`src/simulation_all/launch/gazebo_qr.launch:58–61`
- 控制器配置挂载在命名空间：`src/robots/sz_description/config/robot_control.yaml:1`

## 控制器配置

- 控制器类型：`robot_joint_controller/RobotJointController`
- 示例（左前髋关节）：`src/robots/sz_description/config/robot_control.yaml:8–12`
- 控制器在命名空间 `/qr_gazebo` 下生成对应的话题与参数，例如：
  - 命令话题：`/qr_gazebo/FL_hip_controller/command`
  - 状态话题：`/qr_gazebo/FL_hip_controller/state`

## 控制器代码交互

- 初始化与订阅命令：`src/robot_joint_controller/src/robot_joint_controller.cpp:47–79`
  - 订阅命令话题：`src/robot_joint_controller/src/robot_joint_controller.cpp:74`
  - 状态发布器创建：`src/robot_joint_controller/src/robot_joint_controller.cpp:77`
- 更新循环：`src/robot_joint_controller/src/robot_joint_controller.cpp:115–167`
  - 读取最新命令：`src/robot_joint_controller/src/robot_joint_controller.cpp:117–121`
  - 计算并下发力矩：`src/robot_joint_controller/src/robot_joint_controller.cpp:146–151`
  - 采集与发布状态：`src/robot_joint_controller/src/robot_joint_controller.cpp:155–166`

## 消息类型

- `robot_msgs/MotorCommand`：`src/robot_msgs/msg/MotorCommand.msg:1–5`
  - 字段：`q`（目标位置）、`dq`（目标速度）、`tau`（目标力矩）、`kp`（位置刚度）、`kd`（速度阻尼）
- `robot_msgs/MotorState`：`src/robot_msgs/msg/MotorState.msg:1–5`
  - 字段：`q`（当前位置）、`dq`（当前速度）、`ddq`（当前加速度）、`tauEst`（估计力矩）、`cur`（电流或相关估计）

## 常用话题

- 关节命令（示例：左前髋关节）
  - `topic=/qr_gazebo/FL_hip_controller/command`
  - `type=robot_msgs/MotorCommand`
- 关节状态（示例：左前髋关节）
  - `topic=/qr_gazebo/FL_hip_controller/state`
  - `type=robot_msgs/MotorState`
- 全局关节状态（供 RViz/上层使用）
  - `topic=/qr_gazebo/joint_states`
  - 来源重映射：`src/simulation_all/launch/gazebo_qr.launch:58–61`

## 交互示例

- 发布控制命令（ROS CLI 示例）：
  - `rostopic pub /qr_gazebo/FL_hip_controller/command robot_msgs/MotorCommand "q: 0.5" "dq: 0.0" "tau: 0.0" "kp: 100.0" "kd: 5.0"`
- 订阅状态：
  - `rostopic echo /qr_gazebo/FL_hip_controller/state`

## 设计说明

- 命名空间清晰：通过 `/qr_gazebo` 聚合 QR 模型相关话题，便于隔离与调试
- 控制与反馈解耦：每关节控制器只负责一个 `command` 与一个 `state`，提高可维护性
- 可扩展：切换机器人或配置只需更改 `rname` 与 YAML，复用相同控制器框架
