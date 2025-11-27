# 异常处理流程 (Exception Handling)

## 1. 异常分级

系统将异常分为三个等级，根据严重程度采取不同的响应策略：

| 等级 | 描述 | 示例 | 处理策略 |
| :--- | :--- | :--- | :--- |
| **Critical** | 严重故障，可能导致硬件损坏或人身伤害 | 电机过热保护、通信总线断开、IMU数据丢失 | 立即切断动力输出，进入 `error` 状态，锁定所有制动器 |
| **Error** | 功能性故障，系统无法继续当前任务 | 运动解算发散、目标位置不可达、传感器数据异常 | 停止当前运动，保持当前姿态，尝试降级运行或复位 |
| **Warning** | 警告，系统仍可运行但需注意 | 电池电量低、CPU负载过高、偶尔的丢包 | 记录日志，播放语音提示，不中断运行 |

## 2. 处理流程图

```mermaid
graph TD
    Start[Error Detected] --> Log[Log Error (Critical/Error)]
    Log --> CheckSeverity{Severity Level?}
    
    CheckSeverity -- Low (Warning) --> NotifyUser[Play Warning Audio]
    NotifyUser --> Continue[Continue Operation]
    
    CheckSeverity -- High (Critical) --> SafeMode[Enter Safe Mode]
    SafeMode --> StopMotors[Stop All Motors]
    StopMotors --> Shutdown[System Shutdown/Restart]
    
    subgraph "Safe Mode Actions"
        StopMotors
        LockBrakes[Engage Brakes]
    end
```

## 3. 常见故障排查

### 3.1 电机通信超时
- **现象**: 机器人突然失去动力，日志显示 "CAN timeout"。
- **原因**: 
  1. CAN总线接线松动。
  2. 某个节点故障导致总线堵塞。
  3. 系统负载过高导致调度延迟。
- **排查**:
  1. 检查硬件连接。
  2. 使用 `candump` 工具抓包分析。
  3. 检查 `top` 命令下的 CPU 占用率。

### 3.2 姿态解算发散
- **现象**: 机器人姿态剧烈抖动或倾倒。
- **原因**:
  1. IMU校准参数失效。
  2. 强磁场干扰。
  3. 运动加速度过大超出量程。
- **排查**:
  1. 重新进行 IMU 校准。
  2. 检查周围环境。
  3. 降低控制增益 `kp` / `kd`。

### 3.3 语音识别无响应
- **现象**: 对机器人喊话无反应。
- **原因**:
  1. 麦克风被遮挡或损坏。
  2. 离线模型文件丢失。
  3. 声卡设备号配置错误。
- **排查**:
  1. 使用 `arecord` 测试录音功能。
  2. 检查配置文件中的 `audio_device` 参数。
  3. 确认模型路径正确。
