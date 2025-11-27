# QR_WL 项目开发记录

## 2025-10-15 电机状态监控功能实现

### 需求背景

四足机器人系统中有12个电机（4条腿 × 3个关节），机器人启动时需要所有12个电机连接正常才能通过摇杆控制。当电机连接异常时，用户无法控制机器人，但现有的 qr_chart 图表界面只能在电机正常且摇杆控制时才能看到电机状态变化。

**核心需求**：实现一个独立的电机状态监控界面，让用户在任何时候都能看到12个电机的连接状态，快速定位故障电机。

### 技术方案

#### 数据流设计

```
电机硬件 → SPI-CAN转换 → 电机驱动层(MotorMitV3) → 消息传递 → 网络层(quadDebugServer) → UDP网络 → qr_chart接收 → Qt界面显示
```

#### 协议扩展

在 MIT 电机协议的返回数据结构中增加状态字段：

```cpp
struct mit_motor_ret {
    double alpha{0};   // 位置 (rad)
    double torq{0};    // 速度 (rad/s)
    double blta{0};    // 力矩 (Nm)
    uint8_t sta{0};    // 状态字段（新增）
};
```

电机状态枚举定义：
- 0: ok - 正常
- 1: disable - 未使能
- 2: enable - 已使能
- 3: error - 错误
- 4: timeout - 超时（通讯丢失）

### 代码修改清单

#### qr_wl 项目修改

1. **baseline/pub/impl/msgType.hpp**
   - 在 `mit_motor_ret` 结构体中添加 `uint8_t sta{0}` 字段

2. **app/motion/qrMotor/qrMotorMitV3.cpp**
   - `BlockRecv()` 函数：填充 sta 字段
   ```cpp
   data.leg[leg][motor].sta = static_cast<uint8_t>(infos_[leg][motor].state);
   ```

3. **app/user/userQuad/quadDebugServer.cpp**
   - `SaveMotorRet()` 函数：将 sta 字段拷贝到网络数据包
   ```cpp
   quad_debug_packet_.legRet.leg[leg][0].sta = data.leg[leg][0].sta;
   quad_debug_packet_.legRet.leg[leg][1].sta = data.leg[leg][1].sta;
   quad_debug_packet_.legRet.leg[leg][2].sta = data.leg[leg][2].sta;
   ```

4. **app/motion/hardware/spi2canV3.cpp**
   - `ErrorCheck()` 函数：SPI校验错误时清空rxCan_缓存，防止返回旧数据导致误判电机在线
   ```cpp
   if (chkErr == true || isOk_ == false) {
       rxCan_ = {};  // 清空rxCan_
       return;
   }
   ```

5. **app/motion/qrMotor/qrMotorMitV3.cpp** (关键修复)
   - `Transfer()` 函数：即使数据无效也要调用 `CheckMotorState()`，确保超时检测机制工作
   ```cpp
   // 检查数据完整性
   bool dataValid = true;
   for (int i = 0; i < 4; i++) {
       if (msg[i].size() != (size_t)MOTOR_NUM) {
           dataValid = false;
           break;
       }
   }

   // 构造检查结果向量
   vector<vector<bool>> checks;
   for (int leg = 0; leg < LEG_NUM; leg++) {
       vector<bool> check;
       for (int motor = 0; motor < MOTOR_NUM; motor++) {
           bool ret = false;
           if (dataValid) {
               ret = infos_[leg][motor].trans->CanToDataV3(msg[leg][motor], &motorRawPrevData_.leg[leg][motor]);
           }
           check.push_back(ret);  // 数据无效时推入false
       }
       checks.push_back(check);
   }

   // 即使数据无效，也要调用CheckMotorState让超时检测工作
   CheckMotorState(checks);
   ```

6. **script/env.sh**
   - 修复 Docker TTY 问题：将 `-it` 改为 `-i`

#### qr_chart 项目修改

1. **msg/motion_data_t.h**
   - 添加状态数组
   ```cpp
   uint8_t sta_abad[4];
   uint8_t sta_hip[4];
   uint8_t sta_knee[4];
   ```

2. **DataCollector/DataStruct.h**
   - MotorInfo 结构体添加 sta 字段
   ```cpp
   struct MotorInfo {
       double q;
       double qd;
       double tau;
       uint8_t sta;  // 新增
   };
   ```

3. **DataCollector/debugClient.cpp**
   - `LegData()` 函数：解析网络数据包中的状态字段
   ```cpp
   elem.ret[j].motor[0].sta = data.ret.leg[j][0].sta;
   elem.ret[j].motor[1].sta = data.ret.leg[j][1].sta;
   elem.ret[j].motor[2].sta = data.ret.leg[j][2].sta;
   ```

4. **mainwindow.h**
   - 添加电机状态表格相关声明
   ```cpp
   #include <QTableWidget>

   void InitMotorStateTable();
   void UpdateMotorState();
   QTableWidget* motorStateTable_ = nullptr;

   private slots:
   void on_actionMotorState_triggered();
   ```

5. **mainwindow.cpp**
   - 构造函数：创建菜单动作
   ```cpp
   InitMotorStateTable();

   QAction* actionMotorState = new QAction(tr("查看电机状态"), this);
   ui->menu_motorState->addAction(actionMotorState);
   connect(actionMotorState, &QAction::triggered, this, &MainWindow::on_actionMotorState_triggered);
   ```

   - `InitMotorStateTable()`：初始化电机状态表格
   ```cpp
   motorStateTable_ = new QTableWidget(12, 4, this);
   QStringList headers;
   headers << "电机名称" << "当前位置" << "状态" << "最后更新";
   motorStateTable_->setHorizontalHeaderLabels(headers);

   QStringList motorNames;
   motorNames << "左前腿-abad" << "左前腿-hip" << "左前腿-knee"
              << "右前腿-abad" << "右前腿-hip" << "右前腿-knee"
              << "右后腿-abad" << "右后腿-hip" << "右后腿-knee"
              << "左后腿-abad" << "左后腿-hip" << "左后腿-knee";

   // 初始化表格内容...
   motorStateTable_->setWindowTitle("电机状态监控");
   motorStateTable_->setWindowFlags(Qt::Window);
   ```

   - `UpdateMotorState()`：更新电机状态显示（带颜色编码）
   ```cpp
   switch (sta) {
       case 0:  // ok
           statusText = "正常";
           bgColor = QColor(144, 238, 144);
           break;
       case 1:  // disable
           statusText = "未使能";
           bgColor = QColor(200, 200, 200);
           break;
       case 2:  // enable
           statusText = "已使能";
           bgColor = QColor(152, 251, 152);
           break;
       case 3:  // error
           statusText = "错误";
           bgColor = QColor(255, 99, 71);
           break;
       case 4:  // timeout
           statusText = "超时";
           bgColor = QColor(255, 165, 0);
           break;
   }
   ```

   - `on_actionMotorState_triggered()`：菜单点击处理
   ```cpp
   if (motorStateTable_ != nullptr) {
       motorStateTable_->show();
       motorStateTable_->raise();
       motorStateTable_->activateWindow();
   }
   ```

6. **mainwindow.ui**
   - 添加电机状态菜单
   ```xml
   <widget class="QMenu" name="menu_motorState">
    <property name="title">
     <string>电机状态</string>
    </property>
   </widget>
   ```

7. **chart/chartwidget.h 和 chart/chartwidget.cpp**
   - Qt5兼容性修复：将 `QEnterEvent` 改为 `QEvent`

### 遇到的问题及解决方案

#### 问题1: Docker TTY 错误
**现象**：`./build.sh arm` 编译失败，提示 "the input device is not a TTY"

**原因**：Docker 配置使用了 `-it` 参数，要求交互式终端

**解决**：修改 `script/env.sh`，将 `-it` 改为 `-i`

#### 问题2: Qt5/Qt6 兼容性问题
**现象**：编译 qr_chart 时报错 `QEnterEvent` 不存在

**原因**：Qt5 使用 `QEvent*`，Qt6 引入了 `QEnterEvent*`

**解决**：将 `chartwidget.h` 和 `chartwidget.cpp` 中的 `QEnterEvent` 改为 `QEvent`

#### 问题3: 网络层缺少状态字段拷贝
**现象**：qr_chart 收不到状态数据

**原因**：`quadDebugServer.cpp` 中 `SaveMotorRet()` 函数未拷贝 sta 字段到网络数据包

**解决**：添加 sta 字段拷贝代码

#### 问题4: 用户体验问题 - 窗口自动弹出
**现象**：程序启动时电机状态窗口自动弹出

**反馈**：用户认为自动弹出的体验不好

**解决**：改为菜单触发方式，在菜单栏添加"电机状态"菜单项，点击后显示窗口

#### 问题5: 电机未连接时状态显示错误（核心问题）
**现象**：未连接电机时，qr_chart 显示电机状态为"已使能"(sta=2)，应该显示"超时"(sta=4)

**根本原因分析**：
1. SPI 校验错误发生时，`spi2canV3.cpp` 的 `ErrorCheck()` 函数未清空 `rxCan_` 缓存
2. `GetMessage()` 返回旧的/脏数据
3. `qrMotorMitV3.cpp` 的 `Transfer()` 函数检测到 `msg[i].size() != MOTOR_NUM` 时提前返回
4. `CheckMotorState()` 未被调用，超时检测机制失效
5. 电机状态保持初始化时的 `enable` 状态

**解决方案（两部分）**：
1. **spi2canV3.cpp**: SPI 校验错误时清空 rxCan_ 缓存
   ```cpp
   if (chkErr == true || isOk_ == false) {
       rxCan_ = {};  // 清空rxCan_，避免返回旧数据导致误判电机在线
       return;
   }
   ```

2. **qrMotorMitV3.cpp**: 即使数据无效也要调用 CheckMotorState()
   ```cpp
   // 即使数据无效时，checks 向量中填充 false，
   // CheckMotorState() 收到 false 时不更新 lastRecv 时间戳，
   // 超过300ms后触发超时检测，将状态设置为 timeout
   CheckMotorState(checks);
   ```

**验证结果**：
未连接电机时，所有12个电机在302ms后正确显示超时状态：
```
[10:23:15-839][error][robot] motor:left_front_abad, comm lose, dura:302
[10:23:15-840][error][robot] motor:left_front_hip, comm lose, dura:302
[10:23:15-842][error][robot] motor:left_front_knee, comm lose, dura:302
... (共12个电机)
```

### 部署流程

1. **编译 ARM 版本**
   ```bash
   cd /home/wl/twh/workspace/qr_wl
   ./build.sh arm
   ```

2. **部署到树莓派**
   ```bash
   sshpass -p '123456' scp build/arm64/output/qr wl@192.168.1.54:/home/wl/autorun/iiri-qr/qr.new
   sshpass -p '123456' ssh wl@192.168.1.54 "cd /home/wl/autorun/iiri-qr && cp qr qr.backup_20251015 && mv qr.new qr"
   ```

3. **重启 qr 程序**
   ```bash
   # 查找并杀掉旧进程
   ps aux | grep qr
   kill <pid>

   # 启动新版本
   cd /home/wl/autorun/iiri-qr
   ./qr_start.sh qr-linkV2-3.toml
   ```

4. **编译并启动 qr_chart**
   ```bash
   cd /home/wl/twh/workspace/client/qr_chart
   ./build.sh
   export DISPLAY=:0 && ./build/qrChart
   ```

### 代码提交记录

#### qr_wl 项目
- 分支：dev
- 提交 ID：665711e
- 提交信息：电机状态监控功能实现及超时检测修复
- 修改文件：
  - `app/motion/hardware/spi2canV3.cpp`
  - `app/motion/qrMotor/motorMitV3.cpp`
  - `app/motion/qrMotor/qrMotorMitV3.cpp`
  - `app/user/userQuad/quadDebugServer.cpp`
  - `baseline/pub/impl/msgType.hpp`
  - `script/env.sh`

- 合并到主分支：
  ```bash
  git checkout main
  git pull origin main
  git merge dev  # Fast-forward merge 981f039..665711e
  git push origin main
  ```

#### qr_chart 项目
- 分支：master
- 提交 ID：4ac32d3
- 仓库：http://192.168.1.55/hepHephaestushaestus/qr-chart-linux.git
- 修改文件：
  - `msg/motion_data_t.h`
  - `DataCollector/DataStruct.h`
  - `DataCollector/debugClient.cpp`
  - `mainwindow.h`
  - `mainwindow.cpp`
  - `mainwindow.ui`
  - `chart/chartwidget.h`
  - `chart/chartwidget.cpp`

### 功能验证

1. **数据流验证**：通过在 `debugClient.cpp` 中添加 `qDebug()<<"123";` 调试输出，确认数据接收正常（日志中出现2500+次"123"输出）

2. **UI 显示验证**：
   - 菜单项正确显示在"文件"和"帮助"之间
   - 点击菜单项后窗口正常弹出
   - 表格显示12个电机的名称、位置、状态、更新时间

3. **状态颜色验证**：
   - 正常(ok): 绿色
   - 未使能(disable): 灰色
   - 已使能(enable): 浅绿色
   - 错误(error): 红色
   - 超时(timeout): 橙色

4. **超时检测验证**：
   - 未连接电机时，所有电机在300ms后显示"超时"状态
   - 日志正确输出 "motor:xxx, comm lose, dura:302" 错误信息

### 关键设计思路

1. **最小化协议修改**：在现有协议中添加一个 uint8_t 字段，默认值为0，保持向后兼容

2. **利用现有机制**：复用已有的超时检测代码 `CheckMotorState()`，通过修复调用逻辑而非重写

3. **用户体验优先**：根据用户反馈从自动弹出窗口改为菜单触发，提升使用体验

4. **完整的数据流**：确保状态信息在整个数据链路中传递无误：
   - 电机层填充状态
   - 网络层拷贝状态
   - SDK 解析状态
   - UI 显示状态

5. **健壮的错误处理**：
   - SPI 错误时清空缓存
   - 数据无效时仍然调用超时检测
   - 确保超时机制在任何情况下都能工作

### 测试环境

- **开发机**：Ubuntu Linux 6.8.0-85-generic
- **目标设备**：Raspberry Pi (192.168.1.54)
  - 用户名: wl
  - 密码: 123456
  - 工作目录: /home/wl/autorun/iiri-qr

- **通讯协议**：UDP 端口 20333
- **数据更新频率**：2ms (500Hz)
- **超时阈值**：300ms
- **UI 刷新频率**：100ms

### 经验总结

1. **问题诊断要看准时间戳**：在排查超时检测问题时，发现日志时间戳与当前时间不符（14:25 vs 10:13），说明看的是旧日志，需要重新部署测试

2. **理解现有代码逻辑**：`CheckMotorState()` 超时检测代码本身是正确的，问题在于它没有被正确调用，修复调用逻辑比重写功能更高效

3. **用户反馈很重要**：
   - 用户指出网络层缺少 sta 字段拷贝
   - 用户提出 UX 问题并给出改进方向
   - 用户提供了"一直检查"的解决思路

4. **防御性编程**：即使在异常情况下（数据无效），也要保持核心检测机制（超时检测）的正常工作

5. **分步验证**：通过添加调试输出逐步验证数据流的每个环节，快速定位问题

### 后续改进建议

1. **性能优化**：考虑将超时检测周期从2ms调整为10ms，降低CPU占用
2. **日志优化**：超时检测日志可以添加降频机制，避免持续输出相同错误
3. **UI 增强**：可以添加电机重连次数、通讯质量等更多诊断信息
4. **配置化**：超时阈值(300ms)可以改为配置参数，方便调试

---

**文档最后更新时间**：2025-10-15
**修改人**：Claude
**版本**：v1.0
