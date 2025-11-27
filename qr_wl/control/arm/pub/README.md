对外接口

用户侧，是由app调用的，app可以直接包含算法，因此该层是通过封装API实现。   
电机侧，逻辑上是算法调用电机，因此不通过API实现，直接通过暴露消息接口的方式实现。   
用户侧也可以自行通过消息接口实现，不调用apiArmCtrl.hpp即可。

## 用户侧消息
用户通过API访问

|标签| 类型 | 发送者 | 接收者 | 说明 |
|:----:| :------:| :------: |  :------: | :------: |
|arm::move| rpc | 用户 | 算法 |设置移动任务
|arm::wait_task_complete| rpc | 用户 | 算法 |等待移动任务完成
|arm::user_ctrl| rpc | 用户 | 算法 |用户特定控制指令
|arm::tool_name_list| rpc | 用户 | 算法 | 获取工具坐标系列表
|arm::set_tool_name| rpc | 用户 | 算法 | 设置当前使用的工具坐标系
|arm::get_tool_name| rpc | 用户 | 算法 | 获取当前使用的工具坐标系
|arm::toolvalue| rpc | 用户 | 算法 | 获取对应工具坐标系的实际坐标
|arm::user_set_io_out| rpc | 用户 | 算法 | 设置当前输出io状态
||
|arm::get_param| rpc | 用户 | 算法 | 获取机械臂配置参数
|arm::pub_info| msg | 算法 | 用户 | 实时上报机械臂信息，替换上面的获取接口todo

## 电机侧消息
电机通过暴露的消息接口直接访问。

|标签| 类别 | 类型 | 发送者 | 接收者 | 说明 |
|:----:| :------:| :------:| :------: |  :------: | :------: |
|arm::motor_data| msg | msg::arm_data| 电机 | 算法 |电机当前位置等信息
|arm::motor_cmd| msg | msg::arm_cmd| 电机 | 算法/网络 |电机当前执行的命令
|arm::motor_info| msg |MotorInfo| 电机 | 算法 |电机实时状态，运行状态，io状态，执行命令等
||
|arm::motor_ctrl| rpc |MotorCtrl| 算法 | 电机 |请求电机执行特殊指令，使能暂停等
|arm::motor_cmd_queue| rpc |std::vector < msg::arm_cmd > | 算法 | 电机 |电机命令执行序列
|arm::set_io_out| rpc |std::vector < u8 >| 算法 | 电机 |请求电机设置IO输出
