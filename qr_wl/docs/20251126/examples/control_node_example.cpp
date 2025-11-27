/**
 * @file control_node_example.cpp
 * @brief 自定义控制节点开发示例
 * 
 * 本示例展示了如何基于 ICallback 接口开发一个新的控制节点。
 * 开发者可以参考此模式添加自定义的算法逻辑。
 */

#include <iostream>
#include <vector>
#include <cmath>

// 模拟 ICallback 接口
class ICallback {
public:
    virtual ~ICallback() = default;
    virtual void Init() = 0;
    virtual void Loop() = 0;
};

// 模拟电机命令结构
struct MotorCmd {
    float q;    // 目标角度
    float dq;   // 目标角速度
    float kp;   // 位置增益
    float kd;   // 速度增益
    float tau;  // 前馈扭矩
};

/**
 * @brief 自定义正弦波控制节点
 * 
 * 该节点演示了如何生成简单的正弦波控制信号驱动电机。
 */
class SineWaveController : public ICallback {
private:
    std::vector<MotorCmd> cmds;
    double time_elapsed;
    const double dt = 0.001; // 1kHz 控制频率

public:
    SineWaveController() : time_elapsed(0) {
        cmds.resize(12); // 假设12个电机
    }

    /**
     * @brief 初始化控制器
     * 设置初始增益和状态
     */
    void Init() override {
        std::cout << "[SineWaveController] Initializing..." << std::endl;
        for (auto& cmd : cmds) {
            cmd.kp = 50.0f;
            cmd.kd = 1.0f;
            cmd.tau = 0.0f;
            cmd.q = 0.0f;
            cmd.dq = 0.0f;
        }
    }

    /**
     * @brief 控制循环
     * 每一帧调用一次，计算新的电机命令
     */
    void Loop() override {
        time_elapsed += dt;

        // 生成正弦波信号: Amplitude * sin(Frequency * time)
        float target_pos = 0.5f * std::sin(2.0f * M_PI * time_elapsed);
        float target_vel = 0.5f * 2.0f * M_PI * std::cos(2.0f * M_PI * time_elapsed);

        // 更新所有电机的命令
        for (int i = 0; i < 12; ++i) {
            cmds[i].q = target_pos;
            cmds[i].dq = target_vel;
            
            // 实际项目中，这里会调用 CanBridge 发送命令
            // GetCanBridge().SetMotorCmd(i, cmds[i]);
        }

        // 仅为演示打印第一个电机的状态
        if (int(time_elapsed * 1000) % 100 == 0) { // 每100ms打印一次
            std::cout << "[Ctrl] Time: " << time_elapsed 
                      << "s, Target Pos: " << target_pos << std::endl;
        }
    }
};

int main() {
    SineWaveController controller;
    controller.Init();

    // 模拟运行 1000 个周期 (1秒)
    for (int i = 0; i < 1000; ++i) {
        controller.Loop();
    }

    return 0;
}
