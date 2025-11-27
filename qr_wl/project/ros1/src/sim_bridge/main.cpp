
#include <ros/ros.h>

#include "baseline.hpp"
#include "deviceParam.hpp"
#include "gamepad.hpp"
#include "robotState.hpp"
#include "simArm.hpp"
#include "simQuadruped.hpp"
#include "userArmNode.hpp"
#include "userQuadNode.hpp"

using namespace ::std;

void PrintRequiredInfo(char** argv)
{
    // 获取版本号
    auto tag = GetDevParam().version.software;
    auto sdk = GetDevParam().version.sdk;

    LOG_INFO("robot type:{}, software version:{}, sdk version:{}", argv[1], tag, sdk);
}

void NetworkLoop()
{
    GetMiniServer().Loop();  // 事件循环
}

int main(int argc, char** argv)
{
    // 传入配置文件，必须要有
    if (argc == 2) {
        if (GetRobotCfgFile().ParseFile(argv[1]) == false) {
            printf("config file no exist or error, exit!");
            return -1;
        }
    }
    PrintRequiredInfo(argv);
    bootArgs.isSim = true;
    bootArgs.noMotor = false;

    LOG_DEBUG("sim_bridge main test");

    // 手柄
    auto useGamepad = GetRobotConfigDirect<bool>("useGamepad");
    if (useGamepad.has_value() && useGamepad.value()) {
        LOG_INFO("use gamepad");
        GetGamepadNode().Init();  // 手柄控制节点
    }

    // ROS节点初始化
    ros::init(argc, argv, "sim_bridge");

    // 统一根据配置文件来决定是否启用
    auto ctrModel = GetRobotConfigDirect<string>("ctrModel");

    if (ctrModel.has_value()) {
        int len = ctrModel.value().size();
        if (ctrModel.value().substr(0, std::min(len, 3)) == "qr-") {
            GetQuadNode().Init();
            auto evtThread = std::thread(NetworkLoop);
            SetRobotCurState(RobotState::standby);

            SimQuadruped simQuad;
            simQuad.Run();

        } else if (ctrModel.value().substr(0, std::min(len, 4)) == "arm-") {
            GetArmNode().Init();
            auto evtThread = std::thread(NetworkLoop);

            auto run_state = GetRobotConfigDirect<string>("run_state");

            SimArm simArm;
            SetRobotCurState(RobotState::standby);
            simArm.Run(run_state.value());
        }
    }

    return 0;
}