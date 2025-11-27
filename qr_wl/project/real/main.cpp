

#include <malloc.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h> /* See NOTES */
#include <unistd.h>

#include <chrono>

#include "baseline.hpp"
#include "canBridge.hpp"
#include "deviceCustomParam.hpp"
#include "gamepad.hpp"
#include "gripper.hpp"
#include "imuDrv.hpp"
#include "miniServer.hpp"
#include "offlineVoiceModule.hpp"
#include "realsense.hpp"
#include "robotMedia.hpp"
#include "robotState.hpp"
#include "userArmNode.hpp"
#include "userQuadNode.hpp"

using namespace std;

void PrintRequiredInfo(char **argv)
{
    // 获取版本号
    auto tag = GetDevParam().version.software;
    auto sdk = GetDevParam().version.sdk;
    auto gitCode = GetDevParam().version.git_branch_commit_md5_code;

    LOG_INFO("robot type:{}, software version:{}, sdk version:{}", argv[1], tag, sdk);
    LOG_INFO("git info: {}", gitCode);
}

int main(int argc, char **argv)
{
    SetRobotCurState(RobotState::initing);
    DebugTools debugTools;
    // 传入配置文件，必须要有
    if (argc >= 2)
    {
        if (GetRobotCfgFile().ParseFile(argv[1]) == false)
        {
            LOG_CRITICAL("config file no exist or error, exit!");
            return -1;
        }
        // RobotLogInit();  // 日志初始化放最开始
        PrintRequiredInfo(argv);

        if (argc == 3)
        {
            string arg{argv[2]};
            if (arg == "debug")
            {
                LOG_INFO("debug mode");
                bootArgs.noMotor = true;
            }
        }
    }
    else
    {
        return -1;
    }

    GetDevCustomParam().UpdateBootCount();

    // GetVoice().Init();        // 离线语模块音启动
    GetImuNode().Init();
    GetCanBridge().Init();

    // 手柄
    auto useGamepad = GetRobotConfigDirect<bool>("useGamepad");
    if (useGamepad.has_value() && useGamepad.value())
    {
        LOG_INFO("use gamepad");
        GetGamepadNode().Init(); // 手柄控制节点
    }

    auto ctrModel = GetRobotConfigDirect<string>("ctrModel");
    if (ctrModel.has_value())
    {
        int len = ctrModel.value().size();
        if (ctrModel.value().substr(0, std::min(len, 3)) == "qr-")
        {
            GetQuadNode().Init();
        }
        else if (ctrModel.value().substr(0, std::min(len, 4)) == "arm-")
        {
            GetArmNode().Init();
        }
    }

    // 初始化完成，进入待机状态
    SetRobotCurState(RobotState::standby);
    AddMediaPackage(MediaTaskPackage("readyToStart"));

    GetMiniServer().Loop(); // 事件循环
}