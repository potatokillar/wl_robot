
#include "userArmNode.hpp"

#include "deviceParam.hpp"

using namespace std;

UserArmNode::UserArmNode()
{
    auto armApi = make_shared<ApiArm>();
    auto devApi = make_shared<ApiDevice>();

    sdk_ = make_unique<ArmSdk>(armApi, devApi);
    key_ = make_unique<ArmKey>(armApi, devApi);

    armApi->Start();  // 机械臂上电直接启动算法，不能关闭
    thread_ = std::thread(&UserArmNode::Loop, this);
}

UserArmNode::~UserArmNode() {}

void UserArmNode::Loop()
{
    while (1) {
        key_->Run();
        sdk_->Run();
        // 还是得控制频率，阻止实际用户接口的疯狂调用
        TimerTools::SleepForMs(5);
    }
}