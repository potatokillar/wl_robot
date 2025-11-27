
#include "robotState.hpp"

#include <mutex>

#include "baseline.hpp"

static RobotState robotState;
static std::string robotInfo;
static std::mutex mutexSta;

using namespace std;
RobotState GetRobotCurState()
{
    std::lock_guard<std::mutex> lock(mutexSta);
    return robotState;
}

const std::string& GetRobotCurStateInfo()
{
    std::lock_guard<std::mutex> lock(mutexSta);
    return robotInfo;
}

void SetRobotCurState(const RobotState set, const std::string& info)
{
    std::lock_guard<std::mutex> lock(mutexSta);
    if (robotState != set) {
        robotState = set;
        if (info == "") {
            switch (robotState) {
                case RobotState::initing:
                    robotInfo = string("initing");
                    break;
                case RobotState::standby:
                    robotInfo = "standby";
                    break;
                case RobotState::running:
                    robotInfo = "running";
                    break;
                case RobotState::error:
                    robotInfo = "error";
                    break;
                case RobotState::emgstop:
                    robotInfo = "emgstop";
                    break;
                case RobotState::jointCtrl:
                    robotInfo = "jointCtrl";
                    break;
                default:
                    break;
            }
        } else {
            robotInfo = info;
        }
    }
}

void SetRobotCurState(const RobotState set) { SetRobotCurState(set, ""); }