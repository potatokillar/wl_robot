
#pragma once

#include "baseline.hpp"

RobotState GetRobotCurState();
const std::string& GetRobotCurStateInfo();
void SetRobotCurState(const RobotState set);
void SetRobotCurState(const RobotState set, const std::string& info);
