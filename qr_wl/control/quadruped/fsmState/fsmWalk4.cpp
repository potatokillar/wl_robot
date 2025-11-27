
#include "fsmWalk4.hpp"

#include "baseline.hpp"
#include "eventMsg.hpp"
#include "orientation_tools.h"

using namespace std;
using namespace ori;

FsmWalk4::FsmWalk4(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::walk4, contr, jpData) {}

void FsmWalk4::OnEnter()
{
    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        LOG_WARN("Enter HandStand2Walk. No Implementation for linkV2_3_w.");
    } else if (qrParam.model.contains(QrModel::linkV2_3) || qrParam.model.contains(QrModel::gazebo)) {
        LOG_INFO("Enter HandStand2Walk.");
    } else if (qrParam.model.contains(QrModel::middleV3)) {
        LOG_WARN("Enter HandStand2Walk. No Implementation for middleV3.");
    } else {  // if (qrParam.model.contains(QrModel::middleV3_w)}
        LOG_ERROR("ERROR robot type!");
    }
    done_ = false;
}

void FsmWalk4::Run()
{
    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        SmallWheel();
    } else if (qrParam.model.contains(QrModel::linkV2_3) || qrParam.model.contains(QrModel::gazebo)) {
        Small();
    } else if (qrParam.model.contains(QrModel::middleV3)) {
        Middle();
    } else {  // if (qrParam.model.contains(QrModel::middleV3_w)}
        MiddleWheel();
    }
}

void FsmWalk4::Small()
{
    interval_++;
    if (interval_ == durations_) {
        done_ = true;
    }
}

void FsmWalk4::SmallWheel()
{
    done_ = true;
    return;
}

void FsmWalk4::Middle()
{
    done_ = true;
    return;
}

void FsmWalk4::MiddleWheel()
{
    done_ = true;
    return;
}

void FsmWalk4::OnExit()
{
    LOG_INFO("Exit HandStand2Walk");
    interval_ = 0;
}