/*
 * @Author: 唐文浩
 * @Date: 2024-03-18
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "armTask.hpp"

#include <iostream>

#include "apiArm.hpp"
#include "apiDevice.hpp"
#include "metaData.hpp"
#include "sdkProtocolClient.hpp"
#include "timerTools.hpp"

using namespace std;
namespace iiri::arm
{

    ArmTask::ArmTask(const std::string &ip)
    {
        rpcCli_ = make_shared<SdkProtocol>(ip);
        apiDev_ = make_unique<ApiDevice>(rpcCli_);
        apiArm_ = make_unique<ApiArm>(rpcCli_);
        cout << meta() << endl; // 打印元数据
    }

    // 析构和移动使用默认
    ArmTask::~ArmTask() {}
    ArmTask::ArmTask(ArmTask &&rhs) = default;
    ArmTask &ArmTask::operator=(ArmTask &&rhs) = default;

    /**
     * @description: 元数据
     * @return {}
     */
    const std::string_view &ArmTask::meta() { return SDK_VERSION; }

    RetState ArmTask::SetMoveJTaskBlock(const vector<double> &jointVal, MoveMode mode)
    {
        vector<vector<double>> vals;
        vals.push_back(jointVal);
        return SetMoveJTaskBlock(vals, mode);
    }

    RetState ArmTask::SetMoveJTaskBlock(const vector<double> &jointVal, MoveMode mode, double speed)
    {
        vector<vector<double>> vals;
        vals.push_back(jointVal);
        return SetMoveJTaskBlock(vals, mode, speed);
    }

    RetState ArmTask::SetMoveJTaskBlock(const vector<double> &jointVal, MoveMode mode, double speed, double acc)
    {
        vector<vector<double>> vals;
        vals.push_back(jointVal);
        return SetMoveJTaskBlock(vals, mode, speed, acc);
    }

    RetState ArmTask::SetMoveJTaskBlock(const std::vector<std::vector<double>> &jointVals, MoveMode mode)
    {
        auto time = apiArm_->SetMoveJTask(jointVals, mode);
        if (time.first == RetState::ok)
        {
            auto ret = apiArm_->SetCtrlState(ArmCtrl::start);
            if (ret == RetState::ok)
            {
                return apiArm_->WaitTaskComplete(time.second);
            }
            return ret;
        }
        return time.first;
    }
    RetState ArmTask::SetMoveJTaskBlock(const std::vector<std::vector<double>> &jointVals, MoveMode mode, double speed)
    {
        auto time = apiArm_->SetMoveJTask(jointVals, mode, speed);
        if (time.first == RetState::ok)
        {
            auto ret = apiArm_->SetCtrlState(ArmCtrl::start);
            if (ret == RetState::ok)
            {
                return apiArm_->WaitTaskComplete(time.second);
            }
            return ret;
        }
        return time.first;
    }
    RetState ArmTask::SetMoveJTaskBlock(const std::vector<std::vector<double>> &jointVals, MoveMode mode, double speed, double acc)
    {
        auto time = apiArm_->SetMoveJTask(jointVals, mode, speed, acc);
        if (time.first == RetState::ok)
        {
            auto ret = apiArm_->SetCtrlState(ArmCtrl::start);
            if (ret == RetState::ok)
            {
                return apiArm_->WaitTaskComplete(time.second);
            }
            return ret;
        }
        else
        {
            cout << "SetMoveJ::::" << Enum2Num(time.first) << endl;
        }
        return time.first;
    }

    RetState ArmTask::SetMoveLTaskBlock(const CartesianPose &jointVal, MoveMode mode)
    {
        vector<CartesianPose> vals;
        vals.push_back(jointVal);
        return SetMoveLTaskBlock(vals, mode);
    }

    RetState ArmTask::SetMoveLTaskBlock(const CartesianPose &jointVal, MoveMode mode, double speed)
    {
        vector<CartesianPose> vals;
        vals.push_back(jointVal);
        return SetMoveLTaskBlock(vals, mode, speed);
    }
    RetState ArmTask::SetMoveLTaskBlock(const CartesianPose &jointVal, MoveMode mode, double speed, double acc)
    {
        vector<CartesianPose> vals;
        vals.push_back(jointVal);
        return SetMoveLTaskBlock(vals, mode, speed, acc);
    }

    RetState ArmTask::SetMoveLTaskBlock(const std::vector<CartesianPose> &jointVals, MoveMode mode)
    {
        auto time = apiArm_->SetMoveLTask(jointVals, mode);
        if (time.first == RetState::ok)
        {
            auto ret = apiArm_->SetCtrlState(ArmCtrl::start);
            if (ret == RetState::ok)
            {
                return apiArm_->WaitTaskComplete(time.second);
            }
            return ret;
        }
        return time.first;
    }
    RetState ArmTask::SetMoveLTaskBlock(const std::vector<CartesianPose> &jointVals, MoveMode mode, double speed)
    {
        auto time = apiArm_->SetMoveLTask(jointVals, mode, speed);
        if (time.first == RetState::ok)
        {
            auto ret = apiArm_->SetCtrlState(ArmCtrl::start);
            if (ret == RetState::ok)
            {
                return apiArm_->WaitTaskComplete(time.second);
            }
            return ret;
        }
        return time.first;
    }
    RetState ArmTask::SetMoveLTaskBlock(const std::vector<CartesianPose> &jointVals, MoveMode mode, double speed, double acc)
    {
        auto time = apiArm_->SetMoveLTask(jointVals, mode, speed, acc);
        if (time.first == RetState::ok)
        {
            auto ret = apiArm_->SetCtrlState(ArmCtrl::start);
            if (ret == RetState::ok)
            {
                return apiArm_->WaitTaskComplete(time.second);
            }
            return ret;
        }
        return time.first;
    }
    RetState ArmTask::SetMoveJTaskNoBlock(const std::vector<double> &jointVal, MoveMode mode)
    {
        vector<vector<double>> vals;
        vals.push_back(jointVal);
        auto time = apiArm_->SetMoveJTask(vals, mode);
        return time.first;
    }
    RetState ArmTask::SetMoveJTaskNoBlock(const std::vector<double> &jointVal, MoveMode mode, double speed)
    {
        vector<vector<double>> vals;
        vals.push_back(jointVal);
        auto time = apiArm_->SetMoveJTask(vals, mode, speed);
        return time.first;
    }

    RetState ArmTask::SetMoveJTaskNoBlock(const std::vector<double> &jointVal, MoveMode mode, double speed, double acc)
    {
        vector<vector<double>> vals;
        vals.push_back(jointVal);
        auto time = apiArm_->SetMoveJTask(vals, mode, speed, acc);
        return time.first;
    }

    RetState ArmTask::SetMoveLTaskNoBlock(const CartesianPose &jointVal, MoveMode mode)
    {
        vector<CartesianPose> vals;
        vals.push_back(jointVal);
        auto time = apiArm_->SetMoveLTask(vals, mode);
        return time.first;
    }
    RetState ArmTask::SetMoveLTaskNoBlock(const CartesianPose &jointVal, MoveMode mode, double speed)
    {
        vector<CartesianPose> vals;
        vals.push_back(jointVal);
        auto time = apiArm_->SetMoveLTask(vals, mode, speed);
        return time.first;
    }

    RetState ArmTask::SetMoveLTaskNoBlock(const CartesianPose &jointVal, MoveMode mode, double speed, double acc)
    {
        vector<CartesianPose> vals;
        vals.push_back(jointVal);
        auto time = apiArm_->SetMoveLTask(vals, mode, speed, acc);
        return time.first;
    }

    RetState ArmTask::SetEnable(bool set)
    {
        if (set)
        {
            apiArm_->SetCtrlState(ArmCtrl::enable);
        }
        else
        {
            apiArm_->SetCtrlState(ArmCtrl::disable);
        }
        // 延时一秒等待后判断状态
        TimerTools::SleepForS(1);
        auto sta = apiArm_->GetArmState();
        if (sta.first == RetState::ok)
        {
            if (set)
            {
                if (sta.second == ArmState::ready)
                {
                    return RetState::ok;
                }
            }
            else
            {
                // 失能不会清除错误，若电机存在错误，本身就是失能状态
                if ((sta.second == ArmState::noReady) || (sta.second == ArmState::error))
                {
                    return RetState::ok;
                }
            }
            return RetState::error;
        }
        return sta.first;
    }

    RetState ArmTask::SetTaskCmd(ArmTaskCmd cmd)
    {
        switch (cmd)
        {
        case ArmTaskCmd::start:
            return apiArm_->SetCtrlState(ArmCtrl::start);
        case ArmTaskCmd::pause:
            return apiArm_->SetCtrlState(ArmCtrl::pause);
        case ArmTaskCmd::stop:
            return apiArm_->SetCtrlState(ArmCtrl::stop);
        default:
            break;
        }
        return RetState::noSupport;
    }

    std::pair<RetState, std::vector<std::string>> ArmTask::GetToolNameList() { return apiArm_->GetToolNameList(); }
    RetState ArmTask::SetToolName(const std::string &name) { return apiArm_->SetToolName(name); }
    std::pair<RetState, std::string> ArmTask::GetToolName() { return apiArm_->GetToolName(); }
    std::pair<RetState, CartesianPose> ArmTask::GetToolValue(const std::string &name) { return apiArm_->GetToolValue(name); }

    RetState ArmTask::SetToolValue(const std::string &name, const CartesianPose &pose) { return apiArm_->SetToolValue(name, pose); }

    std::pair<RetState, std::vector<double>> ArmTask::GetNowAngle() { return apiArm_->GetNowAngle(); }

    RetState ArmTask::SubscribeNowAngle(std::function<void(const std::vector<double> &)> func) { return apiArm_->SubscribeNowAngle(func); }
    std::pair<RetState, CartesianPose> ArmTask::GetNowCart() { return apiArm_->GetNowCart(); }

    RetState ArmTask::SubscribeNowCart(std::function<void(const CartesianPose &)> func) { return apiArm_->SubscribeNowCart(func); }
    std::pair<RetState, ArmState> ArmTask::GetArmState() { return apiArm_->GetArmState(); }

    RetState ArmTask::SubscribeArmState(std::function<void(ArmState)> func) { return apiArm_->SubscribeArmState(func); }

    std::pair<RetState, CartesianPose> ArmTask::GetFkine(const std::array<double, 6> &robotjoint) { return apiArm_->GetFkine(robotjoint); }
    std::pair<RetState, std::array<double, 6>> ArmTask::GetIkine(const CartesianPose &FK) { return apiArm_->GetIkine(FK); }

    Result<CartesianPose7> ArmTask::GetFkine7(const std::array<double, 7> &robotjoint) { return apiArm_->GetFkine7(robotjoint); }
    Result<std::array<double, 7>> ArmTask::GetIkine7(const CartesianPose7 &FK) { return apiArm_->GetIkine7(FK); }

    RetState ArmTask::SetIoOutputState(const std::string &name, std::bitset<32> set) { return apiArm_->SetIoOutputState(name, set); }
    RetState ArmTask::SetIoOutputState(const std::string &name, uint8_t pos, bool set)
    {
        auto [ret, sta] = GetIoOutputState(name);
        if (ret == RetState::ok)
        {
            if (pos < sta.size())
            {
                if (set)
                {
                    sta.set(pos);
                }
                else
                {
                    sta.reset(pos);
                }
                return SetIoOutputState(name, sta);
            }
            else
            {
                return RetState::outRange;
            }
        }
        return ret;
    }

    std::pair<RetState, std::bitset<32>> ArmTask::GetIoOutputState(const std::string &name) { return apiArm_->GetIoOutputState(name); }
    std::pair<RetState, bool> ArmTask::GetIoOutputState(const std::string &name, uint8_t pos)
    {
        auto [ret, sta] = GetIoOutputState(name);
        if (ret == RetState::ok)
        {
            if (pos < sta.size())
            {
                return {ret, sta.test(pos)};
            }
            return {RetState::outRange, 0};
        }
        return {ret, 0};
    }

    std::pair<RetState, std::bitset<32>> ArmTask::GetIoInputState(const std::string &name) { return apiArm_->GetIoInputState(name); }
    std::pair<RetState, bool> ArmTask::GetIoInputState(const std::string &name, uint8_t pos)
    {
        auto [ret, sta] = GetIoInputState(name);
        if (ret == RetState::ok)
        {
            if (pos < sta.size())
            {
                return {ret, sta.test(pos)};
            }
            return {RetState::outRange, 0};
        }
        return {ret, 0};
    }

    RetState ArmTask::SubscribeIoOutputState(std::function<void(const std::map<std::string, std::bitset<32>> &)> func)
    {
        return apiArm_->SubscribeIoOutputState(func);
    }
    RetState ArmTask::SubscribeIoInputState(std::function<void(const std::map<std::string, std::bitset<32>> &)> func)
    {
        return apiArm_->SubscribeIoInputState(func);
    }
    std::pair<RetState, std::vector<ValueRange<double>>> ArmTask::GetPositionLimit() { return apiArm_->GetPositionLimit(); }
    RetState ArmTask::SetPositionLimit(const std::vector<ValueRange<double>> &jLimit) { return apiArm_->SetPositionLimit(jLimit); }
    RetState ArmTask::SetToTransport()
    {
        vector<double> val{0.0, -0.52, 2.0944, 0, 1.57, 0.0};
        return SetMoveJTaskBlock(val, MoveMode::abs, 0.01);
    }
    RetState ArmTask::SetToHome()
    {
        vector<double> val{0.0, 0.0, 0.0, 0, 0.0, 0.0};
        return SetMoveJTaskBlock(val, MoveMode::abs, 0.01);
    }

    RetState ArmTask::SetDragMode(bool set) { return apiArm_->SetDragMode(set); }

    std::pair<RetState, bool> ArmTask::GetDragMode() { return apiArm_->GetDragMode(); }

    RetState ArmTask::SetCollisionLevel(int level) { return apiArm_->SetCollisionLevel(level); }

    std::pair<RetState, int> ArmTask::GetCollisionLevel() { return apiArm_->GetCollisionLevel(); }

    RetState ArmTask::SetRapidrate(double rate) { return apiArm_->SetRapidrate(rate); }
    std::pair<RetState, double> ArmTask::GetRapidrate() { return apiArm_->GetRapidrate(); }

    RetState ArmTask::SetJointSpeedLimit(const std::vector<double> &vMax) { return apiArm_->SetJointSpeedLimit(vMax); }

    RetState ArmTask::SetJointAccLimit(const std::vector<double> &aMax) { return apiArm_->SetJointAccLimit(aMax); }

    std::pair<RetState, std::vector<double>> ArmTask::GetJointSpeedLimit() { return apiArm_->GetJointSpeedLimit(); }

    std::pair<RetState, std::vector<double>> ArmTask::GetJointAccLimit() { return apiArm_->GetJointAccLimit(); }

    std::pair<RetState, ArmInfo> ArmTask::GetArmInfo() { return apiArm_->GetArmInfo(); }

    RetState ArmTask::SubscribeArmInfo(std::function<void(const ArmInfo &)> func) { return apiArm_->SubscribeArmInfo(func); }

    RetState ArmTask::SetReset(bool set) { return apiArm_->SetReset(set); }

    Result<bool> ArmTask::GetReset() { return apiArm_->GetReset(); }

    RetState ArmTask::SetGripper(bool state, double speed) { return apiArm_->SetGripper(state, speed); }
} // namespace iiri::arm