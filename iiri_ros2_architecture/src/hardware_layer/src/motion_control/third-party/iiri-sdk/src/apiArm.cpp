/*
 * @Author: 唐文浩
 * @Date: 2024-04-15
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description: 机械臂协议实现
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#include "apiArm.hpp"

#include <iostream>
#include <set>

using namespace std;

namespace iiri::arm
{
    const std::set<std::string> IO_NAME{"panel", "tool", "extra"};

    ApiArm::ApiArm(std::shared_ptr<SdkProtocol> rpcCli)
    {
        rpcCli_ = rpcCli;

        STRING_ARMSTATE["noRun"] = ArmState::noRun;
        STRING_ARMSTATE["init"] = ArmState::init;
        STRING_ARMSTATE["noReady"] = ArmState::noReady;
        STRING_ARMSTATE["ready"] = ArmState::ready;
        STRING_ARMSTATE["running"] = ArmState::running;
        STRING_ARMSTATE["pause"] = ArmState::pause;
        STRING_ARMSTATE["interrupt"] = ArmState::interrupt;
        STRING_ARMSTATE["complete"] = ArmState::complete;
        STRING_ARMSTATE["error"] = ArmState::error;
    }

    Result<nlohmann::json> ApiArm::Call(const std::string &method, const nlohmann::json &params)
    {
        return rpcCli_->Call("arm", method, params);
    }
    Result<nlohmann::json> ApiArm::Call(const std::string &method, const nlohmann::json &params, uint32_t ms)
    {
        return rpcCli_->Call("arm", method, params, ms);
    }

    nlohmann::json ApiArm::GetMoveJParam(const std::vector<std::vector<double>> &jointVals, MoveMode mode)
    {
        nlohmann::json param;

        for (const auto &vals : jointVals)
        {
            vector<std::string> jVals;
            for (const auto &v : vals)
            {
                jVals.push_back(Double2Json(v));
            }
            param["goal"].push_back(jVals);
        }

        if (mode == MoveMode::abs)
        {
            param["mode"] = "abs";
        }
        else
        {
            param["mode"] = "incr";
        }
        return param;
    }
    nlohmann::json ApiArm::GetMoveLParam(const std::vector<CartesianPose> &jointVals, MoveMode mode)
    {
        nlohmann::json param;

        for (const auto &v : jointVals)
        {
            vector<std::string> vals;
            vals.push_back(Double2Json(v.x));
            vals.push_back(Double2Json(v.y));
            vals.push_back(Double2Json(v.z));
            vals.push_back(Double2Json(v.rx));
            vals.push_back(Double2Json(v.ry));
            vals.push_back(Double2Json(v.rz));
            param["goal"].push_back(vals);
        }

        if (mode == MoveMode::abs)
        {
            param["mode"] = "abs";
        }
        else
        {
            param["mode"] = "incr";
        }
        return param;
    }

    Result<double> ApiArm::CallMoveJTask(const nlohmann::json &param)
    {
        auto ret = Call("SetMoveJTask", param);

        Result<double> time;
        if (ret.first == RetState::ok)
        {
            try
            {
                time.second = ret.second;
                time.first = RetState::ok;
            }
            catch (const std::exception &)
            {
                try
                {
                    time.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    time.first = RetState::parseErr;
                }
            }
        }
        else
        {
            time.first = ret.first;
        }

        return time;
    }

    Result<double> ApiArm::CallMoveLTask(const nlohmann::json &param)
    {
        auto ret = Call("SetMoveLTask", param);
        Result<double> time;
        if (ret.first == RetState::ok)
        {
            try
            {
                time.second = ret.second;
                time.first = RetState::ok;
            }
            catch (const std::exception &)
            {
                try
                {
                    time.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    time.first = RetState::parseErr;
                }
            }
        }
        else
        {
            time.first = ret.first;
        }

        return time;
    }

    /**
     * @description: 设置关节运动
     * @param jointVal
     * @param mode
     * @param speed
     * @param acc
     * @return {} 执行时间
     */

    Result<uint32_t> ApiArm::SetMoveJTask(const std::vector<std::vector<double>> &jointVal, MoveMode mode)
    {
        return CallMoveJTask(GetMoveJParam(jointVal, mode));
    }

    Result<uint32_t> ApiArm::SetMoveJTask(const std::vector<vector<double>> &jointVal, MoveMode mode, double speed)
    {
        nlohmann::json param = GetMoveJParam(jointVal, mode);
        param["speed"] = Double2Json(speed);
        return CallMoveJTask(param);
    }
    Result<uint32_t> ApiArm::SetMoveJTask(const std::vector<std::vector<double>> &jointVal, MoveMode mode, double speed, double acc)
    {
        nlohmann::json param = GetMoveJParam(jointVal, mode);
        param["speed"] = Double2Json(speed);
        param["acc"] = Double2Json(acc);
        return CallMoveJTask(param);
    }

    /**
     * @description: 设置笛卡尔运动
     * @param jointVal
     * @param mode
     * @param speed
     * @param acc
     * @return {} 执行时间
     */
    Result<uint32_t> ApiArm::SetMoveLTask(const std::vector<CartesianPose> &jointVal, MoveMode mode)
    {
        return CallMoveLTask(GetMoveLParam(jointVal, mode));
    }
    Result<uint32_t> ApiArm::SetMoveLTask(const std::vector<CartesianPose> &jointVal, MoveMode mode, double speed)
    {
        nlohmann::json param = GetMoveLParam(jointVal, mode);
        param["speed"] = Double2Json(speed);
        return CallMoveLTask(param);
    }
    Result<uint32_t> ApiArm::SetMoveLTask(const std::vector<CartesianPose> &jointVal, MoveMode mode, double speed, double acc)
    {
        nlohmann::json param = GetMoveLParam(jointVal, mode);
        param["speed"] = Double2Json(speed);
        param["acc"] = Double2Json(acc);
        return CallMoveLTask(param);
    }

    RetState ApiArm::WaitTaskComplete(uint32_t timeout)
    {
        nlohmann::json param = timeout;

        Result<nlohmann::json> ret = Call("WaitTaskComplete", param, timeout);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }

    /**
     * @description: 设置任务控制状态
     * @param set
     * @param timeout 超时时间，若为0，表示使用非阻塞的默认超时时间
     * @return {}
     */
    RetState ApiArm::SetCtrlState(ArmCtrl set)
    {
        nlohmann::json param;

        switch (set)
        {
        case ArmCtrl::start:
            param = "start";
            break;
        case ArmCtrl::stop:
            param = "stop";
            break;
        case ArmCtrl::pause:
            param = "pause";
            break;
        case ArmCtrl::enable:
            param = "enable";
            break;
        case ArmCtrl::disable:
            param = "disable";
            break;
        default:
            break;
        }

        Result<nlohmann::json> ret = Call("SetCtrlState", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }

    /**
     * @description: 获取工具名字列表
     * @return {*}
     */
    Result<std::vector<std::string>> ApiArm::GetToolNameList()
    {
        nlohmann::json param;
        Result<std::vector<std::string>> tools;

        auto ret = Call("GetToolNameList", param);
        if (ret.first == RetState::ok)
        {
            try
            {
                for (const auto &v : ret.second)
                {
                    tools.second.push_back(v);
                }
            }
            catch (const std::exception &e)
            {
                try
                {
                    tools.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    tools.first = RetState::parseErr;
                }
            }
        }
        else
        {
            tools.first = ret.first;
        }

        return tools;
    }

    /**
     * @description: 设置当前使用的工具坐标系
     * @param &name
     * @return {}
     */
    RetState ApiArm::SetToolName(const std::string &name)
    {
        RetState ret;
        try
        {
            nlohmann::json param = name;
            auto jRet = Call("SetToolName", param);
            if (jRet.first == RetState::ok)
            {
                ret = Json2errState(jRet.second);
            }
            else
            {
                ret = jRet.first;
            }
        }
        catch (const std::exception &e)
        {
            ret = RetState::parseErr;
        }
        return ret;
    }

    /**
     * @description: 读取当前的工具坐标系名称
     * @return {}
     */
    Result<std::string> ApiArm::GetToolName()
    {
        nlohmann::json param;
        Result<std::string> tool;

        auto ret = Call("GetToolName", param);
        if (ret.first == RetState::ok)
        {
            try
            {
                tool.second = ret.second;
            }
            catch (const std::exception &e)
            {
                try
                {
                    tool.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    tool.first = RetState::parseErr;
                }
            }
        }
        else
        {
            tool.first = ret.first;
        }

        return tool;
    }

    Result<CartesianPose> ApiArm::GetToolValue(const std::string &name)
    {
        nlohmann::json param = name;
        Result<CartesianPose> tool;

        auto ret = Call("GetToolValue", param);
        if (ret.first == RetState::ok)
        {
            try
            {
                tool.second.x = Json2Double(ret.second.at(0));
                tool.second.y = Json2Double(ret.second.at(1));
                tool.second.z = Json2Double(ret.second.at(2));
                tool.second.rx = Json2Double(ret.second.at(3));
                tool.second.ry = Json2Double(ret.second.at(4));
                tool.second.rz = Json2Double(ret.second.at(5));
                tool.first = ret.first;
            }
            catch (const std::exception &e)
            {
                try
                {
                    tool.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    tool.first = RetState::parseErr;
                }
            }
        }
        else
        {
            tool.first = ret.first;
        }
        return tool;
    }

    RetState ApiArm::SetToolValue(const std::string &name, const CartesianPose &pose)
    {
        RetState ret;

        try
        {
            nlohmann::json param;
            param["name"] = name;
            param["pos"].push_back(Double2Json(pose.x));
            param["pos"].push_back(Double2Json(pose.y));
            param["pos"].push_back(Double2Json(pose.z));
            param["pos"].push_back(Double2Json(pose.rx));
            param["pos"].push_back(Double2Json(pose.ry));
            param["pos"].push_back(Double2Json(pose.rz));
            auto jRet = Call("SetToolValue", param);
            if (jRet.first == RetState::ok)
            {
                return jRet.second;
            }
            else
            {
                ret = jRet.first;
            }
        }
        catch (const std::exception &e)
        {
            ret = RetState::parseErr;
        }
        return ret;
    }

    Result<std::vector<double>> ApiArm::GetNowAngle()
    {
        nlohmann::json param;
        Result<std::vector<double>> angle;

        auto ret = Call("GetNowAngle", param);
        if (ret.first == RetState::ok)
        {
            try
            {
                for (const auto &ang : ret.second)
                {
                    angle.second.push_back(Json2Double(ang));
                }
            }
            catch (const std::exception &e)
            {
                try
                {
                    angle.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    angle.first = RetState::parseErr;
                }
            }
        }
        else
        {
            angle.first = ret.first;
        }
        return angle;
    }

    Result<CartesianPose> ApiArm::GetNowCart()
    {
        nlohmann::json param;
        Result<CartesianPose> cart;

        auto ret = Call("GetNowCart", param);
        if (ret.first == RetState::ok)
        {
            try
            {
                cart.second.x = Json2Double(ret.second.at(0));
                cart.second.y = Json2Double(ret.second.at(1));
                cart.second.z = Json2Double(ret.second.at(2));
                cart.second.rx = Json2Double(ret.second.at(3));
                cart.second.ry = Json2Double(ret.second.at(4));
                cart.second.rz = Json2Double(ret.second.at(5));
            }
            catch (const std::exception &e)
            {
                try
                {
                    cart.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    cart.first = RetState::parseErr;
                }
            }
        }
        else
        {
            cart.first = ret.first;
        }
        return cart;
    }
    Result<ArmState> ApiArm::GetArmState()
    {
        nlohmann::json param;
        Result<ArmState> sta;

        auto ret = Call("GetArmState", param);
        if (ret.first == RetState::ok)
        {
            try
            {
                if (STRING_ARMSTATE.count(ret.second))
                {
                    sta.second = STRING_ARMSTATE.at(ret.second);
                    sta.first = RetState::ok;
                }
                else
                {
                    sta.first = RetState::noSupport;
                }
            }
            catch (const std::exception &e)
            {
                try
                {
                    sta.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    sta.first = RetState::parseErr;
                }
            }
        }
        else
        {
            sta.first = ret.first;
        }
        return sta;
    }

    /**
     * @description: 订阅和回调
     * @param func
     * @return {}
     */
    RetState ApiArm::SubscribeArmState(std::function<void(ArmState)> func)
    {
        pubArmState_ = func;
        return rpcCli_->SetSubscribe("arm", "PubArmState", [this](const nlohmann::json &ret)
                                     { this->RxPubArmState(ret); });
    }
    void ApiArm::RxPubArmState(const nlohmann::json &ret)
    {
        ArmState sta;
        try
        {
            if (STRING_ARMSTATE.count(ret))
            {
                sta = STRING_ARMSTATE.at(ret);
            }
            else
            {
                return;
            }
        }
        catch (const std::exception &e)
        {
            return;
        }

        if (pubArmState_)
        {
            pubArmState_(sta);
        }
    }

    Result<CartesianPose> ApiArm::GetFkine(const std::array<double, 6> &robotjoint)
    {
        nlohmann::json param;
        std::vector<std::string> tmpjoint;
        Result<CartesianPose> pose;
        try
        {
            for (int i = 0; i < 6; i++)
            {
                tmpjoint.push_back(Double2Json(robotjoint.at(i)));
            }
            param["joint"] = tmpjoint;
        }
        catch (const std::exception &)
        {
            pose.first = RetState::parseErr;
            return pose;
        }
        auto ret = Call("GetFkine6", param);

        if (ret.first == RetState::ok)
        {
            try
            {
                pose.second.x = Json2Double(ret.second.at(0));
                pose.second.y = Json2Double(ret.second.at(1));
                pose.second.z = Json2Double(ret.second.at(2));
                pose.second.rx = Json2Double(ret.second.at(3));
                pose.second.ry = Json2Double(ret.second.at(4));
                pose.second.rz = Json2Double(ret.second.at(5));
                pose.first = RetState::ok;
            }
            catch (const std::exception &)
            {
                try
                {
                    pose.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    pose.first = RetState::parseErr;
                }
            }
        }
        else
        {
            pose.first = ret.first;
        }

        return pose;
    }
    Result<std::array<double, 6>> ApiArm::GetIkine(const CartesianPose &FK)
    {
        nlohmann::json param;
        Result<std::array<double, 6>> pose;
        try
        {
            param["FK"].push_back(Double2Json(FK.x));
            param["FK"].push_back(Double2Json(FK.y));
            param["FK"].push_back(Double2Json(FK.z));
            param["FK"].push_back(Double2Json(FK.rx));
            param["FK"].push_back(Double2Json(FK.ry));
            param["FK"].push_back(Double2Json(FK.rz));
        }
        catch (const std::exception &)
        {
            pose.first = RetState::parseErr;
            return pose;
        }

        auto ret = Call("GetIkine6", param);

        if (ret.first == RetState::ok)
        {
            try
            {
                for (int i = 0; i < 6; i++)
                {
                    pose.second[i] = Json2Double(ret.second.at(i));
                }
                pose.first = RetState::ok;
            }
            catch (const std::exception &)
            {
                try
                {
                    pose.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    pose.first = RetState::parseErr;
                }
            }
        }
        else
        {
            pose.first = ret.first;
        }

        return pose;
    }

    Result<CartesianPose7> ApiArm::GetFkine7(const std::array<double, 7> &robotjoint)
    {
        nlohmann::json param;
        Result<CartesianPose7> pose;
        try
        {
            for (int i = 0; i < 7; i++)
            {
                param.push_back(Double2Json(robotjoint.at(i)));
            }
        }
        catch (const std::exception &)
        {
            pose.first = RetState::parseErr;
            return pose;
        }
        auto ret = Call("GetFkine7", param);

        if (ret.first == RetState::ok)
        {
            try
            {
                pose.second.x = Json2Double(ret.second.at(0));
                pose.second.y = Json2Double(ret.second.at(1));
                pose.second.z = Json2Double(ret.second.at(2));
                pose.second.rx = Json2Double(ret.second.at(3));
                pose.second.ry = Json2Double(ret.second.at(4));
                pose.second.rz = Json2Double(ret.second.at(5));
                pose.second.phi = Json2Double(ret.second.at(6));
                pose.first = RetState::ok;
            }
            catch (const std::exception &)
            {
                try
                {
                    pose.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    pose.first = RetState::parseErr;
                }
            }
        }
        else
        {
            pose.first = ret.first;
        }

        return pose;
    }
    Result<std::array<double, 7>> ApiArm::GetIkine7(const CartesianPose7 &FK)
    {
        nlohmann::json param;
        Result<std::array<double, 7>> pose;
        try
        {
            param.push_back(Double2Json(FK.x));
            param.push_back(Double2Json(FK.y));
            param.push_back(Double2Json(FK.z));
            param.push_back(Double2Json(FK.rx));
            param.push_back(Double2Json(FK.ry));
            param.push_back(Double2Json(FK.rz));
            param.push_back(Double2Json(FK.phi));
        }
        catch (const std::exception &)
        {
            pose.first = RetState::parseErr;
            return pose;
        }

        auto ret = Call("GetIkine7", param);

        if (ret.first == RetState::ok)
        {
            try
            {
                for (int i = 0; i < 7; i++)
                {
                    pose.second[i] = Json2Double(ret.second.at(i));
                }
                pose.first = RetState::ok;
            }
            catch (const std::exception &)
            {
                try
                {
                    pose.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    pose.first = RetState::parseErr;
                }
            }
        }
        else
        {
            pose.first = ret.first;
        }

        return pose;
    }

    RetState ApiArm::SetIoOutputState(const string &name, std::bitset<32> set)
    {
        if ((IO_NAME.count(name) == 0) || (set.size() > 32))
        {
            return RetState::noSupport;
        }

        nlohmann::json param;

        param["name"] = name;
        param["sta"] = set.to_ulong();

        auto ret = Call("SetIoOutState", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }

    Result<std::bitset<32>> ApiArm::GetIoInputState(const std::string &name)
    {
        if (IO_NAME.count(name) == 0)
        {
            return {RetState::noSupport, {}};
        }

        auto ret = Call("GetIoInState", name);

        Result<std::bitset<32>> sta;
        if (ret.first == RetState::ok)
        {
            try
            {
                sta.second = std::bitset<32>(ret.second);
                sta.first = RetState::ok;
            }
            catch (const std::exception &)
            {
                try
                {
                    sta.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    sta.first = RetState::parseErr;
                }
            }
        }
        else
        {
            sta.first = ret.first;
        }

        return sta;
    }

    Result<std::bitset<32>> ApiArm::GetIoOutputState(const std::string &name)
    {
        if (IO_NAME.count(name) == 0)
        {
            return {RetState::noSupport, {}};
        }

        auto ret = Call("GetIoOutState", name);

        Result<std::bitset<32>> sta;
        if (ret.first == RetState::ok)
        {
            try
            {
                sta.second = std::bitset<32>(ret.second);
                sta.first = RetState::ok;
            }
            catch (const std::exception &)
            {
                try
                {
                    sta.first = Json2errState(ret.second);
                }
                catch (const std::exception &e)
                {
                    sta.first = RetState::parseErr;
                }
            }
        }
        else
        {
            sta.first = ret.first;
        }

        return sta;
    }

    RetState ApiArm::SubscribeIoInputState(std::function<void(const std::map<std::string, std::bitset<32>> &)> func)
    {
        pubIoInputState_ = func;
        return rpcCli_->SetSubscribe("arm", "PubIoInState", [this](const nlohmann::json &ret)
                                     { this->RxPubIoInputState(ret); });
    }

    void ApiArm::RxPubIoInputState(const nlohmann::json &ret)
    {
        std::map<std::string, std::bitset<32>> iosta;
        try
        {
            iosta["panel"] = bitset<32>(ret.at("panel"));
            iosta["tool"] = bitset<32>(ret.at("tool"));
            iosta["extra"] = bitset<32>(ret.at("extra"));
        }
        catch (const std::exception &)
        {
            return;
        }
        if (pubIoInputState_)
        {
            pubIoInputState_(iosta);
        }
    }

    RetState ApiArm::SubscribeIoOutputState(std::function<void(const std::map<std::string, std::bitset<32>> &)> func)
    {
        pubIoOutputState_ = func;
        return rpcCli_->SetSubscribe("arm", "PubIoOutState", [this](const nlohmann::json &ret)
                                     { this->RxPubIoOutputState(ret); });
    }

    void ApiArm::RxPubIoOutputState(const nlohmann::json &ret)
    {
        std::map<std::string, std::bitset<32>> iosta;
        try
        {
            iosta["panel"] = bitset<32>(ret.at("panel"));
            iosta["tool"] = bitset<32>(ret.at("tool"));
            iosta["extra"] = bitset<32>(ret.at("extra"));
        }
        catch (const std::exception &)
        {
            return;
        }
        if (pubIoOutputState_)
        {
            pubIoOutputState_(iosta);
        }
    }

    RetState ApiArm::SetPositionLimit(const std::vector<ValueRange<double>> &jLimit)
    {
        nlohmann::json param;
        for (const auto &v : jLimit)
        {
            param.push_back({Double2Json(v.min), Double2Json(v.max)});
        }

        auto ret = Call("SetPositionLimit", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }

    Result<std::vector<ValueRange<double>>> ApiArm::GetPositionLimit()
    {
        nlohmann::json param;
        auto ret = Call("GetPositionLimit", param);

        std::vector<ValueRange<double>> pos;
        if (ret.first == RetState::ok)
        {
            try
            {
                for (const auto &v : ret.second)
                {
                    ValueRange<double> tmp{Json2Double(v.at(0)), Json2Double(v.at(1))};
                    pos.push_back(tmp);
                }
                return {RetState::ok, pos};
            }
            catch (const std::exception &)
            {
                return {RetState::parseErr, pos};
            }
        }
        return {RetState::netErr, pos};
    }

    RetState ApiArm::SubscribeNowAngle(std::function<void(const std::vector<double> &)> func)
    {
        pubNowAngle_ = func;
        return rpcCli_->SetSubscribe("arm", "PubNowAngle", [this](const nlohmann::json &ret)
                                     { this->RxPubNowAngle(ret); });
    }

    void ApiArm::RxPubNowAngle(const nlohmann::json &ret)
    {
        std::vector<double> angle;
        try
        {
            for (const auto &v : ret)
            {
                angle.push_back(Json2Double(v));
            }
        }
        catch (const std::exception &)
        {
            return;
        }
        if (pubNowAngle_)
        {
            pubNowAngle_(angle);
        }
    }

    RetState ApiArm::SubscribeNowCart(std::function<void(const CartesianPose &)> func)
    {
        pubNowCart_ = func;
        return rpcCli_->SetSubscribe("arm", "PubNowCart", [this](const nlohmann::json &ret)
                                     { this->RxPubNowCart(ret); });
    }

    void ApiArm::RxPubNowCart(const nlohmann::json &ret)
    {
        CartesianPose pose;
        try
        {
            pose.x = Json2Double(ret.at(0));
            pose.y = Json2Double(ret.at(1));
            pose.z = Json2Double(ret.at(2));
            pose.rx = Json2Double(ret.at(3));
            pose.ry = Json2Double(ret.at(4));
            pose.rz = Json2Double(ret.at(5));
            if (pubNowCart_)
            {
                pubNowCart_(pose);
            }
        }
        catch (const std::exception &)
        {
            return;
        }
    }

    RetState ApiArm::SetDragMode(bool set)
    {
        nlohmann::json param;
        param = set;

        auto ret = Call("SetDragMode", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }

    Result<bool> ApiArm::GetDragMode()
    {
        nlohmann::json param;
        auto ret = Call("GetDragMode", param);

        bool mode;
        if (ret.first == RetState::ok)
        {
            try
            {
                mode = ret.second;
                return {RetState::ok, mode};
            }
            catch (const std::exception &)
            {
                return {RetState::parseErr, mode};
            }
        }
        return {RetState::netErr, mode};
    }

    RetState ApiArm::SetCollisionLevel(int level)
    {
        nlohmann::json param;
        param = level;

        auto ret = Call("SetCollisionLevel", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }

    Result<int> ApiArm::GetCollisionLevel()
    {
        nlohmann::json param;
        auto ret = Call("GetCollisionLevel", param);

        int level;
        if (ret.first == RetState::ok)
        {
            try
            {
                level = ret.second;
            }
            catch (const std::exception &)
            {
                return {RetState::parseErr, level};
            }
            return {RetState::ok, level};
        }
        else
        {
            return {ret.first, level};
        }
    }

    RetState ApiArm::SetRapidrate(double rate)
    {
        nlohmann::json param;
        param = Double2Json(rate);

        auto ret = Call("SetRapidrate", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }

    Result<double> ApiArm::GetRapidrate()
    {
        nlohmann::json param;
        auto ret = Call("GetRapidrate", param);
        if (ret.first == RetState::ok)
        {
            try
            {
                return {RetState::ok, Json2Double(ret.second)};
            }
            catch (const std::exception &)
            {
                return {RetState::parseErr, 0};
            }
        }
        else
        {
            return {ret.first, 0};
        }
    }

    RetState ApiArm::SetJointSpeedLimit(const std::vector<double> &vMax)
    {
        nlohmann::json param;
        for (const auto &v : vMax)
        {
            param.push_back(Double2Json(v));
        }

        auto ret = Call("SetJointSpeedLimit", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }

    Result<std::vector<double>> ApiArm::GetJointSpeedLimit()
    {
        nlohmann::json param;
        auto ret = Call("GetJointSpeedLimit", param);

        std::vector<double> vMax;
        if (ret.first == RetState::ok)
        {
            try
            {
                for (const auto &v : ret.second)
                {
                    vMax.push_back(Json2Double(v));
                }
                return {ret.first, vMax};
            }
            catch (const std::exception &)
            {
                return {RetState::parseErr, vMax};
            }
        }
        else
        {
            return {ret.first, vMax};
        }
    }

    RetState ApiArm::SetJointAccLimit(const std::vector<double> &aMax)
    {
        nlohmann::json param;
        for (const auto &v : aMax)
        {
            param.push_back(Double2Json(v));
        }

        auto ret = Call("SetJointAccLimit", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        else
        {
            return ret.first;
        }
    }

    Result<std::vector<double>> ApiArm::GetJointAccLimit()
    {
        nlohmann::json param;
        auto ret = Call("GetJointAccLimit", param);

        std::vector<double> aMax;
        if (ret.first == RetState::ok)
        {
            try
            {
                for (const auto &v : ret.second)
                {
                    aMax.push_back(Json2Double(v));
                }
                return {ret.first, aMax};
            }
            catch (const std::exception &)
            {
                return {RetState::parseErr, aMax};
            }
        }
        else
        {
            return {ret.first, aMax};
        }
    }

    Result<ArmInfo> ApiArm::GetArmInfo()
    {
        nlohmann::json param;
        auto ret = Call("GetArmInfo", param);
        if (ret.first == RetState::ok)
        {
            try
            {
                ArmInfo info;
                for (int i = 0; i < 6; i++)
                {
                    info.current.push_back(Json2Double(ret.second.at("current").at(i)));
                    info.voltage.push_back(Json2Double(ret.second.at("voltage").at(i)));
                    info.torque.push_back(Json2Double(ret.second.at("torque").at(i)));
                    info.temperature.push_back(Json2Double(ret.second.at("temperature").at(i)));
                }

                return {RetState::ok, info};
            }
            catch (const std::exception &)
            {
                return {RetState::parseErr, ArmInfo()};
            }
        }
        else
        {
            return {ret.first, ArmInfo()};
        }
    }

    RetState ApiArm::SubscribeArmInfo(std::function<void(const ArmInfo &)> func)
    {
        pubArmInfo_ = func;
        return rpcCli_->SetSubscribe("arm", "PubArmInfo", [this](const nlohmann::json &ret)
                                     { this->RxPubArmInfo(ret); });
    }

    void ApiArm::RxPubArmInfo(const nlohmann::json &ret)
    {
        ArmInfo info;
        try
        {
            for (int i = 0; i < 6; i++)
            {
                info.current.push_back(Json2Double(ret.at("current").at(i)));
                info.voltage.push_back(Json2Double(ret.at("voltage").at(i)));
                info.torque.push_back(Json2Double(ret.at("torque").at(i)));
                info.temperature.push_back(Json2Double(ret.at("temperature").at(i)));
            }
            if (pubArmInfo_)
            {
                pubArmInfo_(info);
            }
        }
        catch (const std::exception &)
        {
            return;
        }
    }

    RetState ApiArm::SetReset(bool set)
    {
        nlohmann::json param;
        param = set;

        auto ret = Call("SetReset", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }

    Result<bool> ApiArm::GetReset()
    {
        nlohmann::json param;
        auto ret = Call("GetReset", param);

        bool mode;
        if (ret.first == RetState::ok)
        {
            try
            {
                mode = ret.second;
                return {RetState::ok, mode};
            }
            catch (const std::exception &)
            {
                return {RetState::parseErr, mode};
            }
        }
        return {RetState::netErr, mode};
    }

    RetState ApiArm::SetGripper(bool state, double speed)
    {
        nlohmann::json param;
        param["state"] = state;
        param["speed"] = Double2Json(speed);
        auto ret = Call("SetGripper", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }
}