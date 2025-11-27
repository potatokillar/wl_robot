/*
 * @Author: 唐文浩
 * @Date: 2024-04-15
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description: 机械臂协议实现，此处API和协议一一对应
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <bitset>

#include "armTask.hpp"
#include "sdkProtocolClient.hpp"
#include "type/armType.hpp"

namespace iiri::arm
{
    enum class ArmCtrl
    {
        start,
        stop,
        pause,
        enable,
        disable,
    };
    class ApiArm
    {
    public:
        ApiArm(std::shared_ptr<SdkProtocol> rpcCli);

        Result<uint32_t> SetMoveJTask(const std::vector<std::vector<double>> &jointVal, MoveMode mode);
        Result<uint32_t> SetMoveJTask(const std::vector<std::vector<double>> &jointVal, MoveMode mode, double speed);
        Result<uint32_t> SetMoveJTask(const std::vector<std::vector<double>> &jointVal, MoveMode mode, double speed, double acc);

        Result<uint32_t> SetMoveLTask(const std::vector<CartesianPose> &jointVal, MoveMode mode);
        Result<uint32_t> SetMoveLTask(const std::vector<CartesianPose> &jointVal, MoveMode mode, double speed);
        Result<uint32_t> SetMoveLTask(const std::vector<CartesianPose> &jointVal, MoveMode mode, double speed, double acc);

        RetState WaitTaskComplete(uint32_t timeout);

        RetState SetCtrlState(ArmCtrl set);

        Result<std::vector<std::string>> GetToolNameList();
        RetState SetToolName(const std::string &name);
        Result<std::string> GetToolName();
        RetState SetToolValue(const std::string &name, const CartesianPose &pose);
        Result<CartesianPose> GetToolValue(const std::string &name);

        Result<std::vector<double>> GetNowAngle();
        RetState SubscribeNowAngle(std::function<void(const std::vector<double> &)> func);

        Result<CartesianPose> GetNowCart();
        RetState SubscribeNowCart(std::function<void(const CartesianPose &)> func);

        Result<ArmState> GetArmState();
        RetState SubscribeArmState(std::function<void(ArmState)> func);

        Result<CartesianPose> GetFkine(const std::array<double, 6> &robotjoint);
        Result<std::array<double, 6>> GetIkine(const CartesianPose &FK);

        Result<CartesianPose7> GetFkine7(const std::array<double, 7> &robotjoint);
        Result<std::array<double, 7>> GetIkine7(const CartesianPose7 &FK);

        RetState SetIoOutputState(const std::string &name, std::bitset<32> set);
        Result<std::bitset<32>> GetIoInputState(const std::string &name);
        Result<std::bitset<32>> GetIoOutputState(const std::string &name);

        RetState SubscribeIoInputState(std::function<void(const std::map<std::string, std::bitset<32>> &)> func);
        RetState SubscribeIoOutputState(std::function<void(const std::map<std::string, std::bitset<32>> &)> func);

        RetState SetPositionLimit(const std::vector<ValueRange<double>> &jLimit);
        Result<std::vector<ValueRange<double>>> GetPositionLimit();

        RetState SetCollisionLevel(int level);
        Result<int> GetCollisionLevel();

        Result<ArmInfo> GetArmInfo();
        RetState SubscribeArmInfo(std::function<void(const ArmInfo &)> func);

        RetState SetJointSpeedLimit(const std::vector<double> &vMax);
        Result<std::vector<double>> GetJointSpeedLimit();

        RetState SetJointAccLimit(const std::vector<double> &aMax);
        Result<std::vector<double>> GetJointAccLimit();

        RetState SetDragMode(bool set);
        Result<bool> GetDragMode();

        RetState SetRapidrate(double rate);
        Result<double> GetRapidrate();

        RetState SetReset(bool set);
        Result<bool> GetReset();

        RetState SetGripper(bool state, double speed);

    private:
        Result<nlohmann::json> Call(const std::string &method, const nlohmann::json &params);
        Result<nlohmann::json> Call(const std::string &method, const nlohmann::json &params, uint32_t ms);
        std::shared_ptr<SdkProtocol> rpcCli_;

        void RxPubArmState(const nlohmann::json &ret);
        std::function<void(ArmState)> pubArmState_;

        void RxPubIoInputState(const nlohmann::json &ret);
        void RxPubIoOutputState(const nlohmann::json &ret);
        std::function<void(const std::map<std::string, std::bitset<32>> &)> pubIoInputState_, pubIoOutputState_;

        void RxPubNowAngle(const nlohmann::json &ret);
        std::function<void(const std::vector<double> &)> pubNowAngle_;

        void RxPubNowCart(const nlohmann::json &ret);
        std::function<void(const CartesianPose &)> pubNowCart_;

        void RxPubArmInfo(const nlohmann::json &ret);
        std::function<void(const ArmInfo &)> pubArmInfo_;

        std::map<std::string, ArmState> STRING_ARMSTATE;

        nlohmann::json GetMoveJParam(const std::vector<std::vector<double>> &jointVals, MoveMode mode);
        nlohmann::json GetMoveLParam(const std::vector<CartesianPose> &jointVals, MoveMode mode);

        Result<double> CallMoveJTask(const nlohmann::json &param);
        Result<double> CallMoveLTask(const nlohmann::json &param);
    };
} // namespace iiri::arm