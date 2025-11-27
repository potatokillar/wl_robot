/*
 * @Author: 唐文浩
 * @Date: 2022-08-09
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-03-03
 * @Description: quadruped命名空间的API
 * 该文档和quadruped.hpp的区别是，该文档下的每一个函数对应具体的协议
 * 但quadruped.hpp表示对外的四足接口，其下面并不全是本文档的API
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include "sdkProtocolClient.hpp"
#include "type/qrType.hpp"

namespace iiri::qr
{
    class ApiQuadruped
    {
    public:
        ApiQuadruped(std::shared_ptr<SdkProtocol> rpcCli);

        RetState SetLinearVelocity(double x, double y, double z);
        RetState GetLinearVelocity(double &x, double &y, double &z);
        RetState GetLinearVelocityInfo(ValueRange<double> &x, ValueRange<double> &y, ValueRange<double> &z);
        RetState SubscribeLinearVelocity(std::function<void(const LinearVelocityData &)> func);

        RetState SetAngularVelocity(double x, double y, double z);
        RetState GetAngularVelocity(double &x, double &y, double &z);
        RetState GetAngularVelocityInfo(ValueRange<double> &x, ValueRange<double> &y, ValueRange<double> &z);
        RetState SubscribeAngularVelocity(std::function<void(const AngularVelocityData &)> func);

        RetState SetPose(double roll, double pitch, double yaw);
        RetState GetPose(double &roll, double &pitch, double &yaw);
        RetState GetPoseInfo(ValueRange<double> &roll, ValueRange<double> &pitch, ValueRange<double> &yaw);
        RetState SubscribePose(std::function<void(const PoseData &)> func);

        RetState SetHeight(double set);
        RetState GetHeight(double &height);
        RetState GetHeightInfo(ValueRange<double> &height);
        RetState SubscribeHeight(std::function<void(double)> func);

        RetState SetRunState(RunState sta);
        RetState GetRunState(RunState &sta);
        RetState GetRunStateInfo(std::vector<std::string> &runSta);
        RetState SubscribeRunState(std::function<void(RunState)> func);

        RetState SetWalkMode(WalkMode mode);
        RetState GetWalkMode(WalkMode &mode);
        RetState GetWalkModeInfo(std::vector<std::string> &runMode);
        RetState SubscribeWalkMode(std::function<void(WalkMode)> func);

        RetState SetRobotCtrl(RobotCtrl sta);
        RetState GetRobotState(RobotState &sta);
        RetState GetRobotCtrlInfo(std::vector<std::string> &motor);
        RetState SubscribetRobotState(std::function<void(RobotState)> func);

        RetState SetHoldFlag(bool set);
        RetState GetHoldFlag(bool &set);

        RetState SetLoadMass(double set);
        RetState GetLoadMass(double &height);
        RetState GetLoadMassInfo(ValueRange<double> &height);

        Result<ImuRet> GetImuData();
        RetState SubscribImuData(std::function<void(const ImuRet &)> func);

        RetState SetDance(const std::string &name, uint32_t ms);

        RetState SetMotorCmd(const MotorCmd &cmd);
        Result<MotorRet> GetMotorData();
        RetState SubscribMotorData(std::function<void(const MotorRet &)> func);

    private:
        Result<nlohmann::json> Call(const std::string &method, const nlohmann::json &params);
        std::shared_ptr<SdkProtocol> rpcCli_;

    private:
        void RxPubLinearVelocity(const nlohmann::json &ret);
        void RxPubAngularVelocity(const nlohmann::json &ret);
        void RxPubPose(const nlohmann::json &ret);
        void RxPubHeight(const nlohmann::json &ret);
        void RxPubRunState(const nlohmann::json &ret);
        void RxPubWalkMode(const nlohmann::json &ret);
        void RxPubImuData(const nlohmann::json &ret);
        void RxPubRobotState(const nlohmann::json &ret);
        void RxPubMotorData(const nlohmann::json &ret);

        std::function<void(const LinearVelocityData &)> pubLinearVelocity_;
        std::function<void(const AngularVelocityData &)> pubAngularVelocity_;
        std::function<void(const PoseData &)> pubPose_;
        std::function<void(double)> pubHeight_;
        std::function<void(RunState)> pubRunState_;
        std::function<void(WalkMode)> pubWalkMode_;
        std::function<void(const ImuRet &)> pubImuData_;
        std::function<void(RobotState)> pubRobotState_;
        std::function<void(const MotorRet &)> pubMotorData_;
    };
} // namespace iiri::qr
