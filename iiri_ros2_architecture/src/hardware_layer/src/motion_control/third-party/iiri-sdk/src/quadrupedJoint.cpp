/*
 * @Author: 唐文浩-0036
 * @Date: 2023-07-06
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-18
 * @Description:
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "quadrupedJoint.hpp"

#include <iostream>

#include "apiDevice.hpp"
#include "apiQuadruped.hpp"
#include "metaData.hpp"
#include "protocol.hpp"
// #include "udpClient.hpp"

using namespace std;
namespace iiri::qr
{
    QuadrupedJoint::QuadrupedJoint(const std::string &ip)
    {
        rpcCli_ = make_shared<SdkProtocol>(ip);
        apiDev_ = make_unique<ApiDevice>(rpcCli_);
        apiQuad_ = make_unique<ApiQuadruped>(rpcCli_);
        SetJointState(true);
    }

    QuadrupedJoint::~QuadrupedJoint() { SetJointState(false); }

    QuadrupedJoint::QuadrupedJoint(QuadrupedJoint &&rhs) = default;
    QuadrupedJoint &QuadrupedJoint::operator=(QuadrupedJoint &&rhs) = default;

    // unique_ptr不支持拷贝，需要特殊定义
    // shared_ptr支持拷贝操作，可以直接复制
    QuadrupedJoint::QuadrupedJoint(const QuadrupedJoint &rhs)
    {
        rpcCli_ = rhs.rpcCli_;
        apiDev_ = make_unique<ApiDevice>(*rhs.apiDev_);
    }
    QuadrupedJoint &QuadrupedJoint::operator=(const QuadrupedJoint &rhs)
    {
        *apiDev_ = *rhs.apiDev_; // 必须是值赋值，因为unique_ptr不支持复制
        rpcCli_ = rhs.rpcCli_;   // 可以直接指针赋值，因为shared_ptr原生支持复制
        return *this;
    }

    /**
     * @description: 元数据
     * @return {}
     */
    const std::string_view &QuadrupedJoint::meta() { return SDK_VERSION; }

    /**
     * @description: 设置进入关节级控制模式
     * @param set
     * @return {}
     */
    RetState QuadrupedJoint::SetJointState(bool set)
    {
        nlohmann::json param;
        if (set == true)
        {
            param = "lowStart";
        }
        else
        {
            param = "stop";
        }
        auto ret = rpcCli_->Call("quadruped", "SetRobotCtrl", param);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }

    void QuadrupedJoint::SetMotorCmd(const MotorCmd &cmd) { apiQuad_->SetMotorCmd(cmd); }

    Result<MotorRet> QuadrupedJoint::GetMotorData() { return apiQuad_->GetMotorData(); }
    Result<ImuRet> QuadrupedJoint::GetImuData() { return apiQuad_->GetImuData(); }

    RetState QuadrupedJoint::SubscribetMotorData(std::function<void(const MotorRet &)> func) { return apiQuad_->SubscribMotorData(func); }
    RetState QuadrupedJoint::SubscribetImuData(std::function<void(const ImuRet &)> func) { return apiQuad_->SubscribImuData(func); }
} // namespace iiri::qr