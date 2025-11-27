/*
 * @Author: 唐文浩-0036
 * @Date: 2023-07-06
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description: 关节级的命令控制
 * 该系列命令和Quadruped下的高级命令互斥，机器人默认进入高级命令状态，若要进入该关节命令状态
 * 需要调用device下的API开启，为保证安全，只允许机器人在趴下状态才能进行切换。
 * 析构该类时，自动离开关节命令控制状态
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#pragma once

#include <functional>
#include <memory>
#include <string_view>

#include "type/qrType.hpp"

namespace iiri
{
    class ApiDevice;
    class SdkProtocol;

    // 四足有关的均处于qr命名空间
    namespace qr
    {
        class ApiQuadruped;
        class QuadrupedJoint
        {
        public:
            QuadrupedJoint(const std::string &ip);
            virtual ~QuadrupedJoint();
            QuadrupedJoint(const QuadrupedJoint &rhs);
            QuadrupedJoint &operator=(const QuadrupedJoint &rhs);
            QuadrupedJoint(QuadrupedJoint &&rhs);
            QuadrupedJoint &operator=(QuadrupedJoint &&rhs);

            static const std::string_view &meta();

        public:
            /**
             * @description: 往机器人发送机器人控制数据
             * @param &cmd
             * @return {}
             */
            void SetMotorCmd(const MotorCmd &cmd);

            /**
             * @description: 从机器人获取原始电机数据，涉及效率问题，频繁使用建议用订阅
             * @param *data
             * @return {}
             */
            Result<MotorRet> GetMotorData();

            /**
             * @description: 订阅电机数据上报
             * @param func
             * @return {}
             */
            RetState SubscribetMotorData(std::function<void(const MotorRet &)> func);

            RetState SubscribetImuData(std::function<void(const ImuRet &)> func);

            /**
             * @description: 获取原始IMU数据，涉及效率问题，频繁使用建议用订阅
             * @param *data
             * @return {}
             */
            Result<ImuRet> GetImuData();
            /**
             * @description: 设置关节级控制状态
             * 必须进入关节级状态，才能进入关节级控制
             * @param set
             * @return {}
             */
            RetState SetJointState(bool set);

        private:
            std::shared_ptr<SdkProtocol> rpcCli_;
            std::unique_ptr<ApiDevice> apiDev_;
            std::unique_ptr<ApiQuadruped> apiQuad_;
        };
    }; // namespace qr
} // namespace iiri
