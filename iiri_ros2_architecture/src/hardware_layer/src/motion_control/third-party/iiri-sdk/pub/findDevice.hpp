/*
 * @Author: 唐文浩
 * @Date: 2022-06-24
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description: 设备发现/查找
 *
 * Copyright (c) 2022~2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "type/commType.hpp"

namespace iiri
{

    struct FindDeviceResult
    {
        std::string ip;
        std::string name;
        std::string model;
        FindDeviceResult(const std::string &ipIn, const std::string &nameIn, const std::string &modelIn)
            : ip(ipIn), name(nameIn), model(modelIn)
        {
        }
    };

    class FindDeviceImpl;
    class FindDevice
    {
    public:
        FindDevice(uint32_t interval_ms = 100);
        virtual ~FindDevice();
        std::vector<FindDeviceResult> GetResult();
        void Start();
        void Stop();
        void Loop();

    private:
        std::shared_ptr<FindDeviceImpl> impl_;
    };
} // namespace iiri
