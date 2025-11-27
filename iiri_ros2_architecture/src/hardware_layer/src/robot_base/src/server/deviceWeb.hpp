/*
 * @Author: 唐文浩
 * @Date: 2025-01-16
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-01-20
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <memory>
#include <thread>

class DeviceWeb
{
public:
    DeviceWeb();

private:
    std::thread thread_;
    void Run();
};