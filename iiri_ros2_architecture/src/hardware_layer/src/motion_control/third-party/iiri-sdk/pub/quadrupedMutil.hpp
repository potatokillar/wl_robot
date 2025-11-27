/*
 * @Author: 唐文浩
 * @Date: 2024-01-31
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-18
 * @Description: 多机控制，目前只支持dance
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <string>
#include <vector>

#include "quadruped.hpp"

namespace iiri::qr
{

    class QuadrupedMutil
    {
    public:
        QuadrupedMutil(const std::vector<std::string> &ips);

        /**
         * @description: 设置跳舞，目前仅stand模式下支持
         * 注意，该函数是阻塞型函数
         * @param set 舞蹈序号，目前是0~3
         * @param ms 超时时间
         * @return {}
         */
        RetState SetDance(const std::string &name, uint32_t ms = 20000);
        RetState SetRunState(RunState sta);

    private:
        std::vector<Quadruped> contrs_;
    };

} // namespace iiri