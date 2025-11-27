/*
 * @Author: 唐文浩
 * @Date: 2024-01-31
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-06-18
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */

#include "quadrupedMutil.hpp"

#include <future>

using namespace std;
namespace iiri::qr
{

    QuadrupedMutil::QuadrupedMutil(const std::vector<std::string> &ips)
    {
        for (const auto &ip : ips)
        {
            contrs_.emplace_back(ip);
        }
    }

    RetState QuadrupedMutil::SetDance(const std::string &name, uint32_t ms)
    {
        vector<future<RetState>> fs;
        for (auto &ctr : contrs_)
        {
            fs.emplace_back(async([&ctr, name, ms]()
                                  { return ctr.SetDance(name, ms); }));
        }

        RetState ret = RetState::ok;
        for (auto &f : fs)
        {
            RetState ret2 = f.get();
            if (ret2 != RetState::ok)
            {
                ret = ret2;
            }
        }
        return ret;
    }

    RetState QuadrupedMutil::SetRunState(RunState sta)
    {
        RetState ret = RetState::ok;
        for (auto &ctr : contrs_)
        {
            auto oneRet = ctr.SetRunState(sta);
            if (oneRet != RetState::ok)
            {
                ret = oneRet;
            }
        }
        return ret;
    }
} // namespace iiri::qr