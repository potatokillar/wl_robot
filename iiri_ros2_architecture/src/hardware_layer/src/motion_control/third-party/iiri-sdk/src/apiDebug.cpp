/*
 * @Author: 唐文浩
 * @Date: 2022-08-09
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description:
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "apiDebug.hpp"

#include <vector>

using namespace nlohmann;

namespace iiri::debug
{
    ApiDebug::ApiDebug(std::shared_ptr<SdkProtocol> rpcCli) { rpcCli_ = rpcCli; }
    Result<nlohmann::json> ApiDebug::Call(const std::string &method, const nlohmann::json &params)
    {
        return rpcCli_->Call("debug", method, params);
    }

    RetState ApiDebug::SetCustomParam(const std::vector<ParamValue> &set)
    {
        nlohmann::json param;
        for (const auto &var : set)
        {
            nlohmann::json item;
            item["name"] = var.name;
            // 协议是支持double数组，字符串数组，double，字符串
            // 但这里只支持前两个，毕竟前两个在数量等于1的时候就是后两个
            if (var.num.size())
            {
                std::vector<double> str_vec = {};
                for (auto &element : var.num)
                {
                    str_vec.push_back(element);
                }
                item["value"] = str_vec;
            }
            else if (var.str.size())
            {
                item["value"] = var.str;
            }

            param.push_back(item);
        }

        // 有可能涉及到写数据库的操作，因此时间加长
        auto ret = rpcCli_->Call("debug", "SetCustomParam", param, 500);
        if (ret.first == RetState::ok)
        {
            return Json2errState(ret.second);
        }
        return ret.first;
    }
    RetState ApiDebug::GetCustomParam(const std::vector<std::string> &set, std::vector<ParamValue> &result)
    {
        nlohmann::json param;
        for (const auto &var : set)
        {
            param.push_back(var);
        }

        auto ret = Call("GetCustomParam", param);
        if (ret.first == RetState::ok)
        {
            for (const auto &item : ret.second)
            {
                try
                {
                    // 若对端收到了指令但无法获取到数据，则会返回只有name但没有value的字符串
                    // 此时不认为是异常，只认为是虽通讯成功但未获取到值
                    if (item.contains("value"))
                    {
                        if (item.at("value")[0].is_number())
                        {
                            std::vector<double> value = item.at("value");
                            ParamValue one(item.at("name"), value);
                            result.push_back(one);
                        }
                        else if (item.at("value")[0].is_string())
                        {
                            std::vector<std::string> value = item.at("value");
                            ParamValue one(item.at("name"), value);
                            result.push_back(one);
                        }
                    }
                    else
                    {
                        ParamValue one(item.at("name"));
                        result.push_back(one);
                    }
                }
                catch (const std::exception &e)
                {
                    return RetState::parseErr;
                }
            }
        }
        return ret.first;
    }
    RetState ApiDebug::GetCustomParamInfo(std::vector<ParamInfo> &result)
    {
        nlohmann::json param;

        auto ret = Call("GetCustomParamInfo", param);
        if (ret.first == RetState::ok)
        {
            for (const auto &item : ret.second)
            {
                try
                {
                    ParamInfo one(item.at("name"));

                    if (item.contains("restrict") == true)
                    {
                        for (auto &var : item.at("restrict"))
                        {
                            one.num.push_back(var);
                        }
                        one.type = ParamType::restrict;
                    }
                    else if (item.contains("range"))
                    {
                        for (auto &var : item.at("range"))
                        {
                            one.num.push_back(var);
                        }
                        one.type = ParamType::range;
                    }
                    else if (item.contains("text"))
                    {
                        for (auto &var : item.at("text"))
                        {
                            one.str.push_back(var);
                        }
                        one.type = ParamType::text;
                    }

                    result.push_back(one);
                }
                catch (const std::exception &)
                {
                    return RetState::parseErr;
                }
            }
        }
        return ret.first;
    }
} // namespace iiri::debug