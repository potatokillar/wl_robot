/*
 * @Author: 唐文浩
 * @Date: 2025-02-28
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-28
 * @Description:
 *
 * Copyright (c) 2022-2025 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <string>
#include <vector>

#include "armType.hpp"
#include "commType.hpp"
#include "humanType.hpp"
#include "qrType.hpp"

namespace iiri::debug
{
    struct LegData
    {
        iiri::qr::MotorCmd cmd;
        iiri::qr::MotorRet ret;
    };
    struct RpyData
    {
        iiri::ImuRet cmd;
        iiri::ImuRet ret;
    };

    // 调试关心的数据
    struct WatchData
    {
        double a[3]{0}; // 每一个数据一个页面，三条曲线
        double b[3]{0};
        double c[3]{0};

        double ra[3]{0}; // 上面a的反馈曲线，和a一张图
        double rb[3]{0};
        double rc[3]{0};
    };

    // 机械臂数据
    struct ArmData
    {
        iiri::arm::MotorCmd cmd;
        iiri::arm::MotorRet ret;
    };

    // 机械臂示教器
    struct ArmTeachs
    {
        iiri::arm::ArmTeach cmd;
        iiri::arm::ArmTeach ret;
    };

    // 人型数据
    struct HumanData
    {
        iiri::human::MotorCmd cmd;
        iiri::human::MotorRet ret;
    };

    // 参数数据
    enum class ParamType
    {
        value,    // 参数是值
        range,    // 参数是范围
        restrict, // 参数是可选值
        text,     // 参数是文本
    };

    // 参数信息
    // todo 此接口并不完全属于debug，参数配置也是用到这个接口的，将来再议
    struct ParamInfo
    {
        std::string name;
        ParamType type; // param的类型，仅info有用
        std::vector<double> num;
        std::vector<std::string> str;
        ParamInfo() {}
        ParamInfo(std::string _name) : name(_name) {}
        ParamInfo(std::string _name, std::vector<double> _param) : name(_name), num(_param) { type = ParamType::value; }
        ParamInfo(std::string _name, std::vector<std::string> _param) : name(_name), str(_param) { type = ParamType::text; }
    };

    // 参数值
    struct ParamValue
    {
        std::string name;
        // num和str只有一个会有效，num优先
        std::vector<double> num;
        std::vector<std::string> str;
        ParamValue(std::string _name) : name(_name) {}
        ParamValue(std::string _name, std::vector<double> _param) : name(_name), num(_param) {}
        ParamValue(std::string _name, std::vector<std::string> _param) : name(_name), str(_param) {}
    };

}; // namespace iiri::debug