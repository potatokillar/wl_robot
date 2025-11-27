/*
 * @Author: 唐文浩
 * @Date: 2024-12-17
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-20
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#ifndef COMMAND_EXTRACT_HPP_
#define COMMAND_EXTRACT_HPP_

#include <memory>

#include "DequeTemplate.hpp"
#include "speech_recognition.hpp"

extern std::shared_ptr<DequeTemplate<std::string>> g_ptr_trans_result_queue; // 存放识别结果字符串的容器指针

class CmdExtract
{
public:
    void Work(rclcpp::Logger logger);

private:
    std::string MatchGeneralizedCommand(const std::string &string_to_check);
    void CheckEnableDeepseek(rclcpp::Logger logger, const std::string &string_to_check);
};

#endif // COMMAND_EXTRACT_HPP_
