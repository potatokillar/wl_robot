/*
 * @Author: 唐文浩
 * @Date: 2024-11-28
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-04-21
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "CommandExtract.hpp"

#include <future>
#include <regex>
#include <set>

#include "Global.hpp"

extern std::shared_ptr<moodycamel::ReaderWriterQueue<std::string>> g_ptr_stt_queue;
extern std::shared_ptr<DequeTemplate<std::string>> g_ptr_trans_result_queue;                                         // 存放识别结果字符串的容器指针
extern std::shared_ptr<DequeTemplate<std::string>> g_ptr_cmd_queue = std::make_shared<DequeTemplate<std::string>>(); // 存放待执行的指令的容器指针
extern std::deque<std::string> g_deepseek_msg;
extern std::mutex g_deepseek_msg_mutex;
extern std::condition_variable g_queue_cv;

static const int MAX_SEARCH_NUM = 10; // 搜索指令的最长范围的字符个数
static const int MAX_POP_NUM = 7;     // 到达最长范围后，从队首pop出字符的个数
/** @todo 20241126 - 这里的搜索范围和安全删除范围，其实是可以根据指令的具体长度生成的。
 * 假设所有指令中最长指令含有6个字符，那么搜索范围和安全删除的范围应该大于等于5
 * 保证不因为多删1个字符而导致接下来的5个字符无法被识别为本应该有的指令
 * 也尽可能地不少删字符
 */
static const int NOT_CMD_CHAR_NUM = 2;

void CmdExtract::Work(rclcpp::Logger logger)
{
    RCLCPP_INFO(logger, "CmdExtract::Work() start!");
    signal(SIGINT, signal_handler_int);
    signal(SIGQUIT, signal_handler_quit);

    std::set<std::string> command_text_set;
    command_text_set = g_node_ptr->GetCommandTextSet();

    // 将set转换为vector以便于排序
    std::vector<std::string> sorted_command_text_set(command_text_set.begin(), command_text_set.end());
    // 按照字符串长度降序排序
    std::sort(sorted_command_text_set.begin(), sorted_command_text_set.end(), [](const std::string &a, const std::string &b)
              { return a.size() > b.size(); });

    while (g_run.load())
    {
        // 从队列中取出1个字符串
        // auto item_popped = g_ptr_trans_result_queue->dPopWithTimeout(std::chrono::milliseconds(100));  // 从队列中取出1个字符串
        // std::string string_to_check;
        // if (item_popped.has_value()) {
        //     string_to_check = item_popped.value();
        // }
        std::string string_to_check;
        if (!g_ptr_stt_queue->try_dequeue(string_to_check))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // 检查是否是开启深度思考模式的指令
        CheckEnableDeepseek(logger, string_to_check);
        // 如果深度思考模式未启动，则检查里面是否有语音指令
        // 如果深度思考模式已启动，则跳过整个环节
        if (!g_enable_deepseek.load())
        {
            bool flag_matched = false;
            // 执行严格匹配
            for (const auto &item : sorted_command_text_set)
            {
                if (string_to_check.find(item) != std::string::npos)
                {
                    // std::thread thread_wav_play = std::thread([=]() { WavPlay(std::string("attention")); });
                    RCLCPP_INFO(logger, "found command = %s", item.c_str());
                    g_ptr_cmd_queue->Push(item); // 将匹配到的指令，放到队列中
                    flag_matched = true;
                    // thread_wav_play.detach();
                    break;
                }
            }
            // 执行泛化匹配
            if (!flag_matched)
            {
                std::string generalized_command = MatchGeneralizedCommand(string_to_check);
                if (!generalized_command.empty())
                {
                    // std::thread thread_wav_play = std::thread([=]() { WavPlay(std::string("attention")); });
                    RCLCPP_INFO(logger, "found generalized command = %s", generalized_command.c_str());
                    g_ptr_cmd_queue->Push(generalized_command); // 将匹配到的指令，放到队列中
                    // thread_wav_play.detach();
                }
            }
        }
    }
    RCLCPP_INFO(logger, "CmdExtract::Work() finished!");
}

std::string CmdExtract::MatchGeneralizedCommand(const std::string &string_to_check)
{
    std::wstring wide_string_to_check = String2WideString(string_to_check);
    std::string result = "";

    // 定义正则表达式模式 "小X小X...."，其中 X 是任意单个字符，后面跟着最多4个任意字符
    std::wregex pattern(L"小(.{1})小\\1.{0,4}");
    // 用于存储匹配结果
    std::wsregex_iterator it(wide_string_to_check.begin(), wide_string_to_check.end(), pattern);
    std::wsregex_iterator end;

    if (it != end)
    {
        // 如果找到匹配项，则提取匹配部分
        std::wstring matched_part = (*it)[0];
        // 确保至少有5个字符才能正确截取AAAA部分
        if (matched_part.length() >= 5)
        {
            // 提取AAAA部分，即从第5个字符开始的4个字符
            std::wstring AAAA_part = matched_part.substr(4, 4);
            std::string AAAA_part_short = WideString2String(AAAA_part);

            // 检查命令并设置结果
            if (AAAA_part_short.find("前") != std::string::npos)
            {
                result = "小黑小黑前进";
            }
            else if (AAAA_part_short.find("后") != std::string::npos)
            {
                result = "小黑小黑后退";
            }
            else if (AAAA_part_short.find("立") != std::string::npos)
            {
                result = "小黑小黑起立";
            }
            else if (AAAA_part_short.find("下") != std::string::npos)
            {
                result = "小黑小黑趴下";
            }
            else
            {
                std::regex move_left_pattern("左.*(?:移|走)");
                std::regex move_right_pattern("右.*(?:移|走)");

                if (std::regex_search(AAAA_part_short, move_left_pattern))
                {
                    result = "小黑小黑向左平移";
                }
                else if (std::regex_search(AAAA_part_short, move_right_pattern))
                {
                    result = "小黑小黑向右平移";
                }
                else if (AAAA_part_short.find("左") != std::string::npos)
                {
                    result = "小黑小黑左转";
                }
                else if (AAAA_part_short.find("右") != std::string::npos)
                {
                    result = "小黑小黑右转";
                }
            }
        }
    }

    return result;
}

void CmdExtract::CheckEnableDeepseek(rclcpp::Logger logger, const std::string &string_to_check)
{
    // 检查是否要进入Deepseek模式
    // std::regex enable_pattern(R"(^(?:小黑){1,}(?:打开|进入|开启|启动)深度思考.{0,2}$)");
    std::regex enable_pattern(R"(^(?:打开|进入|开启|启动|开始)深度思考.{0,2}$)"); // 去掉了前面的"小黑小黑"部分
    // 检查是否要退出Deepseek模式
    std::regex disable_pattern(R"(^.*(?:关闭|退出|停止|结束)深度思考.{0,2}$)");

    if (std::regex_match(string_to_check, enable_pattern))
    {
        g_enable_deepseek.store(true);
        // auto future = std::async(std::launch::async, [=]() { WavPlay(std::string("attention")); });
        {
            std::lock_guard<std::mutex> guard(g_deepseek_msg_mutex); // 自动加锁并在作用域结束时解锁
            g_deepseek_msg.clear();                                  // 清空队列
        }
        RCLCPP_INFO(logger, "enter Deepseek mode.");
    }
    else if (std::regex_match(string_to_check, disable_pattern))
    {
        g_enable_deepseek.store(false);
        // auto future = std::async(std::launch::async, [=]() { WavPlay(std::string("open_asr")); });
        RCLCPP_INFO(logger, "exit Deepseek mode.");
    }
    else
    {
        // 如果不匹配上述任何一种情况，则不做处理
    }
}