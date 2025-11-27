/*
 * @Author: 唐文浩
 * @Date: 2024-11-12
 * @LastEditors: 唐文浩
 * @LastEditTime: 2024-11-12
 * @Description:
 *
 * Copyright (c) 2022-2024 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

namespace interface
{
    enum class RetState
    {
        ok,        // 无错误
        netErr,    // 网络错误
        outRange,  // 超范围错误
        timeout,   // 超时错误
        noSupport, // 功能不被支持错误
        parseErr,  // 协议解析错误
        noUpdate,  // 数据未更新，是旧的数据
        busy,      // 正忙，前一个任务正在执行
        interrupt, // 任务被中断(异常或正常)
        noExist,   // 请求值不存在，注意和noSupport区别
        error,     // 其他错误
    };
}