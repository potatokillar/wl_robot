/*
 * @Author: 唐文浩-0036
 * @Date: 2023-08-24
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-01-21
 * @Description: sqlite3的相关操作
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <sqlite3.h>

#include <atomic>
#include <cstring>
#include <future>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>

#include "robot_base/cpp_types.hpp"

// 执行数据库操作时的回调函数
using DatabaseCallbackType = std::function<void(int sqliteRet, const std::string &msg)>;

/**
 * @description: sqlite数据库的指针管理类
 * 在密集操作时，不重复开启、关闭sqlite数据库，而是互斥地使用一个带有生命周期的指针。
 * 该指针如果一段时间内没有新的数据库操作，则关闭，回收相应资源。
 *
 * 数据库的内容在内存中有一个拷贝，在数据库操作时，如果缓存中存在，则直接使用缓存，否则才从数据库中读取。减少数据库操作。
 */
class SqlitePtrHandler
{
public:
    SqlitePtrHandler();
    ~SqlitePtrHandler();

    sqlite3 *GetPtr();
    void PushCache(std::string name, std::string value);
    void DropCache(std::string name);
    std::pair<bool, std::string> ReadCache(std::string name);

private:
    void Open();
    void Close();
    void TimerThread();

    sqlite3 *db_;                              // 数据库指针
    std::string dbName_ = "iiri_ros.db";       // 数据库文件(绝对)路径
    size_t duration_ = 10;                     // 计时器上限，单位为 s
    std::atomic<size_t> now_ = {duration_};    // 当前剩余时长，单位为 s
    std::thread timer_;                        // 计时器线程
    std::map<std::string, std::string> cache_; // 数据库缓存
    std::atomic<bool> valid_ = {false};        // 工作标志位
    std::mutex mtx_;                           // 互斥量
};
class CppSqlite
{
public:
    CppSqlite();

    void CreateOrOpen(const std::string &table);
    std::pair<bool, std::string> Read(const std::string &table, const std::string &name);
    std::pair<bool, std::string> Read(const std::string &table, const std::string &name, DatabaseCallbackType func);
    bool SyncWrite(const std::string &table, const std::string &name, const std::string &type, const std::string &value);
    bool SyncWrite(const std::string &table, const std::string &name, const std::string &type, const std::string &value, DatabaseCallbackType func);
    bool Delete(const std::string &table, const std::string &name);
    bool Delete(const std::string &table, const std::string &name, DatabaseCallbackType func);
    void AsyncWrite(const std::string &table, const std::string &name, const std::string &type, const std::string &value);
    void AsyncWrite(const std::string &table, const std::string &name, const std::string &type, const std::string &value, DatabaseCallbackType func);
};
