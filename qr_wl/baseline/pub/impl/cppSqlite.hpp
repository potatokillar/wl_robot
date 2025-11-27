
#pragma once
#include <sqlite3.h>

#include <atomic>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>

#include "cppType.hpp"

// 执行数据库操作时的回调函数
using DatabaseCallbackType = std::function<void(int sqliteRet, const std::string &msg)>;

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

    sqlite3 *db_;
    std::string dbName_ = "customConfig.db";
    size_t duration_ = 10;
    std::atomic<size_t> now_ = {duration_};
    std::thread timer_;
    std::map<std::string, std::string> cache_;
    std::atomic<bool> valid_ = {false};
    std::mutex mtx_;
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
