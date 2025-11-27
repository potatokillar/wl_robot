/*
 * @Author: 唐文浩-0036
 * @Date: 2023-08-25
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-01-24
 * @Description: todo 每次读写都会打开数据库，耗时较长，需要优化
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "cppSqlite.hpp"

using namespace std;

// 本类是操作类，存在多个实例，但仅有一个数据库
static std::mutex dbMutex;
static SqlitePtrHandler db_; // 带计时器的共用数据库句柄指针

/**
 * @description: 默认构造函数
 * @return {}
 */
SqlitePtrHandler::SqlitePtrHandler() { Open(); };

/**
 * @description: 默认析构函数
 * @return {}
 */
SqlitePtrHandler::~SqlitePtrHandler()
{
    Close();
    // 确保计时线程结束
    if (timer_.joinable())
    {
        timer_.join();
    }
};

/**
 * @description: 获取访问数据库的指针sqlite3*。
 * 要确保互斥。
 * 如果目前有正在工作的有效指针(工作标志位valid_为true)，则说明db_里面存放着一个正在使用的指针，那么只需更新一下计时器剩余时长，然后返回db_即可。
 * 如果目前没有正在工作的指针，说明数据库较长时间未有访问，要重新开始访问数据库，运行Open()。
 * @return {}
 */
sqlite3 *SqlitePtrHandler::GetPtr()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (valid_.load())
    {
        now_.store(duration_);
    }
    else
    {
        Open();
    }
    return db_;
};

/**
 * @description: 将1个字段加入缓存中
 * @param name
 * @param value
 * @return {}
 */
void SqlitePtrHandler::PushCache(std::string name, std::string value) { cache_[name] = value; };
/**
 * @description: 从缓存中移除1个字段
 * @param name
 * @return {}
 */
void SqlitePtrHandler::DropCache(std::string name) { cache_.erase(name); };
/**
 * @description: 从缓存中读取给定字段。如果未找到则在std::pair中返回false
 * @param name
 * @return {}
 */
std::pair<bool, std::string> SqlitePtrHandler::ReadCache(std::string name)
{
    std::pair<bool, std::string> ret;
    ret.first = false;
    ret.second = {};

    if (cache_.find(name) != cache_.end())
    {
        ret.first = true;
        ret.second = cache_.at(name);
    }
    return ret;
};

/**
 * @description: 获取指针资源。
 * 先将计时器的剩余时长重置为最大值。
 * 如果指针已失效，则重新开启计时器线程timer_，将工作标志位valid_置为true
 * @return {}
 */
void SqlitePtrHandler::Open()
{
    now_.store(duration_);

    if (!valid_.load())
    {
        const char *database_name = dbName_.c_str();
        sqlite3_open(database_name, &db_);
        timer_ = std::move(std::thread(&SqlitePtrHandler::TimerThread, this));
        timer_.detach();
        valid_.store(true);
    }
};

/**
 * @description: 回收指针资源。
 * 将存储指针变量的数据成员清空，将工作标志位valid_置为false
 * @return {}
 */
void SqlitePtrHandler::Close()
{
    std::lock_guard<std::mutex> lock(mtx_);
    sqlite3_close(db_);
    valid_.store(false);
    db_ = nullptr;
};

/**
 * @description: 计时器线程函数。
 * 每过1 s将剩余时长now_减1，直至降为0之后，说明没有其他对数据库的访问，则调用回收资源的函数。
 * @return {}
 */
void SqlitePtrHandler::TimerThread()
{
    while (now_.load() > 0)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        //  LOG_DEBUG("waiting for 1 second, and there are {} second left.", now_.load());
        now_.fetch_sub(1);
    }
    Close();
    //  LOG_DEBUG("time out, will be destructed immediately.");
};

CppSqlite::CppSqlite()
{
    // todo baseline层不应该涉及具体table
    CreateOrOpen("device");
    //   CreateOrOpen("quadruped");
    //   CreateOrOpen("arm");
}

/**
 * @description: 打开或创建一个数据库，并创建两个表
 * @return {}
 */
void CppSqlite::CreateOrOpen(const std::string &table)
{
    lock_guard<mutex> lock(dbMutex);
    sqlite3 *db;
    if (sqlite3_open("iiri_ros.db", &db) != SQLITE_OK)
    {
        //  LOG_ERROR("can't create/open customConfig.db: {}", sqlite3_errmsg(db));
        return;
    }

    // 用~作为定界符
    string create_table_sql =
        R"~(
        CREATE TABLE IF NOT EXISTS device (
        name TEXT PRIMARY KEY,
        type TEXT,
        value TEXT);
        )~";

    create_table_sql.replace(create_table_sql.find("device"), 6, table);
    if (sqlite3_exec(db, create_table_sql.c_str(), NULL, NULL, NULL) != SQLITE_OK)
    {
        // LOG_ERROR("sqlite can't create table {}: {}", table, sqlite3_errmsg(db));
    }

    sqlite3_close(db);
}

std::pair<bool, string> CppSqlite::Read(const std::string &table, const std::string &name)
{
    lock_guard<mutex> lock(dbMutex);
    std::pair<bool, string> ret;
    ret.first = false;
    sqlite3 *db = db_.GetPtr();

    ret = db_.ReadCache(name);
    if (ret.first == true)
    {
        return ret;
    }

    string sql = R"~( 
        SELECT * FROM table WHERE name = ?; 
        )~";
    sql.replace(sql.find("table"), 5, table);

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, 0) != SQLITE_OK)
    {
        //  LOG_WARN("sqlite can't prepare sql: {}", sqlite3_errmsg(db));
        return ret;
    }

    sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), SQLITE_STATIC);

    // 实际上只能获取一条数据
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
        // const char *name = (const char *)sqlite3_column_text(stmt, 0);   // name字段
        // const char *type = (const char *)sqlite3_column_text(stmt, 1);   // type字段
        const char *value = (const char *)sqlite3_column_text(stmt, 2); // value字段，db所存储的实际数据
        // LOG_INFO("sql select, value = {}", value);
        // func(SQLITE_ROW, sqlite3_errmsg(db));  // 调用回调函数

        ret.first = true;
        ret.second = value;
        db_.PushCache(name, value);
    }

    sqlite3_finalize(stmt);
    return ret;
}

std::pair<bool, string> CppSqlite::Read(const std::string &table, const std::string &name, DatabaseCallbackType func)
{
    lock_guard<mutex> lock(dbMutex);
    std::pair<bool, string> ret;
    ret.first = false;
    sqlite3 *db = db_.GetPtr();

    ret = db_.ReadCache(name);
    if (ret.first == true)
    {
        return ret;
    }

    string sql = R"~( 
        SELECT * FROM table WHERE name = ?; 
        )~";
    sql.replace(sql.find("table"), 5, table);

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, 0) != SQLITE_OK)
    {
        // LOG_WARN("sqlite can't prepare sql: {}", sqlite3_errmsg(db));
        return ret;
    }

    sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), SQLITE_STATIC);

    // 实际上只能获取一条数据
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
        // const char *name = (const char *)sqlite3_column_text(stmt, 0);   // name字段
        // const char *type = (const char *)sqlite3_column_text(stmt, 1);   // type字段
        const char *value = (const char *)sqlite3_column_text(stmt, 2); // value字段，db所存储的实际数据
        // LOG_INFO("sql select, value = {}", value);
        func(SQLITE_ROW, sqlite3_errmsg(db)); // 调用回调函数

        ret.first = true;
        ret.second = value;
        db_.PushCache(name, value);
    }

    sqlite3_finalize(stmt);
    return ret;
}

/**
 * @description: 插入或更新一个文本数据，todo，若数据是空，如何处理？
 * @param &table 插入的表
 * @param &name 主键
 * @param &type 类型说明
 * @param version 版本
 * @param &value 二进制值
 * @return {}
 */
bool CppSqlite::SyncWrite(const std::string &table, const std::string &name, const std::string &type, const string &value)
{
    lock_guard<mutex> lock(dbMutex);
    sqlite3 *db = db_.GetPtr();

    string sql = R"~(
        INSERT OR REPLACE INTO table (name, type, value) VALUES (?, ?, ?);
        )~";
    sql.replace(sql.find("table"), 5, table);

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, 0) != SQLITE_OK)
    {
        // LOG_WARN("sqlite can't prepare sql: {}", sqlite3_errmsg(db));
        return false;
    }

    sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, type.c_str(), type.size(), SQLITE_STATIC);
    sqlite3_bind_text(stmt, 3, value.c_str(), value.size(), SQLITE_STATIC);

    sqlite3_step(stmt);
    db_.PushCache(name, value);

    sqlite3_finalize(stmt);
    return true;
}

bool CppSqlite::SyncWrite(const std::string &table, const std::string &name, const std::string &type, const string &value, DatabaseCallbackType func)
{
    lock_guard<mutex> lock(dbMutex);
    sqlite3 *db = db_.GetPtr();

    string sql = R"~(
        INSERT OR REPLACE INTO table (name, type, value) VALUES (?, ?, ?);
        )~";
    sql.replace(sql.find("table"), 5, table);

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, 0) != SQLITE_OK)
    {
        //  LOG_WARN("sqlite can't prepare sql: {}", sqlite3_errmsg(db));
        return false;
    }

    sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), SQLITE_STATIC);
    sqlite3_bind_text(stmt, 2, type.c_str(), type.size(), SQLITE_STATIC);
    sqlite3_bind_text(stmt, 3, value.c_str(), value.size(), SQLITE_STATIC);

    func(sqlite3_step(stmt), sqlite3_errmsg(db));
    db_.PushCache(name, value);

    sqlite3_finalize(stmt);
    return true;
}

bool CppSqlite::Delete(const std::string &table, const std::string &name)
{
    lock_guard<mutex> lock(dbMutex);
    sqlite3 *db = db_.GetPtr();

    string sql = R"~(
        DELETE FROM table WHERE name = ?;
        )~";
    sql.replace(sql.find("table"), 5, table);

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, 0) != SQLITE_OK)
    {
        // LOG_WARN("sqlite can't prepare sql: {}", sqlite3_errmsg(db));
        return false;
    }

    sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), SQLITE_STATIC);
    sqlite3_step(stmt);
    db_.DropCache(name);

    sqlite3_finalize(stmt);
    return true;
}

bool CppSqlite::Delete(const std::string &table, const std::string &name, DatabaseCallbackType func)
{
    lock_guard<mutex> lock(dbMutex);
    sqlite3 *db = db_.GetPtr();

    string sql = R"~(
        DELETE FROM table WHERE name = ?;
        )~";
    sql.replace(sql.find("table"), 5, table);

    sqlite3_stmt *stmt;
    if (sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, 0) != SQLITE_OK)
    {
        // LOG_WARN("sqlite can't prepare sql: {}", sqlite3_errmsg(db));
        return false;
    }

    sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), SQLITE_STATIC);
    func(sqlite3_step(stmt), sqlite3_errmsg(db));
    db_.DropCache(name);
    sqlite3_finalize(stmt);
    return true;
}

void CppSqlite::AsyncWrite(const std::string &table, const std::string &name, const std::string &type, const std::string &value)
{
    lock_guard<mutex> lock(dbMutex);
    sqlite3 *db = db_.GetPtr();

    string sql = R"~(
        INSERT OR REPLACE INTO table (name, type, value) VALUES (?, ?, ?);
        )~";
    sql.replace(sql.find("table"), 5, table);

    auto fut = std::async([&]
                          {
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, 0) != SQLITE_OK) {
            //    LOG_WARN("sqlite can't prepare sql: {}", sqlite3_errmsg(db));
        }

        sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), SQLITE_STATIC);
        sqlite3_bind_text(stmt, 2, type.c_str(), type.size(), SQLITE_STATIC);
        sqlite3_bind_text(stmt, 3, value.c_str(), value.size(), SQLITE_STATIC);

        const int max_times = 3;  // 最大重试次数
        int ret = 0;
        for (int i = 0; i < max_times; i++) {
            ret = sqlite3_step(stmt);
            if (ret == SQLITE_DONE)  // 如果执行成功则调用回调函数，并退出重试循环
            {
                string sql = R"~( 
                    SELECT * FROM table WHERE name = ?; 
                    )~";
                sql.replace(sql.find("table"), 5, table);
                sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, 0);
                sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), SQLITE_STATIC);
                sqlite3_step(stmt);
                const char *value = (const char *)sqlite3_column_text(stmt, 2);  // value字段，db所存储的实际数据
                                                                                 //  LOG_DEBUG("sql read, value = {}", value);
                db_.PushCache(name, value);

                break;
            }
        }

        sqlite3_finalize(stmt); });
}

void CppSqlite::AsyncWrite(const std::string &table, const std::string &name, const std::string &type, const std::string &value, DatabaseCallbackType func)
{
    lock_guard<mutex> lock(dbMutex);
    sqlite3 *db = db_.GetPtr();

    string sql = R"~(
        INSERT OR REPLACE INTO table (name, type, value) VALUES (?, ?, ?);
        )~";
    sql.replace(sql.find("table"), 5, table);

    auto fut = std::async([&]
                          {
        sqlite3_stmt *stmt;
        if (sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, 0) != SQLITE_OK) {
            //    LOG_WARN("sqlite can't prepare sql: {}", sqlite3_errmsg(db));
        }

        sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), SQLITE_STATIC);
        sqlite3_bind_text(stmt, 2, type.c_str(), type.size(), SQLITE_STATIC);
        sqlite3_bind_text(stmt, 3, value.c_str(), value.size(), SQLITE_STATIC);

        const int max_times = 3;  // 最大重试次数
        int ret = 0;
        std::string msg = {};
        for (int i = 0; i < max_times; i++) {
            ret = sqlite3_step(stmt);
            msg = sqlite3_errmsg(db);
            if (ret == SQLITE_DONE)  // 如果执行成功则调用回调函数，并退出重试循环
            {
                func(ret, msg);

                string sql = R"~( 
                    SELECT * FROM table WHERE name = ?; 
                    )~";
                sql.replace(sql.find("table"), 5, table);
                sqlite3_prepare_v2(db, sql.c_str(), sql.size(), &stmt, 0);
                sqlite3_bind_text(stmt, 1, name.c_str(), name.size(), SQLITE_STATIC);
                sqlite3_step(stmt);
                const char *value = (const char *)sqlite3_column_text(stmt, 2);  // value字段，db所存储的实际数据
                                                                                 //   LOG_DEBUG("sql read, value = {}", value);

                db_.PushCache(name, value);
                break;
            }
        }
        func(ret, msg);  // 如果失败也将结果传递到回调函数

        sqlite3_finalize(stmt); });
}
