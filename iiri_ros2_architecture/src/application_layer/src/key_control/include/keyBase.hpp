/*
 * @Author: 唐文浩
 * @Date: 2023-12-14
 * @LastEditors: 唐文浩
 * @LastEditTime: 2024-11-25
 * @Description: 按键基类
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <atomic>
#include <future>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

#include "gamepad.hpp"

const uint64_t long_threshold{30}; // 长按时间阈值

/**
 * @description: 增强按键功能的类。用于支撑长按功能的计时器counter，标志位has_long_pressed。用于支撑多键功能的标志位multi_key_enbale。
 * @return {}
 */
class KeyPlus
{
public:
    KeyPlus() : counter(0), has_long_pressed(false), multi_key_enbale(false) {};
    ~KeyPlus() = default;

    void ClearAll()
    {
        counter = 0;
        SetLongPressed(false);
    };
    void Count() { counter++; };
    bool IsLongPress()
    {
        if (counter > long_threshold && !GetMultiEnable())
        {
            return true;
        }
        else
        {
            return false;
        }
    };
    void AfterLongPress()
    {
        counter = 0;
        SetLongPressed(true);
    };
    bool GetMultiEnable() { return multi_key_enbale; };
    void SetMultiEnable(bool value) { multi_key_enbale = value; };
    bool HasBeenLongPressed() { return has_long_pressed; };
    void SetLongPressed(bool value) { has_long_pressed = value; };

private:
    uint64_t counter; // 计时器
    bool has_long_pressed;
    bool multi_key_enbale; // 表示这个键是否正处于触发多键功能的状态
};

class KeyRecord
{
public:
    void SetValue(bool press);
    bool IsPress();          // 按下为true，弹起为false
    bool IsTriggerPress();   // 按键按下触发一次，再次读取则为false
    bool IsTriggerRelease(); // 按键释放触发一次，再次读取则为false
private:
    bool isPress_{false};
    bool isTriggerPress_{false};
    bool isTriggerRelease_{false};
};

/**
 * @description: 为std::pair<std::string, std::string>手动提供的哈希函数
 * @return {}
 */
struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &pair) const
    {
        auto hash1 = std::hash<T1>{}(pair.first);
        auto hash2 = std::hash<T2>{}(pair.second);

        if (hash1 != hash2)
        {
            return hash1 ^ hash2; // 或者可以使用其他组合方式
        }

        // 如果两个哈希值相同，则增加一个额外的扰动
        return hash1 + 0x9e3779b9; // 使用一个常数进行扰动
    }
};

/**
 * @description: 表示按键功能分类的枚举量，目前仅有短按与长按两种
 * 多键功能，与短按、长按并未存储在同一个容器中，所以不使用这一枚举量
 * @return {}
 */
enum class KeyMode
{
    Short,
    Long
};

class KeyBase
{
public:
    KeyBase();
    KeyBase(const std::string &dev);
    void Run() {};
    void SaveGamepadBtn(const GamepadData &key);

    // 适配原有按键编程方式的Is接口，仅提供条件判断
    bool IsShortRelease(std::string key);
    bool IsMultiKey(std::string key1, std::string key2);
    bool IsLongPress(std::string key);

    // 注册按键动作的接口
    void AddMultiFunc(std::string key1, std::string key2, std::function<void()> func);
    void AddFunc(std::string key, KeyMode mode, std::function<void()> func);

    std::map<std::string, double> GetStick();

private:
    // 回调式接口，按键动作在构造函数中注册即可，无需关注具体触发细节与顺序
    void RunRelease(std::string key);
    void MultiCallback(std::string key1, std::string key2);
    void LongPressingCallback(std::string key);
    void ShortPressCallback(std::string key);
    void MultiEnable();

    std::map<std::string, KeyRecord> btn_; // bool类型的按键

    std::map<std::string, KeyPlus> key_plus_; // 按键扩展类，每个按键新增一些属性和接口
    std::set<std::string> multi_key_set;      // 支持多键的按键的集合
    // 存储短按、长按动作的容器
    std::map<std::string, std::map<KeyMode, std::function<void()>>> func_;
    // 存储多键动作的容器，显式提供哈希函数pair_hash()
    std::unordered_map<std::pair<std::string, std::string>, std::function<void()>, pair_hash> multi_func_;
    std::pair<std::string, std::string> multi_act; // 存放构成多键的结果

    std::mutex mutex_;
    std::map<std::string, double> stick_; // 扳机类型的按键
    bool isKeyExist_{false};              // 存在手柄按键

    void LongPressingCount(std::string key1);
    std::pair<std::string, std::string> MakePairMultiFunc(std::string key1, std::string key2);
    void SecondReleaseInMulti(std::string key1, std::string key2, bool &value);
    void PutInJustPressed(std::string key1, std::string key2);
    bool IsInJustPressed(std::string key1, std::string key2);
};