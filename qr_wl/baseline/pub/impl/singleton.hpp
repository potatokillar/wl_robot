
#pragma once
#include <thread>

// 单例继承类
template <typename T>
class Singleton
{
public:
    static T& GetInstance()
    {
        static T instance;
        return instance;
    }
    virtual void Init() {};  // 子类初始化不是必须的

protected:
    Singleton() = default;
    // 单例永远不会被析构
    virtual ~Singleton()
    {
        if (thread_.joinable()) {
            thread_.join();
        }
    };
    std::thread thread_;
    virtual void Loop() {};  // 子类loop也不是必须的
};
