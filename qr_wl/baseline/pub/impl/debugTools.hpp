

#pragma once
#include <string>

class DebugTools
{
public:
    DebugTools();
    void DumpTraceInit();
    static void PrintStack();
};

std::string GetSysError(int err = 0);

// 宏定义，若返回false，则退出
#define RETURN_IF(func)        \
    do {                       \
        if ((func) == false) { \
            return false;      \
        }                      \
    } while (0)

#define RETURN_VOID_IF(func)   \
    do {                       \
        if ((func) == false) { \
            return;            \
        }                      \
    } while (0)
