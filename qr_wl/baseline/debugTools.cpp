
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

#include <cstdio>

#include "baseline.hpp"

/****************************************************************
 * 功能：堆栈打印
 * 输入：@1
 * 输出：@1
 *      @2
 ****************************************************************/
void DebugTools::PrintStack()
{
    const int MAX = 50;  // 50层堆栈
    void *buffer[MAX];
    size_t size;
    size = backtrace(buffer, MAX);
    LOG_CRITICAL(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

    // backtrace_symbols_fd(array, size, STDERR_FILENO);

    char **errStr = backtrace_symbols(buffer, size);
    if (errStr) {
        for (size_t i = 0; i < size; i++) {
            LOG_CRITICAL("{}", errStr[i]);
        }
        LOG_CRITICAL("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
        free(errStr);
    }
}

/****************************************************************
 * 功能：段错误时打印堆栈，backtrace等不可重入
 * 输入：@1
 * 输出：@1
 *      @2
 ****************************************************************/
static void DumpHandler(int sig)
{
    if (sig == SIGSEGV) {
        LOG_CRITICAL("recv SEGSEGV, segmentfault");

        const int MAX = 50;  // 50层堆栈
        void *buffer[MAX];
        size_t size;
        size = backtrace(buffer, MAX);
        LOG_CRITICAL(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");

        // backtrace_symbols_fd(array, size, STDERR_FILENO);

        char **errStr = backtrace_symbols(buffer, size);
        if (errStr) {
            for (size_t i = 0; i < size; i++) {
                LOG_CRITICAL("{}", errStr[i]);
            }
            LOG_CRITICAL("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            free(errStr);
        }

        struct sigaction sigUser = {};
        sigUser.sa_handler = SIG_DFL;
        sigUser.sa_flags = 0;
        sigaction(SIGSEGV, &sigUser, NULL);
        raise(SIGSEGV);
    }
}

void DebugTools::DumpTraceInit()
{
    struct sigaction sigUser = {};
    sigUser.sa_handler = DumpHandler;
    sigUser.sa_flags = 0;
    sigaction(SIGSEGV, &sigUser, NULL);
}

DebugTools::DebugTools() { DumpTraceInit(); }

std::string GetSysError(int err)
{
    static constexpr int BUF_SIZE = 100;
    char buf[BUF_SIZE];
    char *ret;
    if (err == 0) {
        ret = strerror_r(errno, buf, BUF_SIZE);  // 线程安全
    } else {
        ret = strerror_r(err, buf, BUF_SIZE);
    }
    (void)ret;
    return buf;
}