
#pragma once

#include <cmath>
#include <iostream>
#include <map>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include <cstdio>

#include "baseline.hpp"
#include "deviceType.hpp"
#include "git_info.hpp"

struct Version
{
    std::string sdk;
    std::string protocol;
    std::string software;
    std::string tag;
    std::string git_branch_commit_md5_code;
};

namespace DevModel
{
    enum class qr
    {
        no,
        base,
        middle,
        middleV3,
        middleV4,
        linkV2_3,
        linkV2_3_w,
    };
    enum class arm
    {
        no,
        base,
        iris,
        mit,
        qa01,
        ga701,
    };
    enum class human
    {
        no,
        base,
        mit,
    };
}; // namespace DevModel

// 计算程序MD5值的函数
inline std::string calculateProgramMd5() {
    std::ifstream file("/proc/self/exe", std::ios::binary);
    if (!file) {
        return "unknown";
    }

    // 简化的MD5计算 - 实际使用系统命令
    file.close();

    // 获取程序路径
    char path[1024];
    ssize_t len = readlink("/proc/self/exe", path, sizeof(path) - 1);
    if (len == -1) {
        return "unknown";
    }
    path[len] = '\0';

    // 使用系统命令计算MD5
    std::string cmd = "md5sum ";
    cmd += path;
    cmd += " | cut -d' ' -f1 | tail -c 6"; // 取最后5位

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        return "unknown";
    }

    char buffer[32];
    std::string result = "";
    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result = buffer;
        // 移除换行符
        if (!result.empty() && result.back() == '\n') {
            result.pop_back();
        }
    }
    pclose(pipe);

    return result.empty() ? "unknown" : result;
}

// 电机配置
struct MotorCfg
{
    MotorModel model{MotorModel::haitai};
    bool chiral{false};
    double ratio{6};
    u32 circleCnt;

    MotorCfg() { circleCnt = std::pow(2, 19); }

    MotorCfg(MotorModel mod) : MotorCfg() { model = mod; }

    MotorCfg(MotorModel mod, bool chRight, double rat) : MotorCfg()
    {
        model = mod;
        chiral = chRight;
        ratio = rat;
    }

    MotorCfg(MotorModel mod, bool chRight, double rat, u32 circleBit)
    {
        model = mod;
        chiral = chRight;
        ratio = rat;
        circleCnt = std::pow(2, circleBit);
    }
};

struct CfgDev_base
{
    Version version;

    std::string audioName = "hw:Headphones";
    std::array<std::string, 2> spiName{"/dev/spidev0.0", "/dev/spidev0.1"};
    uint32_t spiSpeed = 3000000;

    std::vector<std::string> imuDev;
    std::vector<std::string> sensorDev;
    std::vector<std::string> uartVoice;

    CfgDev_base()
    {
        version.sdk = "v1.16.0";
        version.protocol = "v2.11";
        version.software = GitInfo::softwareVersion;  // 使用动态软件版本（开发版本或发布版本）

        // 生成git分支_commit_程序md5的组合
        std::string programMd5 = calculateProgramMd5();
        version.git_branch_commit_md5_code = std::string(GitInfo::branch) + "_" +
                                             std::string(GitInfo::commitShort) + "_" +
                                             programMd5;
    }

    template <typename T>
    void Load(const T &in)
    {
        *this = static_cast<CfgDev_base>(in);
    }
};
