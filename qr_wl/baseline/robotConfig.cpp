
#include <climits>
#include <iostream>

#include "baseline.hpp"

using namespace std;

bool RobotCfgFile::ParseFile(const char *filename)
{
    bool isOk = false;
    // 此时日志设置其实还没初始化，所以手动设置下格式
    // spdlog::set_pattern("[%H:%M:%S-%e][%^%l%$] %v");
    char actualPath[PATH_MAX + 1] = {0};
    char *ptrRet = realpath(filename, actualPath);  // 需判断路径是否正确
    if (ptrRet == NULL) {
        // LOG_ERROR("config file realpath failure: {}", filename);
        return false;
    }

    std::string absFullname = actualPath;
    string absPath = absFullname.substr(0, absFullname.rfind('/') + 1);  // 需要'/'字符

    // LOG_INFO("fullname {}, absPath {}", absFullname, absPath);

    // 递归解析文件
    string nowCfgFile = absFullname;  // 当前解析的文件
    toml::value cfgData;
    configSet_.clear();  // 需清除旧结果
    bool parseComplete = false;
    while (parseComplete == false) {
        try {
            // LOG_INFO("prepare parse config file {}, waiting...", nowCfgFile);
            cfgData = toml::parse(nowCfgFile);
            configSet_.push_back(cfgData);
        } catch (std::runtime_error &e) {
            // LOG_ERROR("robot config file maybe no exist or can't read");
            return false;
        } catch (toml::syntax_error &) {
            // LOG_ERROR("robot config file has syntax error");
            return false;
        }

        try {
            string parentCfgFile = toml::find<string>(cfgData, "include");
            nowCfgFile = absPath + parentCfgFile;
        } catch (std::out_of_range &) {
            break;
        } catch (toml::type_error &) {
            break;
        }
    }

    // 设备型号必须存在，其他型号可以不存在，或者为空
    auto robotModel = GetRobotConfigDirect<string>("devModel");
    if (robotModel) {
        // LOG_INFO("robot config file parse success, model:{}, robot version: {}", robotModel.second, robotVer.second);
        parseOk_ = true;  // 获取到版本号和模型才认为配置文件可用
        isOk = true;
    }

    return isOk;
}

bool GetConfigFileIsSuccess() { return GetRobotCfgFile().parseOk_; }
