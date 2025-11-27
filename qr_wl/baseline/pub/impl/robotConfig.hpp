
#pragma once

#include <initializer_list>
#include <iostream>
#include <string>
#include <vector>

#include "singleton.hpp"
#include "toml11/toml.hpp"

template <typename T>
using configType = std::optional<T>;
using configPacket = std::optional<std::vector<toml::value>>;

// 配置文件解析
class RobotCfgFile : public Singleton<RobotCfgFile>
{
public:
    bool ParseFile(const char *filename);
    const std::vector<toml::value> &GetConfigSet() const { return configSet_; };
    bool parseOk_{false};

private:
    std::vector<toml::value> configSet_;  // 配置文件解析后的集合，序号越小优先级越高
};

inline RobotCfgFile &GetRobotCfgFile() { return RobotCfgFile::GetInstance(); }

/****************************************************************
 * 功能：用于递归打印参数，待优化到spdlog中
 * 输入：@1
 *      @2
 * 输出：@1
 *      @2
 ****************************************************************/
template <typename T>
std::ostream &print(std::ostream &os, const T *t)
{
    return os << t;
}

template <typename T, typename... Args>
std::ostream &print(std::ostream &os, const T &t, const Args &...rest)
{
    os << t << ", ";
    return print(os, rest...);
}

/**
 * @description: 直接获取对应参数
 * GetRobotConfigDirect<T>("key1", "key2", "key3");
 * @return {}
 */
template <typename T, typename... T2>
configType<T> GetRobotConfigDirect(const T2 &...keys)
{
    configType<T> value;

    for (const auto &var : GetRobotCfgFile().GetConfigSet()) {
        try {
            value = toml::find<T>(var, keys...);
            break;  // 不再往后面查找
        } catch (std::out_of_range &) {
            // 不做任何处理，因为当前配置没有，下一个配置文件可能有
        } catch (toml::type_error &) {
            // 不做任何处理，因为当前配置没有，下一个配置文件可能有
        }
    }
    if (value.has_value() == false) {
        printf("can't find config key: ");
        print(std::cout, keys...);
        std::cout << std::endl;
    }

    return value;
}

/**
 * @description: 获取一个间接的配置包，当大量参数来着同一个父类时，适用
 * @return {}
 */
template <typename... T>
configPacket GetRobotConfigPacket(const T &...keys)
{
    std::vector<toml::value> packet;

    for (const auto &var : GetRobotCfgFile().GetConfigSet()) {
        try {
            toml::value value = toml::find(var, keys...);
            packet.push_back(value);  // 递归查找所有的包
        } catch (std::out_of_range) {
            // 不做任何处理，因为当前配置没有，下一个配置文件可能有
        }
    }

    if (packet.empty()) {
        return std::nullopt;
    } else {
        return std::make_optional(packet);
    }
}

/**
 * @description: 从配置包获取参数，配置包来自于GetRobotConfigPacket
 * @return {}
 */
template <typename T, typename... T2>
configType<T> GetRobotConfigFromPacket(std::vector<toml::value> packet, const T2 &...keys)
{
    configType<T> value = std::nullopt;

    for (const auto &var : packet) {
        try {
            value = toml::find<T>(var, keys...);
            break;
        } catch (std::out_of_range) {
            // 不做任何处理，因为当前配置没有，下一个配置文件可能有
        } catch (toml::type_error) {
            // 不做任何处理，因为当前配置没有，下一个配置文件可能有
        }
    }

    return value;
}

bool GetConfigFileIsSuccess();
