
#pragma once
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "msgCenter.hpp"
#include "robotType.hpp"

enum class ParamType
{
    value,     // 参数是值
    range,     // 参数是范围
    restrict,  // 参数是可选值
    text,      // 参数是文本
};

// 参数信息
struct ParamInfo
{
    std::string name;
    ParamType type;  // param的类型，仅info有用
    std::vector<double> num;
    std::vector<std::string> str;
    ParamInfo() {}
    ParamInfo(std::string _name, std::vector<double> _param) : name(_name), num(_param) { type = ParamType::value; }
    ParamInfo(std::string _name, std::vector<std::string> _param) : name(_name), str(_param) { type = ParamType::text; }
};

// 参数值
struct ParamValue
{
    std::string name;
    // num和str只有一个会有效，num优先
    std::vector<double> num;
    std::vector<std::string> str;
    ParamValue(std::string _name, std::vector<double> _param) : name(_name), num(_param) {}
    ParamValue(std::string _name, std::vector<std::string> _param) : name(_name), str(_param) {}
};

template <typename T>
using SetParamType = std::function<bool(std::vector<T>)>;

template <typename T>
using GetParamType = std::function<std::vector<T>()>;
using GetParamInfoType = std::function<ParamInfo()>;

template <typename T>
struct ParamFunc
{
    SetParamType<T> SetValue;
    GetParamType<T> GetValue;
    GetParamInfoType GetInfo;
    ParamFunc() {};
    ParamFunc(SetParamType<T> set, GetParamType<T> get, GetParamInfoType info)
    {
        SetValue = set;
        GetValue = get;
        GetInfo = info;
    }
};

// 参数更新客户端，由SDK调用，只有一个
class ParamUpdateClient
{
public:
    bool AddParam(const std::string &tag, const std::string &name);
    bool RmvParam(const std::string &name);

    RetState ReqSetParameter(const std::string &tag, const std::vector<ParamValue> &req);
    RetState ReqGetParameter(const std::string &tag, const std::vector<std::string> &names, std::vector<ParamValue> &result);
    RetState ReqGetParameterInfo(const std::string &tag, std::vector<ParamInfo> &result);

    std::pair<bool, std::string> GetWhichTagCanDeal(const std::string &name);
    std::pair<bool, std::set<std::string>> GetAllTag();

private:
    std::map<std::string, std::set<std::string>> paramSet_;  // 存放所有支持的参数以及其所属的tag
    std::mutex mutex_;
};
inline ParamUpdateClient paramUpdateClient;

class ParamHotUpdateServer
{
public:
    ParamHotUpdateServer(const std::string &tag);
    ~ParamHotUpdateServer();

    bool AddParamNum(const std::string &name, SetParamType<double> funcSet, GetParamType<double> funcGet, GetParamInfoType funcInfo);
    bool AddParamStr(const std::string &name, SetParamType<std::string> funcSet, GetParamType<std::string> funcGet, GetParamInfoType funcInfo);
    bool RmvParam(const std::string &name);

    void TryGetRequest();

private:
    MsgType RespSetParameter(const MsgType &set);
    MsgType RespGetParameter(const MsgType &set);
    MsgType RespGetParameterInfo(const MsgType &set);
    std::vector<RpcRecv> rpcRxDeal_;  // rpc接收处理
private:
    std::map<std::string, ParamFunc<double>> funcMapNum_;
    std::map<std::string, ParamFunc<std::string>> funcMapStr_;
    std::string tag_;
};
