
#include "baseline.hpp"

ParamHotUpdateServer::ParamHotUpdateServer(const std::string &tag) : tag_(tag)
{
    rpcRxDeal_.emplace_back("bs::SetParameter" + tag_, [this](MsgType set) { return this->RespSetParameter(set); });
    rpcRxDeal_.emplace_back("bs::GetParameter" + tag_, [this](MsgType set) { return this->RespGetParameter(set); });
    rpcRxDeal_.emplace_back("bs::GetParameterInfo" + tag_, [this](MsgType set) { return this->RespGetParameterInfo(set); });
    for (auto &rpc : rpcRxDeal_) {
        rpc.Connect();  // 均连接到消息内存
    }
}

ParamHotUpdateServer::~ParamHotUpdateServer()
{
    for (auto &rpc : rpcRxDeal_) {
        rpc.DisConnect();
    }
}

bool ParamHotUpdateServer::AddParamNum(const std::string &name, SetParamType<double> funcSet, GetParamType<double> funcGet, GetParamInfoType funcInfo)
{
    paramUpdateClient.AddParam(tag_, name);
    ParamFunc func(funcSet, funcGet, funcInfo);
    funcMapNum_[name] = func;
    return true;
}
bool ParamHotUpdateServer::AddParamStr(const std::string &name, SetParamType<std::string> funcSet, GetParamType<std::string> funcGet, GetParamInfoType funcInfo)
{
    paramUpdateClient.AddParam(tag_, name);
    ParamFunc func(funcSet, funcGet, funcInfo);
    funcMapStr_[name] = func;
    return true;
}

bool ParamHotUpdateServer::RmvParam(const std::string &name)
{
    paramUpdateClient.RmvParam(name);
    {
        auto search = funcMapNum_.find(name);
        if (search != funcMapNum_.end()) {
            funcMapNum_.erase(search);
            return true;
        }
    }

    {
        auto search = funcMapStr_.find(name);
        if (search != funcMapStr_.end()) {
            funcMapStr_.erase(search);
            return true;
        }
    }
    return false;
}

/**
 * @description: 运行函数，接收来自SDK的参数操作信息
 * 必须在对应的线程中手动调用，非阻塞。
 * @return {}
 */
void ParamHotUpdateServer::TryGetRequest()
{
    for (auto &rpc : rpcRxDeal_) {
        rpc.Run();
    }
}

/**
 * @description: 接收端设置参数
 * @param req 设置的参数结构体数组
 * @return {}
 */
MsgType ParamHotUpdateServer::RespSetParameter(const MsgType &set)
{
    auto req = set.GetType<std::vector<ParamValue>>();
    // 遍历需要设置的变量
    for (const auto &var : req) {
        if (var.num.empty() == false) {
            if (funcMapNum_.count(var.name) > 0) {
                if (funcMapNum_[var.name].SetValue(var.num) == false) {
                    return RetState::error;
                }
            }
        } else if (var.str.empty() == false) {
            if (funcMapStr_.count(var.name) > 0) {
                if (funcMapStr_[var.name].SetValue(var.str) == false) {
                    return RetState::error;
                }
            }
        }
    }
    return RetState::ok;
}

/**
 * @description: 获取对应参数数据
 * @param &req 请求的参数列表
 * @return {} 请求参数列表对应的参数值
 */
MsgType ParamHotUpdateServer::RespGetParameter(const MsgType &set)
{
    auto names = set.GetType<std::vector<std::string>>();
    std::vector<ParamValue> ret;
    for (const auto &name : names) {
        if (funcMapNum_.count(name) > 0) {
            std::vector<double> param = funcMapNum_[name].GetValue();
            ret.emplace_back(name, param);
        } else if (funcMapStr_.count(name) > 0) {
            std::vector<std::string> param = funcMapStr_[name].GetValue();
            ret.emplace_back(name, param);
        }
    }
    return ret;
}
/**
 * @description: 获取对应参数信息
 * @return {} 返回所有可以设置的参数和值
 */
MsgType ParamHotUpdateServer::RespGetParameterInfo(const MsgType &set)
{
    (void)set;
    LOG_DEBUG("RespGetParameterInfo");
    std::vector<ParamInfo> ret;
    for (const auto &name : funcMapNum_) {
        ParamInfo info = name.second.GetInfo();
        info.name = name.first;
        ret.push_back(info);
    }
    for (const auto &name : funcMapStr_) {
        ParamInfo info = name.second.GetInfo();
        info.name = name.first;
        ret.push_back(info);
    }
    return ret;
}

////////////////////////////////////////////////////////////////////////////////
bool ParamUpdateClient::AddParam(const std::string &tag, const std::string &name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    // 当前或者其他tag存在该name，也可插入，防止用户忘记Rmv导致的问题
    paramSet_[tag].insert(name);
    return true;
}
bool ParamUpdateClient::RmvParam(const std::string &name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &v : paramSet_) {
        if (v.second.count(name) == 1) {
            v.second.erase(name);
            return true;
        }
    }
    return false;
}

/**
 * @description: 请求设置参数，由RPC请求端调用，阻塞
 * @param &req
 * @return {}
 */
RetState ParamUpdateClient::ReqSetParameter(const std::string &tag, const std::vector<ParamValue> &req)
{
    RetState resp = RetState::ok;
    return RpcBlockSend("bs::SetParameter" + tag, req, &resp, 100) == true ? resp : RetState::timeout;
}
/**
 * @description: 请求获取参数，由RPC请求端调用，阻塞
 * @param &names
 * @param &result
 * @return {} 返回值只有成功或者失败
 */
RetState ParamUpdateClient::ReqGetParameter(const std::string &tag, const std::vector<std::string> &names, std::vector<ParamValue> &result)
{
    return RpcTrySend("bs::GetParameter" + tag, names, &result) == true ? RetState::ok : RetState::timeout;
}
/**
 * @description: 请求获取参数信息，由RPC请求端调用，阻塞
 * @param &result
 * @return {}
 */
RetState ParamUpdateClient::ReqGetParameterInfo(const std::string &tag, std::vector<ParamInfo> &result)
{
    return RpcTrySend("bs::GetParameterInfo" + tag, &result) == true ? RetState::ok : RetState::timeout;
}

std::pair<bool, std::string> ParamUpdateClient::GetWhichTagCanDeal(const std::string &name)
{
    std::pair<bool, std::string> ret;
    ret.first = false;

    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &v : paramSet_) {
        if (v.second.count(name) == 1) {
            ret.second = v.first;
            ret.first = true;
        }
    }
    return ret;
}

std::pair<bool, std::set<std::string>> ParamUpdateClient::GetAllTag()
{
    std::pair<bool, std::set<std::string>> ret;

    std::lock_guard<std::mutex> lock(mutex_);
    if (paramSet_.empty()) {
        ret.first = false;
    }

    for (auto &v : paramSet_) {
        ret.second.insert(v.first);
        ret.first = true;
    }
    return ret;
}