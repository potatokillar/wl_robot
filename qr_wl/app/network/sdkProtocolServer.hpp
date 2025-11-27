
#pragma once
#include <any>
#include <iomanip>

#include "miniServer.hpp"
#include "nlohmann/json.hpp"

using RpcServerCallbackType = std::function<nlohmann::json(nlohmann::json)>;
using BinCallbackType = std::function<std::vector<uint8_t>(const NetClientInfo&, const std::vector<uint8_t>&)>;

nlohmann::json ErrState2json(RetState sta);

struct SdkSubscribeFuncType
{
    u32 ms;
    std::set<NetClientInfo> clients;
    std::function<std::pair<bool, nlohmann::json>()> func;
    std::string nameSpace;
    std::string method;
    bool addNew{false};
    void Send();
    u64 lastPub_;
};

struct SubscribeInfo
{
    uint32_t lost;
    std::set<std::string> keys;
};

class SdkProtocolServer
{
public:
    SdkProtocolServer();
    void JsonRecv(const NetClientInfo& info, const ProtocolParse& rxData);
    void JsonSend(const NetClientInfo& info, const nlohmann::json& method);

    void BinRecv(const NetClientInfo& info, const ProtocolParse& rxData);
    bool BinSend(const NetClientInfo& info, const std::vector<BinPackConstruct>& data);
    bool BinSend(const NetClientInfo& info, const BinPackConstruct& data);

    bool RawSend(const NetClientInfo& info, uint8_t opt, uint16_t extPot, const std::vector<uint8_t>& data);

    void AddMethodCall(const std::string& nameSpace, const std::string& method, RpcServerCallbackType func);
    void AddBinCall(const std::string& method, BinCallbackType func);
    void AddSubscribeCall(const std::string& nameSpace, const std::string& name, int ms, std::function<std::pair<bool, nlohmann::json>()> func);
    void AddTimeoutCall(const std::string& name, int ms, EvtTimeoutCallbackType func);

    NetClientInfo GetClientInfo();

private:
    void ParseRecv(const NetClientInfo& info, const ProtocolParse& rxData);
    void RmvSubscribeClient(const NetClientInfo& client);
    nlohmann::json Subscribe(const nlohmann::json& params);
    nlohmann::json Unsubscribe(const nlohmann::json& params);
    nlohmann::json RxAreYouOk(const nlohmann::json& params);
    void CheckClientOnlineStatus();

private:
    std::map<std::string, std::map<std::string, RpcServerCallbackType>> methodFunc_;
    std::map<std::string, BinCallbackType> binFunc_;

    NetClientInfo curClient_;
    std::vector<NetClientInfo> clients_;
    std::vector<std::string> traceFilter_;
    std::map<std::string, std::map<std::string, SdkSubscribeFuncType>> subscribeInfo_;
};

inline std::string Double2Json(double number, size_t width = 6)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(width) << number;
    return ss.str();
};

inline double Json2Double(nlohmann::json data) { return stod(std::string(data)); };
