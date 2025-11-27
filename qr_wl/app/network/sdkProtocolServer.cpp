
#include "sdkProtocolServer.hpp"

#include "baseline.hpp"
#include "netMsg.hpp"

using namespace std;
using namespace ::nlohmann;

static const map<RetState, string> RETSTATE_STRING = {
    {RetState::ok, "ok"},
    {RetState::netErr, "netErr"},
    {RetState::outRange, "outRange"},
    {RetState::timeout, "timeout"},
    {RetState::noSupport, "noSupport"},
    {RetState::parseErr, "parseErr"},
    {RetState::noUpdate, "noUpdate"},
    {RetState::busy, "busy"},
    {RetState::interrupt, "interrupt"},
    {RetState::noExist, "noExist"},
    {RetState::error, "error"},
};
nlohmann::json ErrState2json(RetState sta)
{
    if (RETSTATE_STRING.count(sta))
    {
        return RETSTATE_STRING.at(sta);
    }

    LOG_ERROR_SDK("RetState2json error, please add: {}", Enum2Num(sta));
    return "error";
}

void SdkSubscribeFuncType::Send()
{
    auto bodyData = func();

    if ((bodyData.first == true) || (addNew))
    {
        json jSend;
        jSend[nameSpace]["method"] = method;
        jSend[nameSpace]["result"] = bodyData.second;

        string tx = nlohmann::to_string(jSend) + '\0';

        ProtocolConstruct txData((uint8_t *)tx.c_str(), tx.size());
        txData.SetOption(0, 0);
        for (const auto &client : clients)
        {
            GetMiniServer().Send(client, txData);
        }

        if ((clients.empty() == false) && (TimerTools::GetNowTickMs() - lastPub_ > 1000))
        {
            LOG_TRACE_SDK("subscribe pub: {}", tx);
            lastPub_ = TimerTools::GetNowTickMs();
        }
        addNew = false;
    }
}

SdkProtocolServer::SdkProtocolServer()
{
    GetMiniServer().AddUdpServer("udp-rpc", 20333);
    GetMiniServer().AddMcastServer("mcast", "224.0.0.88", 20444);

    GetMiniServer().SetSdkParse([this](const NetClientInfo &info, const ProtocolParse &data)
                                { this->ParseRecv(info, data); });

    traceFilter_.push_back("AreYouOk");
    traceFilter_.push_back("FindDevice");

    // 自行处理订阅有关的信息
    AddMethodCall("base", "Subscribe", [this](json params) -> json
                  { return this->Subscribe(params); });
    AddMethodCall("base", "Unsubscribe", [this](json params) -> json
                  { return this->Unsubscribe(params); });

    AddMethodCall("device", "AreYouOk", [this](json params) -> json
                  { return this->RxAreYouOk(params); });
    AddTimeoutCall("CheckClientState", 100, [this]()
                   { this->CheckClientOnlineStatus(); });

    AddMethodCall("base", "AreYouOk", [this](json params) -> json
                  { return this->RxAreYouOk(params); });
};

void SdkProtocolServer::ParseRecv(const NetClientInfo &info, const ProtocolParse &rxData)
{
    // SDK层接收到UDP数据包
    // LOG_INFO_SDK("[SDK] Received packet from %s:%d, option=%d, size=%zu",
    //
    curClient_ = info;
    if (rxData.GetOption() == 0)
    {
        JsonRecv(info, rxData);
    }
    if ((rxData.GetOption() == 1) && (rxData.GetExtOption() == 2))
    {
        BinRecv(info, rxData);
    }
    curClient_ = {}; // 仅在回调函数中有效
}

void SdkProtocolServer::JsonRecv(const NetClientInfo &info, const ProtocolParse &rxData)
{
    try
    {
        json jRecv = json::parse(rxData.GetData());

        for (auto &methodFunc : methodFunc_)
        {
            const auto &nameSpace = methodFunc.first;
            auto &funcMap = methodFunc.second;

            if (jRecv.find(nameSpace) == jRecv.end())
            {
                continue;
            }

            const string &method = jRecv.at(nameSpace).at("method");
            json params; // 可以不存在params
            if (jRecv.at(nameSpace).contains("params") == true)
            {
                params = jRecv.at(nameSpace).at("params");
            }

            // SDK RPC调用日志
            // LOG_INFO_SDK("[SDK RPC] namespace='%s', method='%s'", nameSpace.c_str(), method.c_str());

            bool noPrint = true;
            for (const auto &filter : traceFilter_)
            {
                if (filter == method)
                {
                    noPrint = false;
                }
            }
            if (noPrint)
            {
                LOG_TRACE_SDK("rpc req {}", nlohmann::to_string(jRecv) + '\0');
            }

            if (funcMap.find(method) != funcMap.end())
            {
                json result = funcMap[method](params);

                if (result.empty())
                {
                    continue;
                }
                // 构造一个json类，转成字符串并发出去
                json jSend;
                jSend[nameSpace]["method"] = method;
                jSend[nameSpace]["result"] = result;
                if (noPrint)
                {
                    LOG_TRACE_SDK("rpc resp {}", nlohmann::to_string(jSend) + '\0');
                }
                string tx = nlohmann::to_string(jSend) + '\0';
                ProtocolConstruct txData((uint8_t *)tx.c_str(), tx.size());
                txData.SetOption(0, 0);                   // 字符串为自定义字符串
                txData.SetSequence(rxData.GetSequence()); // 复制接收的sequence

                GetMiniServer().Send(info, txData);
            }
        }
    }
    catch (std::exception &)
    {
        LOG_WARN_SDK("rpc call exception");
    }
}

void SdkProtocolServer::JsonSend(const NetClientInfo &info, const nlohmann::json &jSend)
{
    ProtocolConstruct txMsg(nlohmann::to_string(jSend) + '\0');
    GetMiniServer().Send(info, txMsg);
}

void SdkProtocolServer::AddMethodCall(const std::string &nameSpace, const std::string &method, RpcServerCallbackType func) { methodFunc_[nameSpace][method] = func; }

void SdkProtocolServer::AddBinCall(const std::string &method, BinCallbackType func) { binFunc_[method] = func; }

void SdkProtocolServer::BinRecv(const NetClientInfo &info, const ProtocolParse &rxData)
{
    BinPackParse binDatas(rxData.GetData());
    for (u32 i = 0; i < binDatas.GetSize(); i++)
    {
        string method = binDatas.GetTag(i);
        if (binFunc_.count(method))
        {
            auto ret = binFunc_[method](info, binDatas.GetData(i));

            if (ret.empty() == false)
            {
                BinPackConstruct binPack(method, ret);
                ProtocolConstruct txData(reinterpret_cast<const uint8_t *>(&binPack.GetData()), binPack.GetSize());
                GetMiniServer().Send(info, txData);
            }
        }
    }
}

bool SdkProtocolServer::BinSend(const NetClientInfo &info, const std::vector<BinPackConstruct> &data)
{
    ProtocolConstruct txData(reinterpret_cast<const uint8_t *>(data.data()), data.size() * sizeof(BinPackConstruct));
    txData.SetOption(1, 2);
    return GetMiniServer().Send(info, txData);
}

bool SdkProtocolServer::BinSend(const NetClientInfo &info, const BinPackConstruct &data)
{
    ProtocolConstruct txData(data.GetData());
    txData.SetOption(1, 2);
    return GetMiniServer().Send(info, txData);
}

NetClientInfo SdkProtocolServer::GetClientInfo() { return curClient_; }

bool SdkProtocolServer::RawSend(const NetClientInfo &info, uint8_t opt, uint16_t extPot, const std::vector<uint8_t> &data)
{
    ProtocolConstruct txData(data);
    txData.SetOption(opt, extPot);
    return GetMiniServer().Send(info, txData);
}

void SdkProtocolServer::AddSubscribeCall(const std::string &nameSpace, const std::string &method, int ms, std::function<std::pair<bool, nlohmann::json>()> func)
{
    if (ms <= 0)
    {
        LOG_WARN_SDK("subscribe interval can't <= 0!");
        return;
    }
    SdkSubscribeFuncType funcStruct;
    funcStruct.func = func;
    funcStruct.ms = ms;
    funcStruct.nameSpace = nameSpace;
    funcStruct.method = method;

    subscribeInfo_[nameSpace][method] = funcStruct;
}

nlohmann::json SdkProtocolServer::Subscribe(const nlohmann::json &params)
{
    try
    {
        for (const auto &item : params)
        {
            string nameSpace = item.at("namespace");
            string method = item.at("method");
            int ms = 0;
            if (item.contains("interval"))
            {
                ms = item.at("interval");
            }

            auto &funcStruct = subscribeInfo_.at(nameSpace).at(method);

            if (ms == 0)
            {
                ms = funcStruct.ms;
            }

            if (funcStruct.clients.empty())
            {
                std::function<void(void)> sendFunc = [this, nameSpace, method]()
                { this->subscribeInfo_.at(nameSpace).at(method).Send(); };
                GetMiniServer().AddTimeoutCallback(nameSpace + method, ms, sendFunc);
            }

            funcStruct.clients.insert(GetClientInfo());

            LOG_DEBUG_SDK("subscribe, client:{}:{}, namespace:{}, method:{}, interval:{}", GetClientInfo().clientIp, GetClientInfo().clientPort, nameSpace, method, ms);
        }
        return ErrState2json(RetState::ok);
    }
    catch (const std::exception &)
    {
        return ErrState2json(RetState::parseErr);
    }
}

nlohmann::json SdkProtocolServer::Unsubscribe(const nlohmann::json &params)
{
    try
    {
        for (const auto &item : params)
        {
            string nameSpace = item.at("namespace");
            string method = item.at("method");

            auto client = GetClientInfo();

            subscribeInfo_.at(nameSpace).at(method).clients.erase(client);
            LOG_DEBUG_SDK("unsubscribe, client:{}:{}, namespace:{}, method:{}", GetClientInfo().clientIp, GetClientInfo().clientPort, nameSpace, method);
        }
        return ErrState2json(RetState::ok);
        ;
    }
    catch (const std::exception &)
    {
        return ErrState2json(RetState::parseErr);
    }
}

void SdkProtocolServer::AddTimeoutCall(const std::string &name, int ms, EvtTimeoutCallbackType func)
{
    if (GetMiniServer().AddTimeoutCallback(name, ms, func) == false)
    {
        LOG_ERROR_SDK("can't add timeout callback:{}, please rename it or remove the old!", name);
    }
}

void SdkProtocolServer::RmvSubscribeClient(const NetClientInfo &client)
{
    for (auto &region : subscribeInfo_)
    {
        for (auto &method : region.second)
        {
            method.second.clients.erase(client);
        }
    }
}

void SdkProtocolServer::CheckClientOnlineStatus()
{
    for (auto it = clients_.begin(); it != clients_.end();)
    {
        if (it->lost > 5)
        {
            LOG_DEBUG_SDK("Udp client {}:{} lost", it->clientIp, it->clientPort);
            RmvSubscribeClient(*it);
            it = clients_.erase(it);
        }
        else
        {
            it->lost++; // 增加掉线计数
            ++it;
        }
    }
}

nlohmann::json SdkProtocolServer::RxAreYouOk(const nlohmann::json &result)
{
    UNUSED(result);
    try
    {
        auto info = GetClientInfo();

        // 新增或旧客户端lost清零
        auto it = std::find(clients_.begin(), clients_.end(), info);
        if (it != clients_.end())
        {
            it->lost = 0;
        }
        else
        {
            info.lost = 0;
            clients_.push_back(info);
        }
    }
    catch (std::exception &)
    {
        return false;
    }

    return true;
}
