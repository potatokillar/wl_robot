
#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "baseline.hpp"
#include "eventloop.hpp"
#include "multicast.hpp"
#include "netType.hpp"
#include "protocol.hpp"
#include "tcpServer.hpp"
#include "udpServer.hpp"

using SdkDataParseType = std::function<void(const NetClientInfo& info, const ProtocolParse& data)>;

using RawDataParseType = std::function<void(const NetClientInfo& info, const std::vector<u8>& data)>;

class MiniServer
{
public:
    static MiniServer& GetInstance()
    {
        static MiniServer instance;
        return instance;
    }

    bool AddUdpServer(const std::string& name, u16 port);
    bool AddTcpServer(const std::string& name, u16 port);
    bool AddMcastServer(const std::string& name, const std::string& ip, u16 port);

    void SetSdkParse(SdkDataParseType func);
    void SetRawParse(std::string& name, RawDataParseType func);
    bool AddTimeoutCallback(const std::string& name, int ms, EvtTimeoutCallbackType TimeoutFunc);
    bool RmvTimeoutCb(const std::string& name);

    void Loop();

    bool Send(const NetClientInfo& info, const std::vector<u8>& data);
    bool Send(const NetClientInfo& info, const ProtocolConstruct& data);

    bool SendAll(const std::string& name, const std::vector<u8>& data);

private:
    MiniServer();
    virtual ~MiniServer();
    void UdpRecv(const NetClientInfo& info, const std::vector<u8>& data);
    void TcpRecv(const NetClientInfo& info, const std::vector<u8>& data);

private:
    std::shared_ptr<EventLoop> evtloop_;
    std::map<std::string, std::shared_ptr<UdpServer>> udpSer_;
    std::map<std::string, std::shared_ptr<TcpServer>> tcpSer_;
    std::map<std::string, std::shared_ptr<Multicast>> mcastSer_;

    SdkDataParseType sdkParseFunc_;
    std::map<std::string, RawDataParseType> rawParseFunc_;
};

inline MiniServer& GetMiniServer() { return MiniServer::GetInstance(); }
