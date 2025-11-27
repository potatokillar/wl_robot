
#include "miniServer.hpp"

#include <iostream>
using namespace std;

MiniServer::MiniServer() { evtloop_ = make_shared<EventLoop>(); }

MiniServer::~MiniServer() { evtloop_->Stop(); }

bool MiniServer::AddUdpServer(const std::string &name, u16 port)
{
    if (tcpSer_.count(name)) {
        return false;
    }
    if (mcastSer_.count(name)) {
        return false;
    }

    udpSer_[name] = make_shared<UdpServer>(evtloop_, port, name);
    udpSer_[name]->SetReadCallback([this](const NetClientInfo &info, const vector<uint8_t> &data) { this->UdpRecv(info, data); });
    udpSer_[name]->Start();
    return true;
}

bool MiniServer::AddTcpServer(const std::string &name, u16 port)
{
    if (udpSer_.count(name)) {
        return false;
    }
    if (mcastSer_.count(name)) {
        return false;
    }
    tcpSer_[name] = make_shared<TcpServer>(evtloop_, port, name);
    tcpSer_[name]->Start();
    return false;
}

bool MiniServer::AddMcastServer(const std::string &name, const std::string &ip, u16 port)
{
    if (tcpSer_.count(name)) {
        return false;
    }
    if (udpSer_.count(name)) {
        return false;
    }
    mcastSer_[name] = make_shared<Multicast>(evtloop_, ip, port, name);
    // 多播本质也是UDP
    mcastSer_[name]->SetReadCallback([this](const NetClientInfo &info, const vector<uint8_t> &data) { this->UdpRecv(info, data); });
    mcastSer_[name]->Start();
    return true;
}

void MiniServer::Loop() { evtloop_->Loop(); }

void MiniServer::SetSdkParse(SdkDataParseType func) { sdkParseFunc_ = func; }

void MiniServer::SetRawParse(std::string &name, RawDataParseType func) { rawParseFunc_[name] = func; }

void MiniServer::UdpRecv(const NetClientInfo &info, const vector<uint8_t> &data)
{
    if (rawParseFunc_.count(info.serverName)) {
        rawParseFunc_[info.serverName](info, data);
        return;
    }

    ProtocolParse rxData(data);

    if (rxData.Empty() == false) {
        sdkParseFunc_(info, rxData);
    }
}

bool MiniServer::Send(const NetClientInfo &info, const std::vector<u8> &data)
{
    auto name = info.serverName;
    if (udpSer_.count(name)) {
        return udpSer_[name]->Send(info.clientIp, info.clientPort, data);
    }
    if (tcpSer_.count(name)) {
        return tcpSer_[name]->Send(info.clientIp, info.clientPort, data);
    }
    if (mcastSer_.count(name)) {
        return mcastSer_[name]->Send(info.clientIp, info.clientPort, data);
    }
    return false;
}
bool MiniServer::Send(const NetClientInfo &info, const ProtocolConstruct &data) { return Send(info, data.GetData()); }

bool MiniServer::SendAll(const std::string &name, const std::vector<u8> &data)
{
    if (tcpSer_.count(name)) {
        return tcpSer_[name]->Send(data);
    }
    return false;
}

bool MiniServer::AddTimeoutCallback(const std::string &name, int ms, EvtTimeoutCallbackType TimeoutFunc) { return evtloop_->AddTimeoutCb(name, ms, TimeoutFunc); }

bool MiniServer::RmvTimeoutCb(const std::string &name) { return evtloop_->RmvTimeoutCb(name); }