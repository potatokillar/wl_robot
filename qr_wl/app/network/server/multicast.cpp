
#include "multicast.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <errno.h>
#include <cstring>

#include "baseline.hpp"

using namespace std;

Multicast::Multicast(std::shared_ptr<EventLoop> event, const std::string &ipv4, int port, const std::string &name)
{
    loop_ = event;
    ipv4_ = ipv4;
    port_ = port;
    name_ = name;
}

void Multicast::Start()
{
    sockFd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sockFd_ < 0) {
        LOG_WARN_SDK("multicast socket error");
    }

    // 设置端口复用选项，允许多个进程绑定同一组播地址
    int reuse = 1;
    if (setsockopt(sockFd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        LOG_WARN_SDK("Failed to set SO_REUSEADDR for multicast on port %d: %s", port_, strerror(errno));
    }

    struct sockaddr_in localaddr;
    bzero(&localaddr, sizeof(localaddr));
    localaddr.sin_family = AF_INET;
    localaddr.sin_port = htons(port_);
    localaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(sockFd_, (struct sockaddr *)&localaddr, sizeof(struct sockaddr_in)) < 0) {
        LOG_WARN_SDK("multicast bind error on port %d: %s (errno=%d)", port_, strerror(errno), errno);
    }

    int ttl = 5;
    if (setsockopt(sockFd_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) < 0) {
        LOG_WARN_SDK("multicast set ttl error");
    }

    int loop = 0;
    if (setsockopt(sockFd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop)) < 0) {
        LOG_WARN_SDK("multicast set loop error");
    }

    struct ip_mreq mreq;
    bzero(&mreq, sizeof(mreq));
    inet_pton(AF_INET, ipv4_.c_str(), &mreq.imr_multiaddr);

    if (setsockopt(sockFd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        LOG_WARN_SDK("multicast add error");
    }

    evt_ = event_new(loop_->evtBase_, sockFd_, EV_READ | EV_PERSIST, &Multicast::ReadCb, this);
    event_add(evt_, 0);
}

void Multicast::ReadCb(evutil_socket_t fd, short what, void *ptr)
{
    Multicast *myClass = (Multicast *)ptr;
    myClass->ReadCb(fd, what);
}

void Multicast::ReadCb(evutil_socket_t fd, short what)
{
    (void)what;
    struct sockaddr_in recvAddr;
    socklen_t socklen = sizeof(struct sockaddr);

    char rxbuf[100];

    int rxLen = recvfrom(fd, &rxbuf, sizeof(rxbuf), 0, (struct sockaddr *)&recvAddr, &socklen);
    if (rxLen == -1) {
        LOG_WARN("find server recv error");
        return;
    }

    if (ReadFunc_) {
        auto [ret, ip, port] = Sockaddr2String(recvAddr);
        if (ret == true) {
            vector<u8> data(rxbuf, rxbuf + rxLen);
            NetClientInfo info;
            info.serverName = name_;
            info.clientIp = ip;
            info.clientPort = port;
            ReadFunc_(info, data);
        }
    }
}

bool Multicast::Send(const std::string &ip, u16 port, const std::vector<uint8_t> &data)
{
    auto [ret, sockaddr] = String2Sockaddr(ip, port);

    if (ret == true) {
        if (sendto(sockFd_, data.data(), data.size(), 0, (struct sockaddr *)&sockaddr, sizeof(struct sockaddr)) >= 0) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}
