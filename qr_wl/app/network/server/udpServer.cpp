
#include "udpServer.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>

#include <cstring>

#include "baseline.hpp"
using namespace std;

UdpServer::UdpServer(std::shared_ptr<EventLoop> loop, int port, const std::string &name) : loop_(loop), port_(port), name_(name) {}

UdpServer::~UdpServer() {}

void UdpServer::Start()
{
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);

    // 设置端口复用选项，允许快速重启
    int reuse = 1;
    if (setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        LOG_WARN_SDK("Failed to set SO_REUSEADDR on port %d: %s", port_, strerror(errno));
    }

    struct sockaddr_in server_addr;
    // 填充sockaddr_in结构
    bzero(&server_addr, sizeof(struct sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(port_);

    if (bind(fd_, (struct sockaddr *)&server_addr, sizeof(server_addr))) {
        LOG_WARN_SDK("bind error on port %d: %s (errno=%d)", port_, strerror(errno), errno);
    }

    evt = event_new(loop_->evtBase_, fd_, EV_READ | EV_PERSIST, &UdpServer::ReadCb, this);
    event_add(evt, 0);
}

void UdpServer::ReadCb(evutil_socket_t fd, short what, void *ptr)
{
    UdpServer *myClass = (UdpServer *)ptr;
    myClass->ReadCb(fd, what);
}
void UdpServer::ReadCb(evutil_socket_t fd, short what)
{
    (void)what;
    socklen_t socklen = sizeof(struct sockaddr);
    NetClientInfo info;
    info.serverName = name_;

    struct sockaddr_in addr;

    int len = recvfrom(fd, rxbuf_, sizeof(rxbuf_), 0, (struct sockaddr *)&addr, &socklen);
    if (len == -1) {
        LOG_WARN_SDK("find server recv error");
        return;
    }

    if (ReadFunc_) {
        vector<u8> data(rxbuf_, rxbuf_ + len);
        NetClientInfo info;
        auto [ret, ip, port] = Sockaddr2String(addr);
        if (ret == true) {
            info.clientIp = ip;
            info.clientPort = port;
            info.serverName = name_;
            ReadFunc_(info, data);
        }
    }
}

bool UdpServer::Send(const std::string &ip, u16 port, const std::vector<uint8_t> &data)
{
    auto [ret, sockaddr] = String2Sockaddr(ip, port);

    if (ret == true) {
        if (sendto(fd_, data.data(), data.size(), 0, (struct sockaddr *)&sockaddr, sizeof(struct sockaddr)) >= 0) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}
