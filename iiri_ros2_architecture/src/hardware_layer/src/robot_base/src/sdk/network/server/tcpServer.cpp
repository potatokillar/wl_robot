#include "tcpServer.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>

using namespace ::std;

TcpServer::TcpServer(std::shared_ptr<EventLoop> loop, int port, const std::string &name) : loop_(loop), port_(port), name_(name) {}

TcpServer::~TcpServer() {}

void TcpServer::Start()
{
    struct sockaddr_in server_addr;
    // 填充sockaddr_in结构
    bzero(&server_addr, sizeof(struct sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(port_);

    listener = evconnlistener_new_bind(loop_->evtBase_,
                                       ListenCb,
                                       this,
                                       LEV_OPT_CLOSE_ON_FREE | LEV_OPT_REUSEABLE,
                                       MAX_CONNECT,
                                       (struct sockaddr *)(&server_addr),
                                       sizeof(struct sockaddr));
}

/****************************************************************
 * 功能：监听事件回调函数
 * 输入：@1
 * 输出：@1
 ****************************************************************/
void TcpServer::ListenCb(struct evconnlistener *listener, evutil_socket_t fd, struct sockaddr *addr, int socklen, void *ptr)
{
    TcpServer *myClass = (TcpServer *)ptr;
    myClass->ListenCb(listener, fd, addr, socklen);
}

void TcpServer::ListenCb(struct evconnlistener *listener, evutil_socket_t fd, struct sockaddr *addr, int socklen)
{
    (void)listener;
    (void)socklen;
    struct sockaddr_in *newClient = (sockaddr_in *)addr;
    char ipv4str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &newClient->sin_addr, ipv4str, INET_ADDRSTRLEN);
    // LOG_INFO_SDK("accept new connect, ip:{}, port:{:d}", ipv4str, ntohs(newClient->sin_port));

    // 添加新事件
    struct bufferevent *bev = bufferevent_socket_new(loop_->evtBase_, fd, BEV_OPT_CLOSE_ON_FREE | BEV_OPT_THREADSAFE);

    // 设置缓冲区回调
    bufferevent_setcb(bev, TcpServer::ReadCb, NULL, TcpServer::EventCb, this);

    // 使能缓冲区
    bufferevent_enable(bev, EV_READ);

    ClientInfo client;
    client.bev = bev;
    client.ipstr = ipv4str;
    client.port = ntohs(newClient->sin_port);
    clientSet_.push_back(client);
}

/****************************************************************
 * 功能：读回调，直接调用注册的回调函数
 * 输入：@1
 * 输出：@1
 ****************************************************************/
void TcpServer::ReadCb(struct bufferevent *bev, void *ptr)
{
    TcpServer *myClass = (TcpServer *)ptr;

    if (myClass->ReadFunc_) {
        myClass->ReadFunc_(bev);
    }
}

/**
 * @description: 发送数据给指定的客户端
 * @param &ip
 * @param port
 * @param &data
 * @return {}
 */
bool TcpServer::Send(const std::string &ip, uint16_t port, const std::vector<uint8_t> &data)
{
    for (auto v : clientSet_) {
        if ((ip == v.ipstr) && (port == v.port)) {
            return bufferevent_write(v.bev, data.data(), data.size());
        }
    }
    return false;
}

/**
 * @description: 发送数据给所有在线的客户端，tcp是会保存客户端信息，因此可以使用这个函数
 * @param &data
 * @return {}
 */
bool TcpServer::Send(const std::vector<uint8_t> &data)
{
    bool ret = false;
    for (auto v : clientSet_) {
        ret = bufferevent_write(v.bev, data.data(), data.size());
    }
    return ret;
}

void TcpServer::EventCb(struct bufferevent *bev, short what, void *ptr)
{
    TcpServer *myClass = (TcpServer *)ptr;
    myClass->EventCb(bev, what);
}
void TcpServer::EventCb(struct bufferevent *bev, short what)
{
    if ((what & BEV_EVENT_CONNECTED) != BEV_EVENT_CONNECTED) {
        // LOG_WARN_SDK("client exception disconnet!");
        for (auto it = clientSet_.begin(); it != clientSet_.end(); it++) {
            if (it->IsMe(bev) == true) {
                bufferevent_disable(bev, EV_READ);
                bufferevent_free(bev);
                clientSet_.erase(it);
                break;  // 删除后迭代器失效
            }
        }
    }
}

void TcpServer::TimeoutCb(evutil_socket_t fd, short events, void *ptr)
{
    TcpServer *myClass = (TcpServer *)ptr;
    if (myClass->TimeoutFunc_) {
        myClass->TimeoutFunc_(fd, events);
    }
}

void TcpServer::SetTimeoutCallback(int ms, TcpTimeoutCallbackType TimeoutFunc)
{
    TimeoutFunc_ = TimeoutFunc;
    // 创建一个定时器
    struct timeval tv;
    tv.tv_sec = ms / 1000;
    tv.tv_usec = ms % 1000 * 1000;
    evt = event_new(loop_->evtBase_, -1, EV_PERSIST, &TcpServer::TimeoutCb, this);
    event_add(evt, &tv);
}
