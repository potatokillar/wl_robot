
#include "arpa/inet.h"
#include "netType.hpp"

std::pair<bool, struct sockaddr_in> String2Sockaddr(const std::string &ip, uint16_t port)
{
    struct sockaddr_in sockaddr;

    if (inet_pton(AF_INET, ip.c_str(), &(sockaddr.sin_addr)) != 1) {
        return {false, sockaddr};
    }

    sockaddr.sin_port = htons(port);

    sockaddr.sin_family = AF_INET;

    return {true, sockaddr};
}
std::tuple<bool, std::string, uint16_t> Sockaddr2String(const struct sockaddr_in &sockaddr)
{
    char ipBuffer[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &(sockaddr.sin_addr), ipBuffer, INET_ADDRSTRLEN) == nullptr) {
        return {false, "null", 0};
    }
    std::string ip = ipBuffer;

    uint16_t port = ntohs(sockaddr.sin_port);

    return {true, ip, port};
}