
#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "baseline.hpp"
#include "miniServer.hpp"
#include "netMsg.hpp"
#include "protocol.hpp"
#include "robotState.hpp"
#include "tcpServer.hpp"

/**
 * @brief 设备公用的debug数据的二进制数据包类型定义，包含imu传感器数据，以及watch数据。
 */
struct DeviceDebugDataPacket
{
    BinPacketHeader imuHead{"rpy", sizeof(msg::net::rpy_debug)};
    msg::net::rpy_debug imuData;
    BinPacketHeader watchHead{"watch", sizeof(msg::net::watch_data)};
    msg::net::watch_data debugData;
};

/**
 * @brief 负责生成公用debug数据的类，也是三种业务设备发送debug数据都需要使用的基类。
 * 三种业务，各自派生各自的XXXDebugServer类，使用各自独立的二进制数据包类型，与基类所代表的公用数据相互独立。
 */
class DeviceDebugServer
{
public:
    DeviceDebugServer();
    virtual ~DeviceDebugServer() = default;

    void SaveRpyCmd(const msg::imu_data &data);
    void SaveImuData(const msg::imu_data &data);
    void SaveDebugData(const msg::watch_data &data);
    void SaveDebugDataA(const msg::watch_data_a &data);
    void SaveDebugDataB(const msg::watch_data_b &data);
    void SaveDebugDataC(const msg::watch_data_c &data);

    /**
     * @description: GetData()是DebugServer模块的多态接口。
     * 派生类在重写时，要在内部调用一下基类的GetDeviceDebugData()方法，准备基类负责的公用debug数据。
     * 然后与派生类特有的debug数据，拼接二者的字符数组，构成完整的结果。
     * @return {}
     */
    virtual std::pair<bool, const std::vector<uint8_t> &> GetData() = 0;

protected:
    /**
     * @description: 生成基类debug数据的二进制数据包
     * @return {}
     */
    std::pair<bool, const std::vector<uint8_t> &> GetDeviceDebugData();
    std::vector<uint8_t> device_data_;  // 派生类也需要访问，把这个数据拼接成完整的字符数组

private:
    DeviceDebugDataPacket device_debug_packet_;  // 仅在基类内部使用
};
