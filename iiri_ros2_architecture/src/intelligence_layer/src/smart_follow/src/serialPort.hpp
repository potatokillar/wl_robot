/*
 * @Author: 唐文浩
 * @Date: 2023-12-12
 * @LastEditors: 唐文浩
 * @LastEditTime: 2024-12-13
 * @Description: 串口事件类
 *
 * Copyright (c) 2022-2023 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once
#include <boost/asio.hpp>
#include <functional>
#include <map>
#include <string>
#include <thread>
#include <vector>

enum class StopBits
{
    one = boost::asio::serial_port_base::stop_bits::one,
    onepointfive = boost::asio::serial_port_base::stop_bits::onepointfive,
    two = boost::asio::serial_port_base::stop_bits::two,
};

enum class Parity
{
    none = boost::asio::serial_port_base::parity::none,
    odd = boost::asio::serial_port_base::parity::odd,
    even = boost::asio::serial_port_base::parity::even,
};

struct SerialPortInfo
{
    boost::asio::serial_port fd;
    std::function<void(const boost::system::error_code &, std::size_t)> txComplete;
    // std::vector<uint8_t> txBuf;   // 发送用不到
    std::vector<uint8_t> rxBuf;
    SerialPortInfo(boost::asio::io_context &io);

    SerialPortInfo(const SerialPortInfo &) = delete; // 不支持复制
    SerialPortInfo &operator=(const SerialPortInfo &) = delete;
    SerialPortInfo(SerialPortInfo &&other); // 但可以移动
    SerialPortInfo &operator=(SerialPortInfo &&rhs);
    void swap(const SerialPortInfo &other);
};

class SerialPort
{
public:
    static SerialPort &GetInstance()
    {
        static SerialPort instance;
        return instance;
    }
    bool Open(const std::string &name, const std::string &dev);
    bool SetParam(const std::string &name, uint32_t nSpeed = 9600, int nBits = 8, Parity parity = Parity::none, StopBits nStop = StopBits::one);
    void Close(const std::string &name);
    bool IsOpen(const std::string &name);

    void SetWriteCompleteCallback(const std::string &name, std::function<void(const boost::system::error_code &, std::size_t)> func);
    void AsyncWrite(const std::string &name, const std::vector<uint8_t> &data);
    void AsyncRead(const std::string &name, std::function<void(const std::vector<uint8_t> &)> func);
    void Write(const std::string &name, const std::vector<uint8_t> &data);
    std::vector<uint8_t> Read(const std::string &name);

    void SetBufferSize(const std::string &name, uint32_t rxCnt, uint32_t txCnt = 0);

private:
    SerialPort();
    void Start();
    boost::asio::io_context ioCtx_;
    boost::asio::io_service::work work_{ioCtx_};
    std::map<std::string, SerialPortInfo> ports_; // 串口集合
    std::thread thread_;

    void Loop();
};

inline SerialPort &GetSerialPort() { return SerialPort::GetInstance(); }