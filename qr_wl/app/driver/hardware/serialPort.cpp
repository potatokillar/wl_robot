#include "serialPort.hpp"

#include "baseline.hpp"

using namespace boost;

/**
 * @description: 串口信息的相关构造函数，含移动构造
 * @param io
 * @return {}
 */
SerialPortInfo::SerialPortInfo(boost::asio::io_context& io) : fd(io) { rxBuf.resize(128); }

SerialPortInfo::SerialPortInfo(SerialPortInfo&& other) : fd(std::move(other.fd)) { swap(other); }
SerialPortInfo& SerialPortInfo::operator=(SerialPortInfo&& rhs)
{
    if (this != &rhs) {
        swap(rhs);
        fd = std::move(rhs.fd);
    }
    return *this;
}
void SerialPortInfo::swap(const SerialPortInfo& other)
{
    txComplete = other.txComplete;
    // txBuf = other.txBuf;
    rxBuf = other.rxBuf;
}

SerialPort::SerialPort() {}

bool SerialPort::Open(const std::string& name, const std::string& dev)
{
    SerialPortInfo port(ioCtx_);
    try {
        port.fd.open(dev);
        ports_.emplace(name, std::move(port));
        SetParam(name);

    } catch (std::exception& e) {
        // LOG_ERROR("[SerialPort] open error: name:{}, error: {}", dev, e.what());
        return false;
    }
    return true;
}

bool SerialPort::SetParam(const std::string& name, u32 baudRate, int dataBits, Parity parity, StopBits nStop)
{
    try {
        auto& port = ports_.at(name);
        // 设置波特率
        port.fd.set_option(boost::asio::serial_port_base::baud_rate(baudRate));

        // 设置数据位
        port.fd.set_option(boost::asio::serial_port_base::character_size(dataBits));

        // 设置停止位
        switch (nStop) {
            case StopBits::one:
                port.fd.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
                break;
            case StopBits::onepointfive:
                port.fd.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::onepointfive));
                break;
            case StopBits::two:
                port.fd.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::two));
                break;
            default:
                break;
        }

        // 设置校验位
        switch (parity) {
            case Parity::none:
                port.fd.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
                break;
            case Parity::odd:
                port.fd.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::odd));
                break;
            case Parity::even:
                port.fd.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::even));
                break;
            default:
                break;
        }
    } catch (std::exception& e) {
        LOG_ERROR("[SerialPort] SetParam error: {}", e.what());
        return false;
    }
    return true;
}

void SerialPort::Close(const std::string& name)
{
    try {
        ports_.at(name).fd.close();
        if (ports_.count(name)) {
            ports_.erase(name);
        }
    } catch (std::exception& e) {
        LOG_ERROR("[SerialPort] Close error: {}", e.what());
    }
}

bool SerialPort::IsOpen(const std::string& name)
{
    try {
        return ports_.at(name).fd.is_open();
    } catch (std::exception& e) {
        LOG_ERROR("[SerialPort] IsOpen error: {}", e.what());
        return false;
    }
    return false;
}

/**
 * @description: 设置写完成函数
 * @param name
 * @param error
 * @param nCnt
 * @return {}
 */
void SerialPort::SetWriteCompleteCallback(const std::string& name, std::function<void(const boost::system::error_code& error, std::size_t nCnt)> func)
{
    try {
        ports_.at(name).txComplete = func;
    } catch (std::exception& e) {
        LOG_ERROR("[SerialPort] SetWriteCompleteCallback error: {}", e.what());
        return;
    }
}

void SerialPort::AsyncWrite(const std::string& name, const std::vector<u8>& data)
{
    try {
        auto& port = ports_.at(name);
        port.fd.async_write_some(boost::asio::buffer(data), [this, name](const boost::system::error_code& error, std::size_t txCnt) {
            if (this->ports_.at(name).txComplete) {
                this->ports_.at(name).txComplete(error, txCnt);
            }
        });
        Start();
    } catch (std::exception& e) {
        LOG_ERROR("[SerialPort] AsyncWrite error: {}", e.what());
        return;
    }
}

void SerialPort::AsyncRead(const std::string& name, std::function<void(const std::vector<uint8_t>&)> func)
{
    try {
        auto& port = ports_.at(name);
        port.fd.async_read_some(boost::asio::buffer(port.rxBuf), [this, name, func](const boost::system::error_code& error, std::size_t rxCnt) {
            if (!error) {
                func(std::vector<uint8_t>(ports_.at(name).rxBuf.begin(), ports_.at(name).rxBuf.begin() + rxCnt));
            } else {
            }
            AsyncRead(name, func);  // 重新开启
        });
        Start();
    } catch (std::exception& e) {
        LOG_ERROR("[SerialPort] AsyncRead error: {}", e.what());
    }
}

void SerialPort::Write(const std::string& name, const std::vector<u8>& data)
{
    try {
        boost::system::error_code ec;
        ports_.at(name).fd.write_some(boost::asio::buffer(data), ec);
    } catch (std::exception& e) {
        LOG_ERROR("[SerialPort] Write error: {}", e.what());
    }
}

std::vector<u8> SerialPort::Read(const std::string& name)
{
    try {
        auto& port = ports_.at(name);
        size_t rxCnt = port.fd.read_some(boost::asio::buffer(port.rxBuf));
        return std::vector<uint8_t>(port.rxBuf.begin(), port.rxBuf.begin() + rxCnt);
    } catch (std::exception& e) {
        LOG_ERROR("[SerialPort] Read error: {}", e.what());
        return {};  // 返回空
    }
}

/**
 * @description: 设置内部缓存区大小
 * @param name
 * @param rxCnt
 * @param txCnt
 * @return {}
 */
void SerialPort::SetBufferSize(const std::string& name, u32 rxCnt, u32 txCnt)
{
    (void)txCnt;
    try {
        auto& port = ports_.at(name);
        port.rxBuf.resize(rxCnt);
        port.txBuf.resize(txCnt);
    } catch (std::exception& e) {
        LOG_ERROR("[SerialPort] SetBufferSize error: {}", e.what());
    }
}

void SerialPort::Loop() { ioCtx_.run(); };

void SerialPort::Start()
{
    if (thread_.joinable() == false) {
        thread_ = std::thread(&SerialPort::Loop, this);
    }
}