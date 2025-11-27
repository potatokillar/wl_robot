
#include "canProtocol.hpp"

#include "deviceParam.hpp"

using namespace std;

static constexpr u8 UART_TAG = 0xAA;
static constexpr u8 UART_TOTAL = UART_LEN + 2 + 1;

/**
 * @description:
 * @return {}
 */
std::vector<u8> SensorCanData::GetVector() const
{
    std::vector<u8> data;
    data.push_back(UART_TAG);
    data.push_back(UART_LEN);
    data.push_back(version << 5);
    data.push_back(boardId);
    data.push_back(cmd);
    for (auto i = 0; i < 7; i++) {
        data.push_back(payload[i]);
    }
    data.push_back(add);

    return data;
}

CanProtocol::CanProtocol()
{
    if (GetDevParam().sensorDev.empty() == false) {
        LOG_TRACE_SENSOR("GetDevParam().sensorDev[0] {}", GetDevParam().sensorDev[0]);
        if (GetSerialPort().Open(UART_NAME, GetDevParam().sensorDev[0]) == false) {
            LOG_TRACE_SENSOR("open error");
            return;
        }
        if (GetSerialPort().SetParam(UART_NAME, 115200) == false) {
            LOG_TRACE_SENSOR("set param error");
            return;
        }
        GetSerialPort().AsyncRead(UART_NAME, [this](const std::vector<u8> &data) { this->RecvDeal(data); });

        GetSerialPort().SetWriteCompleteCallback(UART_NAME, [this](const boost::system::error_code &e, std::size_t t) { this->SendComplete(e, t); });

    } else {
        LOG_TRACE_SENSOR("no serial port");
    }
}

CanProtocol::~CanProtocol() {}

/**
 * @description:
 * @param data
 * @return {}
 */
void CanProtocol::Send(const SensorCanData &data)
{
    lock_guard<mutex> lock(txMutex_);
    txFifo_.push(data.GetVector());

    if (noMsgSend == true) {
        noMsgSend = false;
        GetSerialPort().AsyncWrite(UART_NAME, txFifo_.front());
    }
}

bool CanProtocol::AddRxCall(u8 cmd, CanRxCallType func)
{
    if (funcMap_.find(cmd) != funcMap_.end()) {
        return false;
    }

    funcMap_[cmd] = func;
    return true;
}

/**
 * @description: 接收回调
 * @return {}
 */
void CanProtocol::RecvDeal(const std::vector<u8> &data)
{
    for (const auto &var : data) {
        rxFifo_.push(var);
    }

    while (rxFifo_.size() >= UART_TOTAL) {
        if (rxFifo_.front() == UART_TAG) {
            rxFifo_.pop();

            // u8 len = rxFifo_.front();
            rxFifo_.pop();

            u8 add = 0;
            for (size_t i = 0; i < UART_LEN; i++) {
                oneMsg[i] = rxFifo_.front();
                rxFifo_.pop();
            }
            // 计算累加和
            for (size_t i = 0; i < 10; i++) {
                add += oneMsg[i];
            }
            // 计较累加和
            if (add == rxFifo_.front()) {
                SensorCanData can(oneMsg);
                if (funcMap_.find(can.cmd) != funcMap_.end()) {
                    LOG_TRACE_SENSOR("[canPro] uart rx: {}", can.cmd);
                    funcMap_[can.cmd](can);  // 调用对应的处理函数
                    static int RXNum = 0;
                    RXNum++;
                } else {
                    LOG_TRACE_SENSOR(" can.cmd err");
                }
            } else {
                LOG_TRACE_SENSOR("err add={:#04x} ", add);
                LOG_TRACE_SENSOR("add ={:#04x} ", rxFifo_.front());
            }
            rxFifo_.pop();
        } else {
            rxFifo_.pop();
        }
    }
}

void CanProtocol::SendComplete(const boost::system::error_code &, std::size_t)
{
    lock_guard<mutex> lock(txMutex_);  // 该函数由异步调用，和Send不是一个线程
    txFifo_.pop();
    if (txFifo_.empty() == false) {
        GetSerialPort().AsyncWrite(UART_NAME, txFifo_.front());
    } else {
        noMsgSend = true;
    }
}