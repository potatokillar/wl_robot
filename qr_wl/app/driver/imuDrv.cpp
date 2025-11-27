#include "imuDrv.hpp"

#include <chrono>

#include "baseline.hpp"
#include "deviceParam.hpp"
#include "robotMedia.hpp"

using namespace std;

std::pair<bool, msg::imu_data> ImuNode::ParseData(std::queue<uint8_t> &q)
{
    bool checkIMUAbnormality = false;
    bool dataValid = false;
    msg::imu_data imuData;  // update默认为false
    while (q.size() >= 11) {
        if (q.front() == 0x55) {
            uint8_t sum = 0;
            uint8_t buf[11] = {0};
            for (int i = 0; i < 11; ++i) {
                buf[i] = q.front();
                q.pop();

                if (i != 10) {
                    sum += buf[i];
                    // fmt::print("{:#04x} ", buf[i]);
                }
            }
            // fmt::print("sum:{:#04x}\n", sum);
            // 校验和检查
            if (sum == buf[10]) {
                // LOG_WARN(" check");
                uint8_t kind = buf[1];
                uint8_t *payload = &buf[2];
                int16_t sData[4];
                memcpy(&sData[0], &buf[2], 8);  // 注，系统必须是小端才行
                switch (kind) {
                    case 0x51:
                        imuInfo.packetCount1[0]++;
                        imuInfo.packetCount[0]++;
                        for (int i = 0; i < 3; ++i) {
                            imuDataIncomplete_.acc[i] = (int16_t)(payload[2 * i] | payload[2 * i + 1] << 8) / 32768.0 * 16.0 * 9.8;
                        }
                        break;
                    case 0x52:
                        imuInfo.packetCount1[1]++;
                        imuInfo.packetCount[1]++;
                        for (int i = 0; i < 3; ++i) {
                            imuDataIncomplete_.gyro[i] = (int16_t)(payload[2 * i] | payload[2 * i + 1] << 8) / 32768.0 * 2000.0;
                            imuDataIncomplete_.gyro[i] = imuDataIncomplete_.gyro[i] / 180.0 * M_PI;
                        }
                        break;
                    case 0x53:
                        imuInfo.packetCount1[2]++;
                        imuInfo.packetCount[2]++;
                        for (int i = 0; i < 3; ++i) {
                            imuDataIncomplete_.ang[i] = (int16_t)(payload[2 * i] | payload[2 * i + 1] << 8) / 32768.0 * 3.14159;
                        }

                        break;

                    case 0x59:
                        for (int i = 0; i < 4; ++i) {
                            imuDataIncomplete_.quat[i] = (int16_t)(payload[2 * i] | payload[2 * i + 1] << 8) / 32768.0;
                        }
                        checkIMUAbnormality = true;
                        dataValid = true;
                        break;
                    default:
                        break;
                }

                if (dataValid == true) {
                    imuData = imuDataIncomplete_;
                    if (checkIMUAbnormality == true) {
                        checkIMUAbnormality = false;

                        constexpr int OVER_TIME = 3000;
                        if (imuInfo.imuTime1 == 0) {
                            imuInfo.imuTime1 = TimerTools::GetNowTickMs();
                        } else {
                            if ((TimerTools::GetNowTickMs() - imuInfo.imuTime1) > OVER_TIME) {
                                imuInfo.imuTime1 = TimerTools::GetNowTickMs();
                                imuInfo.packetCount1[0] = 0;
                                imuInfo.packetCount1[1] = 0;
                                imuInfo.packetCount1[2] = 0;
                            }
                            if ((TimerTools::GetNowTickMs() - imuInfo.imuTime1) > (OVER_TIME * 1.1)) {
                                imuInfo.imuTime1 = TimerTools::GetNowTickMs();
                            }
                        }
                    }
                }
            }
            // else {
            //     LOG_WARN("check err imu" );
            // }
        } else {
            q.pop();
        }
    }
    return {dataValid, imuData};
}

void ImuNode::Init()
{
    isOk_ = false;
    if (thread_.joinable()) {
        thread_.join();  // 必须等上一个线程结束
    }
    for (auto dev : GetDevParam().imuDev) {
        uart_ = std::make_unique<Uart>(dev);

        if (uart_->IsOpen() == true) {
            uart_->SetParam(115200, 8, 'N', 1);
            uart_->SetBlockCnt(11);

            imuInfo.startTime = TimerTools::GetNowTickMs();
            imuInfo.lastReadOk = imuInfo.startTime;
            isOk_ = true;
            thread_ = std::thread(&ImuNode::Loop, this);  // 析构由基类负责
            break;                                        // 成功打开则不再查询下一个设备号
        }
    }

    if (isOk_ == false) {
        AddMediaPackage(MediaTaskPackage("sensorError"));
    }
}

void ImuNode::Loop()
{
    while (isOk_) {
        // 该变量用于判断是否接收到了一个正确的包，而不是一组（完整包含0x51 0x52 0x53）的包
        //  读取串口数据并存入队列
        auto data = uart_->Read(1);  // 超时1ms

        for (const auto &var : data) {
            uartData_.push(var);
        }
        // 解析
        auto [ImuDataValid, imuData] = ParseData(uartData_);
        if (ImuDataValid == true) {
            // 发送
            MsgTrySend("qr::imuData", imuData);
            imuInfo.lastReadOk = TimerTools::GetNowTickMs();
            // LOG_DEBUG("IMU {}: {} {} {}", imuData.quat[0], imuData.quat[1], imuData.quat[2], imuData.quat[3]);
        }
        ErrorDetect();
    }
}

void ImuNode::ErrorDetect()
{
    auto nowTime = TimerTools::GetNowTickMs();
    auto duration = nowTime - imuInfo.lastReadOk;  // 与上一个收到正确的包的时间的间隔
    auto runTime = nowTime - imuInfo.startTime;    // 与系统初始化的时间差值
    constexpr int ALIVE_TIME = 1000;
    // IMU刚启动的那段时间内，不判断错误
    if (runTime < ALIVE_TIME * 1.1) {
        return;
    }

    // 断连超过一定时间，存在两种情况
    if (duration > ALIVE_TIME) {
        if (imuInfo.imuHasError == false) {
            if (imuInfo.initPrintFlag == false) {
                LOG_ERROR("imu Init failure, runTime = {:#d}", runTime);
                imuInfo.initPrintFlag = true;
                imuInfo.imuHasError = true;
                AddMediaPackage(MediaTaskPackage("sensorError"));
                // MsgSend msg("qr::imu_error");
                // msg.Send(true);
                MsgTrySend("qr::imu_error", true);
            } else {
                LOG_ERROR("imu can't read data in 1s!, duration ={:#d}", duration);
                imuInfo.imuHasError = true;
                AddMediaPackage(MediaTaskPackage("sensorError"));
                // MsgSend msg("qr::imu_error");
                // msg.Send(true);
                MsgTrySend("qr::imu_error", true);
                if (uart_->IsValid() == false) {
                    // 若是设备号丢失导致的问题，则重新查找设备号，因此不支持非USB串口
                    // 当前线程需要停止，所以需要新线程启动查找
                    std::thread([this]() { Init(); }).detach();
                }
            }
        }
    }

    else {
        if (imuInfo.initPrintFlag == false) {
            LOG_INFO("imu Init success");
            imuInfo.initPrintFlag = true;
            // MsgSend msg("qr::imu_error");
            // msg.Send(false);
            MsgTrySend("qr::imu_error", false);
        }

        if (imuInfo.imuHasError == true) {
            LOG_INFO("imu read data again");
            imuInfo.imuHasError = false;
            // MsgSend msg("qr::imu_error");
            // msg.Send(false);
            MsgTrySend("qr::imu_error", false);
        }
    }
}
