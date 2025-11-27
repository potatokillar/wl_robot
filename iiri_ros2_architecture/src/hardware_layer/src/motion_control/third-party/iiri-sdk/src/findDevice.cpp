/*
 * @Author: 唐文浩
 * @Date: 2022-06-27
 * @LastEditors: 唐文浩-0036
 * @LastEditTime: 2023-07-26
 * @Description:
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#include "findDevice.hpp"

#include <boost/asio.hpp>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "nlohmann/json.hpp"
#include "protocol.hpp"

using namespace std;
using namespace boost;
namespace iiri
{

    class FindDeviceImpl
    {
    public:
        FindDeviceImpl(uint32_t interval_ms) : socket_(ioContext_, asio::ip::udp::endpoint(asio::ip::udp::v4(), 0))
        {
            if (interval_ms < 10)
            {
                interval_ = 10;
            }
            else if (interval_ms > 2000)
            {
                interval_ = 2000;
            }
            else
            {
                interval_ = interval_ms;
            }

            serverEndpoint_ = asio::ip::udp::endpoint(asio::ip::address_v4::from_string("224.0.0.88"), 20444);
            thread_ = thread(&FindDeviceImpl::Loop, this);
        }
        ~FindDeviceImpl()
        {
            Stop();

            if (thread_.joinable())
            {
                thread_.join();
            }
        }

        void Stop()
        {
            socket_.cancel(); // 会导致正在运行中的socket抛出异常
            running_ = false;
        }

        void Start() {}

        void Loop()
        {
            while (running_)
            {
                try
                {
                    string rpcTx = "{ \"device\":{  \"method\": \"FindDevice\"  } }"; // rpc请求字符串
                    ProtocolConstruct txData(rpcTx);
                    socket_.send_to(boost::asio::buffer(txData.GetData()), serverEndpoint_);

                    boost::asio::ip::udp::endpoint senderEndpoint;
                    size_t bytesReceived = socket_.receive_from(boost::asio::buffer(rxData_), senderEndpoint);

                    ProtocolParse rxData(vector<uint8_t>(rxData_.begin(), rxData_.begin() + bytesReceived));

                    std::string name;
                    string ip = senderEndpoint.address().to_string();
                    std::string model;
                    if (rxData.Empty() == false)
                    {
                        nlohmann::json r1 = nlohmann::json::parse(rxData.GetData());
                        try
                        {
                            name = r1.at("device").at("result").at("name");
                            model = r1.at("device").at("result").at("model");

                            // 只有解析成功才是合法的地址，在andriod下，可能出现非预期的组播回应
                            bool isExist = false;
                            for (auto &var : result_)
                            {
                                if (var.ip == ip)
                                {
                                    isExist = true;
                                    break;
                                }
                            }
                            if (isExist == false)
                            {
                                std::lock_guard<std::mutex> lock(mutex_);
                                result_.emplace_back(ip, name, model);
                            }
                        }
                        catch (std::exception &)
                        {
                            cout << "parse exception" << endl;
                        }
                    }
                }
                catch (boost::system::system_error &e)
                {
                    std::cout << "Error occurred: " << e.what() << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(interval_));
            }
        }
        vector<FindDeviceResult> GetResult()
        {
            std::lock_guard<std::mutex> lock(mutex_);

            return result_;
        }

    public:
    private:
        std::vector<FindDeviceResult> result_;
        std::mutex mutex_;
        std::thread thread_;
        uint32_t interval_ = 0;
        // 最大UDP数据包长度，MTU(1500) - IP头(20) -UDP头(8)
        std::array<uint8_t, 1472> rxData_;
        boost::asio::io_service ioContext_;
        boost::asio::ip::udp::socket socket_;
        boost::asio::ip::udp::endpoint serverEndpoint_;
        bool running_ = true;
    };

    /**
     * @description: 构造函数，构造完成，相关功能就已经起来了
     * @param interval_ms 扫描间隔ms，范围10~2000ms。默认200ms
     * @return {}
     */
    FindDevice::FindDevice(uint32_t interval_ms) { impl_ = std::make_shared<FindDeviceImpl>(interval_ms); }

    /**
     * @description: 析构
     * @return {}
     */
    FindDevice::~FindDevice() {}

    /**
     * @description: 获取结果，因为扫描需要时间，务必在类起来后等待一段时间才查询
     * @return {}
     */
    vector<FindDeviceResult> FindDevice::GetResult() { return impl_->GetResult(); }

    void FindDevice::Loop() { impl_->Loop(); }

    /**
     * @description: 当超时时间设置为0时，必须stop才能读取结果
     * @return {}
     */
    void FindDevice::Stop() { impl_->Stop(); }

    void FindDevice::Start() { impl_->Start(); }

} // namespace iiri
