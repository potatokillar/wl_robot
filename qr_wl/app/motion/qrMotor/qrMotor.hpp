

#pragma once

#include "baseline.hpp"
#include "qrMotorType.hpp"
#include "spi2can.hpp"
#include "transMotorMit.hpp"

class QrMotor
{
public:
    virtual ~QrMotor() {}

    virtual void Start() {}
    virtual void Stop() {}
    virtual void Run() {}
    virtual void Enable() {}
    virtual void Disable() {}

    /**
     * @description: 设置参数，参数可以是功能，可以是选项，具体根据type来
     * @param type 类型改成string的目的在于，更加灵活
     * @param opt
     * @return {} 必须告知失败原因，目前一般是noSupport
     */
    virtual RetState SetParam(const std::string &type, std::any opt)
    {
        (void)type;
        (void)opt;
        return RetState::noSupport;
    }

    /**
     * @description: 获取参数，注意这个不强求和SetParam一一对应
     * @param &type
     * @return {}
     */
    virtual Result<std::any> GetParam(const std::string &type)
    {
        (void)type;
        Result<std::any> ret;
        ret.first = RetState::noExist;
        return ret;
    }

    /**
     * @description: 设置回调函数
     * @param type 回调函数的类别
     * @param func
     * @return {}
     */
    virtual RetState SetCallback(const std::string &type, QrCallbackFunc func)
    {
        (void)type;
        (void)func;
        return RetState::noSupport;
    }
};
