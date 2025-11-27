

#pragma once

#include <map>
#include <string>
#include <vector>

#include "baseline.hpp"
#include "device/qr/config_qr.hpp"
#include "deviceType.hpp"

class DevQrParam : public CfgDev_qr
{
public:
    static DevQrParam& GetInstance()
    {
        static DevQrParam instance;
        return instance;
    }
    DevModel::qr model;

private:
    DevQrParam();
    void ReInit();
    void GetConfig();
};

const inline DevQrParam& GetDevQrParam() { return DevQrParam::GetInstance(); }