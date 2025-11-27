
#include "deviceParam.hpp"

#include "devArmParam.hpp"
#include "devHumanParam.hpp"
#include "devQrParam.hpp"

const CfgDev_base& GetDevParam()
{
    if (GetDevQrParam().model != DevModel::qr::no) {
        return static_cast<const CfgDev_base&>(GetDevQrParam());
    }

    if (GetDevArmParam().model != DevModel::arm::no) {
        return static_cast<const CfgDev_base&>(GetDevArmParam());
    }

    if (GetDevHumanParam().model != DevModel::human::no) {
        return static_cast<const CfgDev_base&>(GetDevHumanParam());
    }
    // toml文件中的参数无法匹配程序中的配置，退出
    LOG_CRITICAL("device config can not match");
    exit(1);
}