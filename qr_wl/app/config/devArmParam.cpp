
#include "devArmParam.hpp"

#include "baseline.hpp"
#include "device/arm/config_arm.hpp"
#include "device/arm/config_arm_ga701.hpp"
#include "device/arm/config_arm_iris.hpp"
#include "device/arm/config_arm_mit.hpp"
#include "device/arm/config_arm_qa01.hpp"

using namespace std;

DevArmParam::DevArmParam() { ReInit(); }

void DevArmParam::ReInit() { GetConfig(); }

void DevArmParam::GetConfig()
{
    auto devModel = GetRobotConfigDirect<string>("devModel");
    if (devModel.has_value() == false) {
        // 未查找到model字符串，退出
        LOG_CRITICAL("can not find model keyword");
        exit(1);
    }

    switch (BKDRHash(devModel.value().c_str())) {
        case "arm"_hash: {
            model = DevModel::arm::base;
            CfgDev_arm para;
            Load(para);
        } break;

        case "arm-iris"_hash: {
            model = DevModel::arm::iris;
            CfgDev_arm_iris para;
            Load(para);
        } break;

        case "arm-mit"_hash: {
            model = DevModel::arm::mit;
            CfgDev_arm_mit para;
            Load(para);
        } break;

        case "arm-qa01"_hash: {
            model = DevModel::arm::qa01;
            CfgDev_arm_qa01 para;
            Load(para);
        } break;

        case "arm-ga701"_hash: {
            model = DevModel::arm::ga701;
            CfgDev_arm_ga701 para;
            Load(para);
        } break;
        default:
            break;
    }
    // LOG_INFO("device config : {}", devModel.value());
}