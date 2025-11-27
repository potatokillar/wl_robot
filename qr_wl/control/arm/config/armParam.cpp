
#include "armParam.hpp"

#include "arm/config_arm.hpp"
#include "arm/config_base.hpp"
#include "arm/config_ga701.hpp"
#include "arm/config_iris.hpp"
#include "arm/config_qa01.hpp"
#include "baseline.hpp"

using namespace std;

ArmParam::ArmParam() { ReInit(); }

/**
 * @description: 重新刷新一次参数，可用于测试用例中
 * @return {}
 */
void ArmParam::ReInit() { GetConfig(); }

/**
 * @description: 获取四足算法模型的型号，根据配置文件映射而来
 * @return {}
 */
void ArmParam::GetConfig()
{
    auto armModel = GetRobotConfigDirect<string>("ctrModel");
    if (armModel.has_value() == false) {
        // 未查找到model字符串，退出
        LOG_CRITICAL("can not find model keyword");
        exit(1);
    }

    switch (BKDRHash(armModel.value().c_str())) {
        case "arm"_hash: {
            CfgArm_arm para;
            Load(para);
        } break;
        case "arm-iris"_hash: {
            CfgArm_iris para;
            Load(para);
        } break;
        case "arm-qa01"_hash: {
            CfgArm_qa01 para;
            Load(para);
        } break;
        case "arm-ga701"_hash: {
            CfgArm_ga701 para;
            Load(para);
        } break;
        default: {
            // toml文件中的参数无法匹配程序中的配置，退出
            LOG_CRITICAL("arm config can not match: {}", armModel.value());
            exit(1);
        }
    }

    if (armModel.value().empty() == false) {
        LOG_INFO("arm model : {}", armModel.value());
    }
}