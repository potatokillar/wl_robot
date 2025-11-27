
#include "devHumanParam.hpp"

#include "baseline.hpp"
#include "device/human/config_human.hpp"
#include "device/human/config_human_mit.hpp"

using namespace std;

DevHumanParam::DevHumanParam() { ReInit(); }

/**
 * @description: 重新读取一次配置，目前只在测试用例中有用，因为每个TEST并不会重置单例
 * @return {}
 */
void DevHumanParam::ReInit() { GetConfig(); }

/**
 * @description: 从配置文件获取配置名字，并载入具体配置
 * @return {}
 */
void DevHumanParam::GetConfig()
{
    auto devModel = GetRobotConfigDirect<string>("devModel");
    if (devModel.has_value() == false) {
        // 未查找到model字符串，退出
        LOG_CRITICAL("can not find model keyword");
        exit(1);
    }

    switch (BKDRHash(devModel.value().c_str())) {
        case "human"_hash: {
            model = DevModel::human::base;
            CfgDev_human para;
            Load(para);
        } break;

        case "human-mit"_hash: {
            model = DevModel::human::mit;
            CfgDev_human_mit para;
            Load(para);
        } break;
        default:
            break;
    }
}