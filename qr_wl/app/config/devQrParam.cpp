

#include "devQrParam.hpp"

#include "baseline.hpp"
#include "device/qr/config_linkV2_3.hpp"
#include "device/qr/config_linkV2_3_w.hpp"
#include "device/qr/config_middle.hpp"
#include "device/qr/config_middleV3.hpp"
#include "device/qr/config_middleV4.hpp"

using namespace std;

DevQrParam::DevQrParam() { ReInit(); }

/**
 * @description: 重新读取一次配置，目前只在测试用例中有用，因为每个TEST并不会重置单例
 * @return {}
 */
void DevQrParam::ReInit() { GetConfig(); }

/**
 * @description: 从配置文件获取配置名字，并载入具体配置
 * @return {}
 */
void DevQrParam::GetConfig()
{
    auto devModel = GetRobotConfigDirect<string>("devModel");
    if (devModel.has_value() == false) {
        // 未查找到model字符串，退出
        LOG_CRITICAL("can not find model keyword");
        exit(1);
    }

    switch (BKDRHash(devModel.value().c_str())) {
        case "qr"_hash: {
            model = DevModel::qr::base;
            CfgDev_qr para;
            Load(para);
        } break;

        case "qr-middle"_hash: {
            model = DevModel::qr::middle;
            CfgDev_middle para;
            Load(para);
        } break;

        case "qr-middleV3"_hash: {
            model = DevModel::qr::middleV3;
            CfgDev_middleV3 para;
            Load(para);
        } break;

        case "qr-middleV4"_hash: {
            model = DevModel::qr::middleV4;
            CfgDev_middleV4 para;
            Load(para);
        } break;

        case "qr-linkV2-3"_hash: {
            model = DevModel::qr::linkV2_3;
            CfgDev_linkV2_3 para;
            Load(para);
        } break;

        case "qr-linkV2-3-w"_hash: {
            model = DevModel::qr::linkV2_3_w;
            CfgDev_linkV2_3_w para;
            Load(para);
        } break;

        default:
            break;
    }
}