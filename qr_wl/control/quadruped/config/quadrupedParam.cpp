
#include "quadrupedParam.hpp"

#include "baseline.hpp"
#include "checker.hpp"
#include "quadruped/config_base.hpp"
#include "quadruped/config_gazebo.hpp"
#include "quadruped/config_gazebo_w.hpp"
#include "quadruped/config_linkV2_3.hpp"
#include "quadruped/config_linkV2_3_w.hpp"
#include "quadruped/config_middle.hpp"
#include "quadruped/config_middleV3.hpp"
#include "quadruped/config_middleV4.hpp"

using namespace std;

void QuadrupedParam::InitAfter()
{
    // 为了使用MIT的卡尔曼滤波
    pCom.setZero();  // 注意：几何中心发生偏移，计算方式需要更改
    // pRollx
    pRoll(0, 0) = pRoll(0, 1) = pCom(0, 0) + BODY_Lx * 0.5;
    pRoll(0, 2) = pRoll(0, 3) = pCom(0, 0) - BODY_Lx * 0.5;
    // pRolly
    pRoll(1, 0) = pRoll(1, 2) = pCom(1, 0) - BODY_Ly * 0.5;
    pRoll(1, 1) = pRoll(1, 3) = pCom(1, 0) + BODY_Ly * 0.5;
    // pRollz
    pRoll.row(2).fill(0.0);
    // pHip
    pHip = pRoll;
    pHip(1, 0) = pRoll(1, 0) - LEG_L0;
    pHip(1, 1) = pRoll(1, 1) + LEG_L0;
    pHip(1, 2) = pRoll(1, 2) - LEG_L0;
    pHip(1, 3) = pRoll(1, 3) + LEG_L0;
    // pFoot
    pFoot = pHip;
    pFoot.row(2) << -bodyHeight, -bodyHeight, -bodyHeight, -bodyHeight;

    // 覆盖qInit
    CppSqlite sql;
    {
        auto data = sql.Read("quadruped", "qInit");
        if (data.first) {
            std::istringstream iss(data.second);
            double number;
            vector<double> tmpData;
            while (iss >> number) {
                tmpData.push_back(number);
            }

            // cout << "old qInit:" << qInit << endl;
            auto dbData = VecToMat34(tmpData);
            if (dbData) {
                qInit = dbData.value();
            }
            // cout << "new qInit:" << qInit << endl;
            LOG_INFO("[qrParam] a new qInit exists, the old qInit has been replaced");
        }
    }

    {
        // 从数据库获取配置的loadMass
        auto [ret, data] = sql.Read("quadruped", "loadMass");
        if (ret) {
            try {
                loadMass = std::stod(data);
                LOG_INFO("Set loadMass:{} kg", loadMass);
            } catch (const std::exception& e) {
                LOG_INFO("sql loadMass exception:{}", e.what());
            }
        } else {
            sql.SyncWrite("quadruped", "loadMass", "double", std::to_string(0));
        }
    }
}

QuadrupedParam::QuadrupedParam() { ReInit(); }

/**
 * @description: 重新刷新一次参数，可用于测试用例中
 * @return {}
 */
void QuadrupedParam::ReInit()
{
    GetConfig();
    InitAfter();
}

/**
 * @description: 获取四足算法模型的型号，根据配置文件映射而来
 * @return {}
 */
void QuadrupedParam::GetConfig()
{
    auto qrModel = GetRobotConfigDirect<string>("ctrModel");
    if (qrModel.has_value() == false) {
        // 未查找到model字符串，退出
        LOG_CRITICAL("can not find model keyword");
        exit(1);
    }

    switch (BKDRHash(qrModel.value().c_str())) {
        case "qr"_hash: {
            Config_base para;
            Load(para);
        } break;

        case "qr-middle"_hash: {
            Config_middle para;
            Load(para);
        } break;

        case "qr-middleV3"_hash: {
            Config_middleV3 para;
            Load(para);
        } break;

        case "qr-middleV4"_hash: {
            Config_middleV4 para;
            Load(para);
        } break;

        case "qr-gazebo"_hash: {
            Config_gazebo para;
            Load(para);
        } break;

        case "qr-linkV2-3"_hash: {
            Config_linkV2_3 para;
            Load(para);
        } break;

        case "qr-gazebo-w"_hash: {
            Config_gazebo_w para;
            Load(para);
        } break;

        case "qr-linkV2-3-w"_hash: {
            Config_linkV2_3_w para;
            Load(para);
        } break;
        default: {
            // toml文件中的参数无法匹配程序中的配置，退出
            LOG_CRITICAL("quadruped config can not match: {}", qrModel.value());
            exit(1);
        }
    }
}