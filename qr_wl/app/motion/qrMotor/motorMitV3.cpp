
#include "motorMitV3.hpp"

#include "deviceCustomParam.hpp"

using namespace std;

MotorMitV3::MotorMitV3(int leg_num, int motor_num, std::unique_ptr<SpiToCanV3> can) : LEG_NUM(leg_num), MOTOR_NUM(motor_num), spi2can(std::move(can))
{
    infos_.resize(LEG_NUM);
    for (int leg = 0; leg < LEG_NUM; leg++) {
        infos_[leg].resize(MOTOR_NUM);
    }
}

/**
 * @description: 检查电机状态，根据CAN的发送接收数据判断
 * @param 数据解析结果
 * @return {}
 */
void MotorMitV3::CheckMotorState(const std::vector<std::vector<bool>>& ret)
{
    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            auto& info = infos_[leg][motor];

            // 只要CAN数据解析成功就更新时间戳
            if (ret[leg][motor]) {
                info.lastRecv = TimerTools::GetNowTickMs();
            }

            // 检查未收到数据
            auto nowTime = TimerTools::GetNowTickMs();
            if (auto dura = nowTime - info.lastRecv; dura > 300) {
                if (info.state != MotorState::timeout) {
                    LOG_ERROR("motor:{}, comm lose, dura:{}", info.name, dura);
                    info.state = MotorState::timeout;
                }
            }
        }
    }
}

/**
 * @description: 调用SPI发送命令
 * @return {}
 */
void MotorMitV3::SpiTransfer(const std::array<std::vector<CanData>, 4>& can)
{
    spi2can->AddMessage(0, can[0], can[1]);
    spi2can->AddMessage(1, can[2], can[3]);
    spi2can->Transfer();
}

/**
 * @description: 从数据库读写电机上电初始化位置
 * @param name
 * @param description
 * @return {}
 */
std::optional<vector<std::vector<double>>> MotorMitV3::GetInitPosFromDatabase(const std::string& tag)
{
    auto dbRet = GetDevCustomParam().ReadMotorInitPos(tag);

    if (dbRet.has_value()) {
        vector<std::vector<double>> poss(LEG_NUM);
        for (int leg = 0; leg < LEG_NUM; leg++) {
            for (int motor = 0; motor < MOTOR_NUM; motor++) {
                poss[leg].push_back(dbRet.value()[leg * MOTOR_NUM + motor]);
            }
        }
        return poss;
    }

    return std::nullopt;
}
void MotorMitV3::SaveInitPos2Database(const std::string& tag, const vector<std::vector<double>>& data)
{
    // 数据库中的重写请求不存在或为0，则退出
    auto [ret, init] = GetDevCustomParam().ReadInitFlag();
    if ((ret == false) || (init == 0)) {
        LOG_INFO("SaveZeroPos2Database: init flag is 0 or noexist, exit");
        return;
    }

    vector<double> poss;
    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            // 如果存在0值，表示可能有电机使能失败，不写入数据库
            if (data[leg][motor] == 0) {
                return;
            }
            poss.push_back(data[leg][motor]);
        }
    }

    GetDevCustomParam().WriteMotorInitPos(tag, poss);
    GetDevCustomParam().WriteInitFlag(0);
    LOG_INFO("save motor init to database");
}

/**
 * @description: 清除SPI发送接收缓存中的数据
 * @return {}
 */
void MotorMitV3::ClearSpiData() { spi2can->ClearBuf(); }

/**
 * @description: 更新bias
 * @return {}
 */
void MotorMitV3::CheckInit(const std::string& dbTag, const std::vector<std::vector<double>>& nowPos)
{
    if (biasIsUpdate_ == true) {
        return;
    }

    SaveInitPos2Database(dbTag, nowPos);

    bool success = true;
    constexpr double PI = 3.1415926;
    for (int leg = 0; leg < LEG_NUM; leg++) {
        for (int motor = 0; motor < MOTOR_NUM; motor++) {
            auto& one_motor_info = infos_[leg][motor];
            auto& one_motor_data = nowPos[leg][motor];
            if (one_motor_data == 0.0) {
                one_motor_info.qBias = 0.0;
                success = false;
            } else {
                one_motor_info.qBias = one_motor_data;
            }
        }
    }
    biasIsUpdate_ = success;
}

void MotorMitV3::SendEnable()
{
    array<vector<CanData>, 4> canMsg;
    for (int i = 0; i < LEG_NUM; i++) {
        canMsg[i].resize(MOTOR_NUM);
        for (int j = 0; j < MOTOR_NUM; j++) {
            canMsg[i][j].id = j + 1;
            canMsg[i][j].data[7] = 0xFC;
        }
    }

    SpiTransfer(canMsg);
}
void MotorMitV3::SendDisable()
{
    array<vector<CanData>, 4> canMsg;
    for (int i = 0; i < LEG_NUM; i++) {
        canMsg[i].resize(MOTOR_NUM);
        for (int j = 0; j < MOTOR_NUM; j++) {
            canMsg[i][j].id = j + 1;
            canMsg[i][j].data[7] = 0xFD;
        }
    }

    SpiTransfer(canMsg);
}
void MotorMitV3::SendZero()
{
    array<vector<CanData>, 4> canMsg;
    for (int i = 0; i < LEG_NUM; i++) {
        canMsg[i].resize(MOTOR_NUM);
        for (int j = 0; j < MOTOR_NUM; j++) {
            canMsg[i][j].id = j + 1;
            canMsg[i][j].data[7] = 0xFE;
        }
    }

    SpiTransfer(canMsg);
}