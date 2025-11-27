
#include "armDevEcat.hpp"

#include <soem/ethercat.h>

#include "baseline.hpp"

using namespace std;

char IOmap[4096];
const std::set<std::string> IO_NAME{"panel", "tool", "extra"};

ArmDevEcat::ArmDevEcat(const std::string& ethName)
{
    if (ec_init(ethName.c_str()) > 0) {
        int wkc = ec_config_init(FALSE);
        if (wkc == 0) {
            LOG_ERROR("ethercat init failed: {}", ethName);
            return;
        }
    } else {
        LOG_ERROR("ethercat init failed: {}", ethName);
        return;
    }

    for (int slc = 1; slc <= ec_slavecount; slc++) {
        // name是char[]类型，必须先转换为string
        if (string(ec_slave[slc].name) == "D1616BP") {
            LOG_INFO("found D1616BP at {}", ec_slave[slc].configadr);

            devs_.push_back(EcatIO(slc));
        } else if (string(ec_slave[slc].name) == "ZeroErr Driver") {
            LOG_INFO("found ZeroErr Driver at {}", slc);
            devs_.push_back(EcatMotor(slc));
        }
    }
    ToPreOperational();
    PreOpInit();
    Init();
    // 注册防止第一个数据收不到
    MsgTryRecv<bool>("arm::enable", this);
    MsgTryRecv<std::pair<std::string, std::bitset<32>>>("arm::set_io", this);
}

/**
 * @description: 调用各个电机的PDO配置
 * @return {}
 */
void ArmDevEcat::PreOpInit()
{
    for (auto& dev : devs_) {
        {
            auto ptr = std::get_if<EcatMotor>(&dev);
            if (ptr) {
                ptr->PreOpInit();
                continue;
            }
        }
        {
            auto ptr = std::get_if<EcatIO>(&dev);
            if (ptr) {
                ptr->PreOpInit();
                continue;
            }
        }
    }
}

/**
 * @description:
 * @return {}
 */
void ArmDevEcat::OpInit()
{
    for (auto& dev : devs_) {
        {
            auto ptr = std::get_if<EcatMotor>(&dev);
            if (ptr) {
                ptr->OpInit();
                continue;
            }
        }
        {
            auto ptr = std::get_if<EcatIO>(&dev);
            if (ptr) {
                ptr->OpInit();
                continue;
            }
        }
    }
}

void ArmDevEcat::PdoSync()
{
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
}

/**
 * @description: 进入operational状态，程序启动后进去safe-operational状态，此时进行相关配置后，再进入本状态才能工作
 * @return {}
 */
void ArmDevEcat::ToOperational()
{
    // 进入operational状态
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);

    // 等待执行
    int chk = 200;
    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
    LOG_INFO("slaves mapped, state to OPERATIONAL");

    // 最后检查是否进入
    ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 4);
    if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
        LOG_ERROR("Not all slaves reached operational state.");
        ec_readstate();
        for (auto i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                LOG_ERROR("slave {} State={}, statuCode={} : {}", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    }
}

void ArmDevEcat::ToPreOperational()
{
    // 进入operational状态
    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);

    // 等待执行
    int chk = 200;
    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_PRE_OP, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_PRE_OP));
    LOG_INFO("slaves mapped, state to PRE-OPERATIONAL");

    // 最后检查是否进入
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE * 4);
    if (ec_slave[0].state != EC_STATE_PRE_OP) {
        LOG_ERROR("Not all slaves reached pre-operational state.");
        ec_readstate();
        for (auto i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_PRE_OP) {
                LOG_ERROR("slave {} State={}, statuCode={} : {}", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    }
}

void ArmDevEcat::ToSafeOperational()
{
    // 进入operational状态
    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);

    // 等待执行
    int chk = 200;
    do {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_SAFE_OP, 50000);
    } while (chk-- && (ec_slave[0].state != EC_STATE_SAFE_OP));
    LOG_INFO("slaves mapped, state to safe-OPERATIONAL");

    // 最后检查是否进入
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    if (ec_slave[0].state != EC_STATE_SAFE_OP) {
        LOG_ERROR("Not all slaves reached safe-operational state.");
        ec_readstate();
        for (auto i = 1; i <= ec_slavecount; i++) {
            if (ec_slave[i].state != EC_STATE_SAFE_OP) {
                LOG_ERROR("slave {} State={}, statuCode={} : {}", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    }
}

/**
 * @description: 初始化，因为PDO等一些配置需要在op状态前完成，因此分开初始化
 * 也可以设置pre-op到safe-op的钩子函数进行pdo配置，即PO2SOconfig
 * @return {}
 */
void ArmDevEcat::Init()
{
    int ret = ec_config_map(&IOmap);
    if (ret == 0) {
        LOG_ERROR("[ecat] map err");
    }
    if (ret) {
        ec_configdc();
        ToSafeOperational();
        ToOperational();
        OpInit();
    }
}

void ArmDevEcat::Run()
{
    UpdateBasicData();
    UpdateStatusWord();
    UpdateIoInState();
    RxArmCmd();

    SendMotorCmd();
    RecvMotorRet();
    PdoSync();
}

void ArmDevEcat::Enable()
{
    if (motorInfos_.state != msg::arm_motor_state::enable) {
        // 接收值放入发送值，阻止速度超限
        for (int i = 0; i < ARM_MOTOR_SIZE; i++) {
            motorCmd_.motor[i].alpha = motorRet_.motor[i].alpha;
            motorCmd_.motor[i].torq = motorRet_.motor[i].torq;
            motorCmd_.motor[i].blta = motorRet_.motor[i].blta;
        }
        asyncFunc_ = std::async([this]() { this->AsyncEnable(); });
    }
}

void ArmDevEcat::AsyncEnable()
{
    for (auto& dev : devs_) {
        auto ptr = std::get_if<EcatMotor>(&dev);
        if (ptr) {
            // 接收值放入发送值，阻止速度超限
            ptr->SetPosition(ptr->GetPosition());
            ptr->SetCtrlWord(0x80);
        }
    }
    TimerTools::SleepForMs(200);

    for (auto& dev : devs_) {
        auto ptr = std::get_if<EcatMotor>(&dev);
        if (ptr) {
            ptr->SetCtrlWord(0x06);
        }
    }
    TimerTools::SleepForMs(200);

    for (auto& dev : devs_) {
        auto ptr = std::get_if<EcatMotor>(&dev);
        if (ptr) {
            ptr->SetCtrlWord(0x07);
        }
    }
    TimerTools::SleepForMs(200);

    for (auto& dev : devs_) {
        auto ptr = std::get_if<EcatMotor>(&dev);
        if (ptr) {
            ptr->SetCtrlWord(0x0F);
        }
    }
    TimerTools::SleepForMs(200);
    LOG_INFO("armMotor enable complete");
}

void ArmDevEcat::Disable()
{
    asyncFunc_ = std::async([this]() { this->AsyncDisable(); });
}

void ArmDevEcat::AsyncDisable()
{
    for (auto& dev : devs_) {
        auto ptr = std::get_if<EcatMotor>(&dev);
        if (ptr) {
            ptr->SetCtrlWord(0x06);
        }
    }
    TimerTools::SleepForMs(200);
    LOG_INFO("armMotor disable complete");
}

void ArmDevEcat::Start()
{
    task_ = make_unique<PeriodicMemberFunction<ArmDevEcat>>("armMotor", 0.004, this, &ArmDevEcat::Run, true);
    task_->Start();
}

void ArmDevEcat::Stop()
{
    Disable();
    task_->Stop();
}
void ArmDevEcat::SendMotorCmd()
{
    if (motorInfos_.state != msg::arm_motor_state::enable) {
        return;
    }

    int cnt = 0;
    for (auto& dev : devs_) {
        auto ptr = std::get_if<EcatMotor>(&dev);
        if (ptr) {
            ptr->SetPosition(motorCmd_.motor[cnt].alpha);
            cnt++;
        }
    }

    MsgTrySend("arm::arm_cmd", motorCmd_);  // 上报给qrChart
}

void ArmDevEcat::RecvMotorRet()
{
    int cnt = 0;
    for (auto& dev : devs_) {
        auto ptr = std::get_if<EcatMotor>(&dev);
        if (ptr) {
            motorRet_.motor[cnt].alpha = ptr->GetPosition();
            motorRet_.motor[cnt].torq = ptr->GetVelocity();
            motorRet_.motor[cnt].blta = ptr->GetTorque();
            cnt++;
        }
    }

    MsgTrySend("arm::arm_data", motorRet_);
}

/**
 * @description: 更新状态字
 * @return {}
 */
void ArmDevEcat::UpdateStatusWord()
{
    std::array<u16, ARM_MOTOR_SIZE> errors;
    auto state = msg::arm_motor_state::enable;  // 默认为全使能
    int cnt = 0;
    for (auto& dev : devs_) {
        auto ptr = std::get_if<EcatMotor>(&dev);
        if (ptr) {
            auto word = ptr->GetStatusWord();
            // printf("word: %x\n", word);
            if ((word & 0x07) != 0x07) {
                state = msg::arm_motor_state::disable;
            }

            if ((word & 0x08) == 0x08) {
                errors[cnt] = ptr->GetErrorCode();
                state = msg::arm_motor_state::error;
            } else {
                errors[cnt] = 0;
            }

            cnt++;
        }
    }

    lock_guard lock(mtx_);
    if (state != motorInfos_.state) {
        motorInfos_.state = state;
        infoIsUpdate_ = true;
    }

    if (errors != motorInfos_.errCode) {
        motorInfos_.errCode = errors;
        infoIsUpdate_ = true;
    }
}

/**
 * @description: 刷新IO状态
 * @return {}
 */
void ArmDevEcat::UpdateIoInState()
{
    {
        auto io = GetIoIn("tool");
        if (io != motorInfos_.ioInSta["tool"]) {
            motorInfos_.ioInSta["tool"] = io;
            infoIsUpdate_ = true;
        }
    }

    {
        auto io = GetIoIn("panel");
        if (io != motorInfos_.ioInSta["panel"]) {
            motorInfos_.ioInSta["panel"] = io;
            infoIsUpdate_ = true;
        }
    }
}

void ArmDevEcat::SetIoOut(const std::string& name, std::bitset<32> set)
{
    u16 io = static_cast<u16>(set.to_ulong());
    if (name == "panel") {
        for (auto& dev : devs_) {
            auto ptr = std::get_if<EcatIO>(&dev);
            if (ptr) {
                ptr->SetIoOut(io);
                // IO模块和电机不支持读取输出IO(相关对象字典不可读或者读取值恒为0)，因此ioOut的读取值是假的
                motorInfos_.ioOutSta["panel"] = io;
            }
        }
    } else if (name == "tool") {
        for (auto& dev : devs_) {
            auto ptr = std::get_if<EcatMotor>(&dev);
            if (ptr) {
                ptr->SetIoOut(io);
                motorInfos_.ioOutSta["tool"] = io;
            }
        }
    } else if (name == "extra") {
        motorInfos_.ioOutSta["extra"] = io;
    }
    infoIsUpdate_ = true;
}

/**
 * @description: 非真实值，见SetIoOut
 * @param name
 * @return {}
 */
std::bitset<32> ArmDevEcat::GetIoOut(const std::string& name)
{
    if (IO_NAME.count(name) == 0) {
        return 0;
    }
    return motorInfos_.ioOutSta[name];
}

std::bitset<32> ArmDevEcat::GetIoIn(const std::string& name)
{
    if (name == "tool") {
        // 只读取最后一个电机IO
        for (int i = devs_.size() - 1; i >= 0; i--) {
            if (auto ptr = std::get_if<EcatMotor>(&devs_[i])) {
                return ptr->GetIoIn();
            }
        }
    } else if (name == "panel") {
        for (auto& dev : devs_) {
            if (auto ptr = std::get_if<EcatIO>(&dev)) {
                return ptr->GetIoIn();  // 只有一个io模块，可直接return
            }
        }
    }
    return 0;
}

void ArmDevEcat::RxArmCmd()
{
    if (auto ret = MsgTryRecv<bool>("arm::enable", this)) {
        if (ret.value() == true) {
            Enable();
        } else {
            Disable();
        }
    }

    if (auto ret = MsgTryRecv<std::pair<std::string, std::bitset<32>>>("arm::set_io", this)) {
        SetIoOut(ret.value().first, ret.value().second);
    }
}