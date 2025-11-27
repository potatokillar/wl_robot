
#include "userQuadNode.hpp"

#include "deviceCustomParam.hpp"
#include "deviceParam.hpp"
#include "legWheelMotorMit.hpp"
#include "legWheelMotorMitV3.hpp"
#include "qrMotorGazebo.hpp"
#include "qrMotorMit.hpp"
#include "qrMotorMitV3.hpp"
#include "robotMedia.hpp"
#include "robotState.hpp"
#include "spi2can.hpp"

using namespace std;

UserQuadNode::UserQuadNode()
{
    std::shared_ptr<ApiQuadruped> quadApi;
    if (GetDevQrParam().qrMotorCfg.used == true)
    {
        if (bootArgs.isSim)
        {
            motor_ = GetSimMotor();
        }
        else
        {
            // motor_ = GetRealMotor();
            motor_ = GetRealMotorV3();
        }
    }
    else if (GetDevQrParam().legWheelMotorCfg.used == true)
    {
        if (bootArgs.isSim)
        {
            motor_ = GetSimLegWheelMotor();
        }
        else
        {
            motor_ = GetRealLegWheelMotor();
        }
    }

    motor_->Start();
    auto devApi = make_shared<ApiDevice>();

    quadApi = make_shared<ApiQuadruped>();
    sdk_ = make_unique<QuadSdk>(quadApi, devApi);
    key_ = make_unique<QuadKey>(quadApi, devApi);
    cmd_ = make_unique<QuadCmd>(quadApi, devApi);
    dataChange_ = make_unique<QuadDataChange>(quadApi, devApi);
    //  web_ = make_unique<QuadWeb>(quadApi, devApi);

    thread_ = std::thread(&UserQuadNode::Loop, this);

    // 自动启动算法功能（如果配置开启）
    auto autoStart = GetRobotConfigDirect<bool>("autoStartAlgorithm");
    if (autoStart.has_value() && autoStart.value())
    {
        // 获取启动延时配置（默认2秒）
        auto delay = GetRobotConfigDirect<int>("autoStartDelay").value_or(2000);

        LOG_INFO("Auto-start algorithm enabled, will start after %d ms delay", delay);

        // 创建异步启动线程，避免阻塞构造函数
        std::thread([quadApi, delay]()
                    {
            // 等待系统初始化完成
            TimerTools::SleepForMs(delay);

            // 检查当前状态，只在standby状态下启动
            if (GetRobotCurState() == RobotState::standby) {
                LOG_INFO("Auto-starting quadruped algorithm...");
                quadApi->Start(QuadBootMode::qr);
                LOG_INFO("Quadruped algorithm auto-started successfully");
            } else {
                LOG_WARN("Cannot auto-start algorithm, robot not in standby state (current state: %d)",
                         static_cast<int>(GetRobotCurState()));
            } })
            .detach();
    }
}
UserQuadNode::~UserQuadNode() { motor_->Stop(); }

void UserQuadNode::Loop()
{
    while (1)
    {
        key_->Run();
        sdk_->Run();
        cmd_->Run();
        dataChange_->Run();

        // 还是得控制频率，阻止实际用户接口的疯狂调用
        TimerTools::SleepForMs(5);
    }
}

/**
 * @description: 获取真实电机
 * @return {}
 */
std::shared_ptr<QrMotor> UserQuadNode::GetRealMotor()
{
    auto canDev = make_unique<MitCanInterface>(GetDevParam().spiName, GetDevParam().spiSpeed);

    QrMotorCfg cfg;
    cfg.boot = bootArgs;
    cfg.chiral = GetDevQrParam().qrMotorCfg.chiral;
    cfg.motorModel = GetDevQrParam().qrMotorCfg.motorModel;
    cfg.ratio = GetDevQrParam().qrMotorCfg.ratio;
    auto [ret, zeroPos] = GetDevCustomParam().ReadZeroPos();
    if (ret)
    {
        cfg.zeroPos = zeroPos;
    }
    // todo, 关节控制的策略略有不同，不过等完成非关节级的代码后再测试
    auto motor = make_shared<QrMotorMit>(cfg, std::move(canDev));

    {
        //  若需要写zeroPos，则请求写入
        auto [ret, init] = GetDevCustomParam().ReadInitFlag();
        if (ret)
        {
            if (init)
            {
                motor->SetParam("req_zeropos_write", true);
                motor->SetCallback("joint_write_complete", [this](const std::any &data)
                                   { return this->ZeroPosWriteComplete(data); });
            }
        }
    }

    motor->SetCallback("joint_error", [this](const std::any &data)
                       { return this->JointErr(data); });

    return motor;
}

/**
 * @description: 适配spi2canV3版本的四足式机器人
 * @return {}
 */
std::shared_ptr<QrMotor> UserQuadNode::GetRealMotorV3()
{
    std::array<size_t, 4> canNum{3, 3, 3, 3};
    auto canDev = make_unique<SpiToCanV3>(GetDevParam().spiName, GetDevParam().spiSpeed, canNum);

    QrMotorCfgV3 cfg;
    cfg.boot = bootArgs;
    for (int leg = 0; leg < 4; leg++)
    {
        for (int motor = 0; motor < 3; motor++)
        {
            cfg.motorCfg[leg][motor].model = GetDevQrParam().qrMotorCfg.motorModel(motor, leg);
            cfg.motorCfg[leg][motor].ratio = GetDevQrParam().qrMotorCfg.ratio(motor, leg);
            cfg.motorCfg[leg][motor].chiral = GetDevQrParam().qrMotorCfg.chiral(motor, leg);
        }
    }
    auto motor = make_shared<QrMotorMitV3>(cfg, std::move(canDev));
    return motor;
}

/**
 * @description: 获取仿真电机
 * @return {}
 */
std::shared_ptr<QrMotor> UserQuadNode::GetSimMotor() { return make_shared<QrMotorGazebo>(); }

/**
 * @description: 电机写零位完成后，会上报新的零位，这里自行写入数据库
 * @param data Mat34<double>类型
 * @return {}
 */
std::any UserQuadNode::ZeroPosWriteComplete(const std::any &anyData)
{
    Mat34<double> pos = std::any_cast<Mat34<double>>(anyData);
    GetDevCustomParam().WriteZeroPos(pos);
    GetDevCustomParam().WriteInitFlag(0);
    LOG_INFO("save zeropos to database");

    return std::any{};
}

/**
 * @description: 电机错误回调
 * @param anyData
 * @return {}
 */
std::any UserQuadNode::JointErr(const std::any &anyData)
{
    (void)anyData;
    AddMediaPackage(MediaTaskPackage("jointError"));

    return std::any{};
}

/**
 * @description: 设置回调，目前只用于ROS的发送接收
 * @param type
 * @param func
 * @return {}
 */
RetState UserQuadNode::SetCallback(const std::string &type, QrCallbackFunc func)
{
    // 仅仿真支持
    if (bootArgs.isSim)
    {
        return motor_->SetCallback(type, func);
    }
    return RetState::noSupport;
}

/**
 * @description: 获取真实电机-轮腿式
 * @return {}
 */
std::shared_ptr<QrMotor> UserQuadNode::GetRealLegWheelMotor()
{
    std::array<size_t, 4> canNum{4, 4, 4, 4};
    auto canDev = make_unique<SpiToCanV3>(GetDevParam().spiName, GetDevParam().spiSpeed, canNum);

    LegWheelMotorCfg cfg;
    cfg.boot = bootArgs;
    for (int leg = 0; leg < 4; leg++)
    {
        for (int motor = 0; motor < 4; motor++)
        {
            cfg.motorCfg[leg][motor].model = GetDevQrParam().legWheelMotorCfg.motorModel(motor, leg);
            cfg.motorCfg[leg][motor].ratio = GetDevQrParam().legWheelMotorCfg.ratio(motor, leg);
            cfg.motorCfg[leg][motor].chiral = GetDevQrParam().legWheelMotorCfg.chiral(motor, leg);
        }
    }
    auto motor = make_shared<LegWheelMotorMitV3>(cfg, std::move(canDev));
    return motor;
}

/**
 * @description: 获取仿真电机
 * @return {}
 */
std::shared_ptr<QrMotor> UserQuadNode::GetSimLegWheelMotor() { return make_shared<QrMotorGazebo>(); }
