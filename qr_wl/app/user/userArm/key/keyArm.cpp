
#include "keyArm.hpp"
using namespace arm;
using namespace std;

KeyArm::KeyArm(std::shared_ptr<ApiArm> api) : api_(api) { thread_ = std::thread(&KeyArm::BlockRun, this); }

/**
 * @description: 阻塞运行处理，自行通过线程处理
 * @return {}
 */
void KeyArm::BlockRun()
{
    while (running_) {
        if (UpdateGamepadKey()) {
            ArmKey();
        }
        TimerTools::SleepForMs(50);
    }
}

/**
 * @description: 非阻塞运行处理，由keyNode调用
 * @return {}
 */
void KeyArm::Run()
{
    if (UpdateGamepadKey()) {
        GripperKey();
        ArmEnableKey();
        SetArmResetKey();
        ChangerateKey();
    }
}

/**
 * @description: 机械臂手柄按键映射
 * @param cmd
 * @return {}
 */
void KeyArm::ArmKey()
{
    // ControlGripperAPI& api = api_->dev_.gripperAPI_;
    /*机载机械臂使用lb+其他、通用机械臂使用rb+其他*/
    Result<u32> timeout{RetState::error, 0};

    if (btn_.at("a_rb").IsTriggerPress()) {  // 准备位置
        vector<double> jointEnd{0.0, 0.8, 0.8, 0, 1.57, 0};
        double speed1 = 0.5;
        double acc1 = 0.2;
        auto movemode = MoveMode::abs;
        // auto speedmode = SpeedMode::prop;
        // jointEnd << 0.0, 0.8, 0.8, 0, 1.57, 0;
        // jointEnd << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        // jointEnd << 0, 0, 0, 0, 0, 0.6;
        // api_->SetArmEnable();
        // TimerTools::SleepForS(10);
        api_->SetMoveJTaskBlock(jointEnd, movemode, speed1, acc1);
        // api_->SetArmDisable();
    }
    if (btn_.at("a_lb").IsTriggerPress()) {
        vector<double> jointEnd{0.0, 0.8, 0.0, 0.8, 0, 1.57, 0};
        double speed1 = 3.14;
        double acc1 = 0.5;
        auto movemode = MoveMode::abs;
        // auto speedmode = SpeedMode::prop;
        // jointEnd << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        // jointEnd << 0, 0, 0, 0, 0, 0.6;
        // api_->SetArmEnable();
        // TimerTools::SleepForS(10);
        api_->SetMoveJTaskBlock(jointEnd, movemode, speed1, acc1);
        // api_->SetArmDisable();
        // // Vec6<double> jointEnd;
        // double speed1 = 3.14;
        // double acc1 = 0.5;
        // auto abs = MoveMode::abs;
        // auto incr = MoveMode::incr;
        // Pose recv = api_->GetNowCart();
        // recv.PrintPose();
        // // auto speedmode = SpeedMode::abs;
        // // jointEnd << 1.0, 0.8, 0.8, 0, 1.57, 0;
        // // api_->SetMoveJTaskBlock(jointEnd, abs, speed1, acc1);
        // Pose pos1 = {0, 0, 0, 0.01, 0, 0};
        // api_->SetMoveLTaskBlock(pos1, incr, speed1, acc1);
    }
    if (btn_.at("b_lb").IsTriggerPress()) {
        vector<double> jointEnd{0, 0.8, 0.8, 0, 1.57, 0};

        double speed1 = 0.5;
        double acc1 = 0.5;
        auto movemode = MoveMode::abs;
        // jointEnd << 0, 0, 0, 0.2, 0, 0;
        Pose pos1{0.4, 0, 0.1, 3.1415, 0.1212, 0.0};
        // Pose pos2{0.2, -0.1, 0.2, 3.1415, -1.57, 1.57};
        Pose pos2{0.2, 0, 0.1, 3.1415, 0.1212, 0.0};
        int N = 10;
        for (int i = 0; i < N;) {
            // api_->SetMoveJTask(jointEnd, speed1, acc1, movemode);
            // TimerTools::SleepForS(1);
            api_->SetMoveLTaskBlock(pos1, movemode, speed1, acc1);
            api_->SetMoveLTaskBlock(pos2, movemode, speed1, acc1);

            i++;
        }
        // j1 1.57,0
        // j2 1.57,0
        // j3 0,0
        // j4
        // j5

        //  TimerTools::SleepForS(3);
        //  api_->SetMoveJTask(joint2, speed1, acc1, movemode);
        //  TimerTools::SleepForS(3);
        //  api_->SetMoveLTask(pos2, speed1, Tool, acc1, movemode);
    }
    // if (btn_.at("b_lb").IsTriggerPress()) {  // 面前的桌子上抓取
    //     Pose pos1{0.6, 0.0, 0.3, 3.1415, 0.0, 0.0};
    //     Pose pos2{0.6, 0.0, 0.2, 3.1415, 0.0, 0.0};
    //     Pose pos3{0.3, -0.3, 0.3, 3.141, 0, 0};
    //     Pose pos4{0.3, -0.3, -0.2, 3.141, 0, 0};
    //     Vec6<double> jointEnd;
    //     jointEnd << 0, 0.8, 0.8, 0, 1.57, 0;
    //     double speed1;
    //     auto movemode = MoveMode::ABS;
    //     speed1 = 1;
    //     api_->arm_.SetMoveJTask(jointEnd, speed1, acc1, movemode);
    //     TimerTools::SleepForS(3);
    //     api_->SetMoveLTask(pos1, speed1, Tool, acc1, movemode);
    //     TimerTools::SleepForS(3);
    //     api_->SetMoveLTask(pos2, speed1, Tool, acc1, movemode);
    //     // api.PositionPercentageNew(30);
    //     TimerTools::SleepForS(1);
    //     api_->SetMoveLTask(pos1, speed1, Tool, acc1, movemode);
    //     TimerTools::SleepForS(3);
    //     api_->SetMoveLTask(pos3, speed1, Tool, acc1, movemode);
    //     TimerTools::SleepForS(3);
    //     api_->SetMoveLTask(pos4, speed1, Tool, acc1, movemode);
    //     TimerTools::SleepForS(3);
    //     // api.PositionPercentageNew(99);
    //     TimerTools::SleepForS(3);
    //     api_->SetMoveLTask(pos3, speed1, Tool, acc1, movemode);
    //     TimerTools::SleepForS(3);
    //     // api_->SetMoveJTask(joint1, speed1, acc1, movemode);
    //     api_->SetMoveLTask(pos1, speed1, Tool, acc1, movemode);
    // }
    if (btn_.at("b_rb").IsTriggerPress()) {
        double speed1 = 0.5;
        double acc1 = 0.5;
        MoveMode movemode = MoveMode::abs;
        Pose pos1{0.3, 0.0, 0.2, 3.1415, 0.0, 0.0};
        Pose pos2{0.3, 0.0, 0.1, 3.1415, 0.0, 0.0};
        Pose pos3{0.3, -0.3, 0.2, 3.1415, 0, 0};
        Pose pos4{0.3, -0.3, -0.1, 3.1415, 0, 0};

        api_->SetMoveLTaskBlock(pos1, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos2, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos1, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos3, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos4, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos3, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos1, movemode, speed1, acc1);
    }

    if (btn_.at("y_lb").IsTriggerPress()) {  // 开门
        double speed1 = 1;
        double acc1 = 0.5;
        MoveMode movemode = MoveMode::abs;
        vector<double> joint1{1.57, 1.57, 0, 0, 0, 0};
        vector<double> joint2{0, 0, 0, 0, 0, 0};
        // , joint3;
        // joint1 << ;
        // joint2 << 0, 0, 0, 0, 0, 0;

        api_->SetMoveJTaskBlock(joint1, movemode, speed1, acc1);  //
        api_->SetMoveJTaskBlock(joint2, movemode, speed1, acc1);

        // TimerTools::SleepForS(0.5);
        // Pose pos1{0.4, 0.0, 0.5, 0, -1.541, 3.141};
        // Pose pos2{0.5, 0.0, 0.5, 0, -1.541, 3.141};
        // Pose pos3{0.5, 0.0, 0.5, -1.57, -1.0, -1.615};
        // Pose pos4{0.55, 0.0, 0.5, -1.57, -1.0, -1.615};
        // Pose pos5{0.5, 0.0, 0.5, 0, -1.541, 3.141};
        // Pose pos6{0.7, 0.0, 0.5, 0, -1.541, 3.141};

        // api_->SetMoveJTask(joint1, speed1, acc1, movemode);  //
        // TimerTools::SleepForS(0.5);
        // // api.PositionPercentageNew(99);
        // api_->SetMoveLTask(pos1, speed1, Tool, acc1, movemode);
        // TimerTools::SleepForS(0.5);
        // api_->SetMoveLTask(pos2, speed1, Tool, acc1, movemode);
        // TimerTools::SleepForS(0.5);
        // // api.PositionPercentageNew(30);
        // api_->SetMoveLTask(pos3, speed1, Tool, acc1, movemode);
        // TimerTools::SleepForS(0.5);
        // api_->SetMoveLTask(pos4, speed1, Tool, acc1, movemode);
        // // api.PositionPercentageNew(99);
        // TimerTools::SleepForS(0.5);
        // api_->SetMoveLTask(pos5, speed1, Tool, acc1, movemode);
        // // api.PositionPercentageNew(30);
        // TimerTools::SleepForS(0.5);
        // api_->SetMoveLTask(pos6, speed1, Tool, acc1, movemode);
        // TimerTools::SleepForS(0.5);
        // api_->SetMoveJTask(joint1, speed1, acc1, movemode);
    }

    if (btn_.at("y_rb").IsTriggerPress()) {  // 开门
        double speed1 = 0.5;
        double acc1 = 0.5;
        MoveMode movemode = MoveMode::abs;
        vector<double> joint1{0.0, -0.2, 1.9, 0, -0.3, 0};

        Pose pos1{0.4, 0.0, 0.3, 0, -1.541, 3.141};
        Pose pos2{0.5, 0.0, 0.3, 0, -1.541, 3.141};
        Pose pos3{0.5, 0.0, 0.2, 0, -1.541, 3.141};
        Pose pos4{0.55, 0.0, 0.2, 0, -1.541, 3.141};
        Pose pos5{0.4, 0.0, 0.2, 0, -1.541, 3.141};
        Pose pos6{0.6, 0.0, 0.3, 0, -1.541, 3.141};

        api_->SetMoveJTaskBlock(joint1, movemode, speed1, acc1);  //

        api_->SetMoveLTaskBlock(pos1, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos2, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos3, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos4, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos5, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos6, movemode, speed1, acc1);
        api_->SetMoveJTaskBlock(joint1, movemode, speed1, acc1);
    }

    if (btn_.at("x_lb").IsTriggerPress()) {  // 从面前的地上捡东西
        // Vec6<double> joint1, joint2, joint3, joint4;
        double speed1 = 0.1;
        double acc1 = 0.5;
        auto movemode = MoveMode::abs;
        vector<double> joint1{0, 1.57, -1.57, 0, 0, 0};
        vector<double> joint2{3.1, 1.0, -1.0, 0.0, -0.8, 0.0};
        vector<double> joint3{3.1, 1.2, -0.8, 0.0, -0.7, 0.0};

        Pose pos1{0.5, 0.0, -0.25, 3.141, 0.0, 0.0};
        Pose pos2{0.5, 0.0, -0.3, 3.141, 0.0, 0.0};
        Pose pos3{0.5, 0.0, 0.1, 3.141, 0.0, 0.0};
        speed1 = 1;

        api_->SetMoveLTaskBlock(pos1, movemode, speed1, acc1);

        api_->SetMoveLTaskBlock(pos2, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos3, movemode, speed1, acc1);
        api_->SetMoveJTaskBlock(joint2, movemode, speed1, acc1);
        api_->SetMoveJTaskBlock(joint3, movemode, speed1, acc1);
        api_->SetMoveJTaskBlock(joint2, movemode, speed1, acc1);

        api_->SetMoveJTaskBlock(joint1, movemode, speed1, acc1);
    }

    if (btn_.at("x_rb").IsTriggerPress()) {  // 从面前的地上捡东西
        // Vec6<double> joint1, joint2, joint3, joint4;
        double speed1 = 1;
        double acc1 = 0.5;
        auto movemode = MoveMode::abs;
        vector<double> joint1{0, 0.3, 1.2, 0, 1.57, 0};
        vector<double> joint2{2.8, 0.2, 1.0, 0.0, 1.2, 0.0};
        vector<double> joint3{2.8, 0.3, 1.2, 0.0, 1.57, 0.0};

        Pose pos1{0.4, 0.0, 0.0, 3.141, 0.0, 0.0};
        Pose pos2{0.4, 0.0, -0.1, 3.141, 0.0, 0.0};
        Pose pos3{0.4, 0.0, 0.1, 3.141, 0.0, 0.0};
        speed1 = 1;
        api_->SetMoveLTaskBlock(pos1, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos2, movemode, speed1, acc1);
        api_->SetMoveLTaskBlock(pos3, movemode, speed1, acc1);
        api_->SetMoveJTaskBlock(joint2, movemode, speed1, acc1);
        api_->SetMoveJTaskBlock(joint3, movemode, speed1, acc1);
        api_->SetMoveJTaskBlock(joint2, movemode, speed1, acc1);
        api_->SetMoveJTaskBlock(joint1, movemode, speed1, acc1);
    }

    // 回到零点位置
    if (btn_.at("x").IsTriggerPress()) {  // 回到零点位置

        double speed1 = 0.5;
        double acc1 = 0.5;
        auto movemode = MoveMode::abs;
        vector<double> jointEnd{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        api_->SetMoveJTaskBlock(jointEnd, movemode, speed1, acc1);
    }
}

/**
 * @description: 夹爪功能按键映射 todo
 * @param cmd
 * @return {}
 */
void KeyArm::GripperKey()
{
    if (stick_.at("rt") == 1) {
        // api_->SetGripper(false, 0.5);
        // KeyArm::gripper_.AotuGetGripperOff(1);

        // cout << "gripper close!" << endl;

    } else {
        // cout << "gripper open!" << endl;
        // api_->SetGripper(true, 0.5);
        // KeyArm::gripper_.AotuGetGripperOn(1);
    }
    ///
}

/**
 * @description: 机械臂使能按键映射
 *
 */
void KeyArm::ArmEnableKey()
{
    if (btn_.at("start").IsTriggerPress()) {
        api_->SetCtrlState(ArmCtrl::enable);
    } else if (btn_.at("back").IsTriggerPress()) {
        api_->SetCtrlState(ArmCtrl::disable);
    } else if (btn_.at("up").IsTriggerPress()) {
        std::bitset<32> io = 0x01;
        api_->SetIoOutState("tool", io);
        io.set(2);
        api_->SetIoOutState("panel", io);
    } else if (btn_.at("down").IsTriggerPress()) {
        std::bitset<32> io = 0x00;
        api_->SetIoOutState("tool", io);
        api_->SetIoOutState("panel", io);
    }
}

void KeyArm::SetArmResetKey()
{
    if (api_->IsRun() == false) {
        return;
    }
    if (btn_.at("lb_rb").IsTriggerPress()) {
        // if (Reset_ == true) {
        //     Reset_ = false;
        // } else if (Reset_ == false) {
        //     Reset_ = true;
        // } else {
        //     Reset_ = false;
        // }
        // std::cout << "Reset_:" << Reset_ << endl;
        // RetState ret = api_->SetReset(Reset_);
        std::cout << "Set postionlimit" << endl;
        vector<ValueRange<double>> limit;
        Vec6<double> min, max;
        min << -1, -2, -3, -4, -5, -6;
        max << 1, 2, 3, 4, 5, 6;
        Vec6<ValueRange<double>> range;
        // range.max = max;
        // range.min = min;

        for (int i = 0; i < 6; i++) {
            range(i).max = max(i);
            range(i).min = min(i);
        }
        for (int i = 0; i < 6; i++) {
            limit.push_back(range(i));
        }
        api_->SetPositionLimit(limit);
    }
}
void KeyArm::ChangerateKey()
{
    if (api_->IsRun() == false) {
        return;
    }
    double rate;
    if (btn_.at("right").IsTriggerPress()) {
        rate = 0.7;
        api_->SetRapidrate(rate);
        auto param = api_->GetRapidrate();
        if (param.first == RetState::ok) {
            cout << param.second << endl;
        }
    }
    if (btn_.at("left").IsTriggerPress()) {
        rate = 0.2;
        api_->SetRapidrate(rate);
        auto param = api_->GetRapidrate();
        if (param.first == RetState::ok) {
            cout << param.second << endl;
        }
        std::cout << "Reset_:" << Reset_ << endl;
        // RetState ret = api_->SetReset(Reset_);
    }
}
/**
 * @description: 机械臂手柄按键映射
 * @param cmd
 * @return {}
 */
/*夹爪夹持范围：完全打开0 - 完全闭合100*/
// void KeyDeal::ArmKey(const msg::bs::gamepad_cmd& cmd)
// {                         // 要注意夹爪长度14cm，在设置抓取高度时应考虑此问题
//     if (cmd.btnA == 1) {  // 准备位置
//         Vec6<double> jointStart, jointEnd;
//         double planT = 1000;
//         jointEnd << 0, 1.57, -1.57, 0, 0, 0;
//         api_->SetMoveJTask(jointEnd);
//     }
//     if (cmd.btnB == 1) {  // 面前的桌子上抓取
//         Vec6<double> pos1, pos2, pos3, pos4, pos5, pos6, pos7;
//         Vec6<double> joint1;
//         pos1 << 0.6, 0, 0.3, 3.14158, 0, 0;
//         pos2 << 0.6, 0, 0.2, 3.141, 0, 0;
//         pos3 << 0.6, 0, 0.3, 3.141, 0, 0;
//         pos4 << 0.3, -0.3, 0.3, 3.141, 0, 0;
//         pos5 << 0.3, -0.3, 0.2, 3.141, 0, 0;
//         pos6 << 0.3, -0.3, 0.3, 3.141, 0, 0;
//         pos7 << 0.6, 0, 0.3, 3.14158, 0, 0;

//         joint1 << 0, 1.57, -1.57, 0, 0, 0;

//         api_->SetMoveLTask(pos1);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveLTask(pos2);
//         gripperAPI_.PositionPercentageNew(30);
//         TimerTools::SleepForS(1);
//         api_->SetMoveLTask(pos3);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveLTask(pos4);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveLTask(pos5);
//         TimerTools::SleepForS(0.5);
//         gripperAPI_.PositionPercentageNew(99);
//         TimerTools::SleepForS(1);
//         api_->SetMoveLTask(pos6);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveJTask(joint1);
//         // api_->SetMoveLTask(pos7);
//     }
//     if (cmd.btnY == 1) {  // 开门
//         Vec6<double> joint1, joint2;
//         Vec6<double> pos1, pos2, pos3, pos4, pos5, pos6;

//         joint1 << 0, 1.0, -0.6, 0, -1.8, 0;

//         pos1 << 0.4, 0.0, 0.5, 0, -1.541, 3.141;
//         pos2 << 0.5, 0.0, 0.5, 0, -1.541, 3.141;
//         pos3 << 0.5, 0.0, 0.5, -1.57, -1.0, -1.615;
//         pos4 << 0.55, 0.0, 0.5, -1.57, -1.0, -1.615;
//         pos5 << 0.5, 0.0, 0.5, 0, -1.541, 3.141;
//         pos6 << 0.7, 0.0, 0.5, 0, -1.541, 3.141;

//         api_->SetMoveJTask(joint1);  //
//         TimerTools::SleepForS(0.5);
//         gripperAPI_.PositionPercentageNew(99);
//         api_->SetMoveLTask(pos1);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveLTask(pos2);
//         TimerTools::SleepForS(1);
//         gripperAPI_.PositionPercentageNew(30);
//         TimerTools::SleepForS(1);
//         api_->SetMoveLTask(pos3);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveLTask(pos4);
//         gripperAPI_.PositionPercentageNew(99);
//         TimerTools::SleepForS(1);
//         api_->SetMoveLTask(pos5);
//         gripperAPI_.PositionPercentageNew(30);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveLTask(pos6);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveJTask(joint1);
//     }
//     if (cmd.btnLb == 1) {  // 从面前的地上捡东西
//         Vec6<double> joint1, joint2, joint3, joint4;
//         Vec6<double> pos1, pos2, pos3, pos4, pos5, pos6;

//         joint1 << 0, 1.57, -1.57, 0, 0, 0;
//         joint2 << 3.141, 1.0, -1.0, 0.0, -0.8, 0.0;
//         joint3 << 3.141, 1.2, -0.8, 0.0, -0.7, 0.0;

//         pos1 << 0.5, 0.0, -0.25, 3.141, 0.0, 0.0;
//         pos2 << 0.5, 0.0, -0.3, 3.141, 0.0, 0.0;
//         pos3 << 0.5, 0.0, 0.1, 3.141, 0.0, 0.0;

//         api_->SetMoveLTask(pos1);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveLTask(pos2);
//         TimerTools::SleepForS(0.5);
//         gripperAPI_.PositionPercentageNew(20);
//         TimerTools::SleepForS(1);
//         api_->SetMoveLTask(pos3);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveJTask(joint2);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveJTask(joint3);
//         TimerTools::SleepForS(0.5);
//         gripperAPI_.PositionPercentageNew(99);
//         TimerTools::SleepForS(1);
//         api_->SetMoveJTask(joint2);
//         TimerTools::SleepForS(0.5);
//         api_->SetMoveJTask(joint1);
//     }
//     if (cmd.btnX == 1) {  // 回到零点位置
//         Vec6<double> jointEnd;
//         double planT = 1000;

//         jointEnd << 0.0, 0.0, 0.0, 0.0, -0.8, 0.0;
//         api_->SetMoveJTask(jointEnd);
//     }
// }

// void KeyDeal::GripperKey(const msg::bs::gamepad_cmd& cmd)
// {
//     // int ret = 1;
//     if (cmd.btnRb == 1) {
//         gripperAPI_.PositionPercentageNew(90);
//         // MsgSend gripSend("gripper close!");
//         // gripSend.Send(ret);
//         //  cout << "gripper close!" << endl;
//     } else {
//         gripperAPI_.PositionPercentageNew(20);
//     }
// }