
#include "simArm.hpp"

#include "baseline.hpp"

using namespace std;

SimArm::SimArm()
{
    recieved_cmd_ = false;
    InitRecv();
    ros::AsyncSpinner subSpinner(1);  // one threads
    subSpinner.start();
    usleep(300000);  // wait for subscribers start
    InitSend();
}

void SimArm::InitSend()
{
    _arm_pub = _nm.advertise<trajectory_msgs::JointTrajectory>("/arm_position_controller/command", 1);
    //_arm_pub = _nm.advertise<sensor_msgs::JointState>("/joint_states", 1);
}
void SimArm::InitRecv()
{
    _arm_sub = _nm.subscribe("/arm_position_controller/state", 1, &SimArm::PosFeedback, this);

    // _arm_sub = _nm.subscribe<control_msgs::JointTrajectoryControllerState>("/arm_position_controller/states", 1, &SimArm::PosFeedback, this)
}

// todo:
void SimArm::Run(std::string run_state)
{
    while (ros::ok()) {
        SendMotorCmd();
        Send2Ros(run_state);
        TimerTools::SleepForMs(4);
    }
}

// todo:
void SimArm::Send2Ros(std::string run_state)
{
    // sensor_msgs::JointState joint_state;
    traj_.header.stamp = ros::Time::now();
    traj_.header.frame_id = "base_link";
    auto robotConfg = GetRobotConfigDirect<string>("ctrModel");
    if (robotConfg == "arm") {
        traj_.joint_names.resize(6);
        traj_.points.resize(1);

        traj_.points[0].positions.resize(6);
        traj_.points[0].velocities.resize(6);
        // 关节注册
        traj_.joint_names[0] = "Joint1";
        traj_.joint_names[1] = "Joint2";
        traj_.joint_names[2] = "Joint3";
        traj_.joint_names[3] = "Joint4";
        traj_.joint_names[4] = "Joint5";
        traj_.joint_names[5] = "Joint6";
    } else if (robotConfg == "arm-iris") {
        traj_.joint_names.resize(6);
        traj_.points.resize(1);

        traj_.points[0].positions.resize(6);
        traj_.points[0].velocities.resize(6);
        // 关节注册
        traj_.joint_names[0] = "Joint1";
        traj_.joint_names[1] = "Joint2";
        traj_.joint_names[2] = "Joint3";
        traj_.joint_names[3] = "Joint4";
        traj_.joint_names[4] = "Joint5";
        traj_.joint_names[5] = "Joint6";
    } else if (robotConfg == "arm-qa01") {
        traj_.joint_names.resize(6);
        traj_.points.resize(1);

        traj_.points[0].positions.resize(6);
        traj_.points[0].velocities.resize(6);
        // 关节注册
        traj_.joint_names[0] = "Joint1";
        traj_.joint_names[1] = "Joint2";
        traj_.joint_names[2] = "Joint3";
        traj_.joint_names[3] = "Joint4";
        traj_.joint_names[4] = "Joint5";
        traj_.joint_names[5] = "Joint6";
        // traj_.joint_names[6] = "Joint8";
    } else if (robotConfg == "arm-ga701") {
        traj_.joint_names.resize(7);
        traj_.points.resize(1);

        traj_.points[0].positions.resize(7);
        traj_.points[0].velocities.resize(7);
        // 关节注册
        traj_.joint_names[0] = "Joint1";
        traj_.joint_names[1] = "Joint2";
        traj_.joint_names[2] = "Joint3";
        traj_.joint_names[3] = "Joint4";
        traj_.joint_names[4] = "Joint5";
        traj_.joint_names[5] = "Joint6";
        traj_.joint_names[6] = "Joint7";
    }

    if (run_state == "0") {
        // * ---发定值--- * //
        static std::chrono::system_clock::time_point clock_start = std::chrono::system_clock::now();
        for (size_t i = 0; i < traj_.joint_names.size(); i++) {
            traj_.points[0].positions[i] = 0.4;
        }
        auto clock_now = std::chrono::system_clock::now();
        auto time_duration = std::chrono::duration_cast<std::chrono::seconds>(clock_now - clock_start);
        if (time_duration.count() > 6) return;
        // time_from_start: 表示轨迹点相对于轨迹起始时刻的时间
        traj_.points[0].time_from_start = ros::Duration(1);
        _arm_pub.publish(traj_);
        ros::spinOnce();

    } else if (run_state == "1") {
        // * ---发正弦周期关节角--- * //
        std::vector<double> joint_min = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> joint_max = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        static auto clock_start = std::chrono::system_clock::now();
        auto clock_now = std::chrono::system_clock::now();
        int t_cur = std::chrono::duration_cast<std::chrono::milliseconds>(clock_now - clock_start).count();
        if (t_cur < 1000) {
            for (size_t i = 0; i < traj_.joint_names.size(); ++i) {
                traj_.points[0].positions[i] = joint_min[i];
            }
            traj_.points[0].time_from_start = ros::Duration(1);
            _arm_pub.publish(traj_);
            ros::spinOnce();
            TimerTools::SleepForMs(2);
            return;
        }
        static int t_stage = t_cur;
        int period_ms = 4000;
        int t_res = (t_cur - t_stage) % period_ms;
        double weight = (std::sin(2 * M_PI * t_res / period_ms - 0.5 * M_PI) + 1) * 0.5;
        for (size_t i = 0; i < traj_.joint_names.size(); i++) {
            traj_.points[0].positions[i] = joint_max[i] * weight + joint_min[i] * (1 - weight);
        }
        if (std::chrono::duration_cast<std::chrono::milliseconds>(clock_now - clock_start).count() >= t_stage + period_ms * 4) return;
        traj_.points[0].time_from_start = ros::Duration(0.5);
        _arm_pub.publish(traj_);
        ros::spinOnce();

    } else {
        // * ---原来的--- * //
        for (size_t i = 0; i < traj_.joint_names.size(); i++) {
            traj_.points[0].positions[i] = cmd_.motor[i].alpha;
            // std::cout << "cmd_.motor[" << i << "].alpha: " << cmd_.motor[i].alpha << std::endl;
        }
        traj_.points[0].time_from_start = ros::Duration(0.5);
        // 没收到电机控制指令之前不能将错误的cmd_.motor赋值给traj_消息进行下发
        if (!recieved_cmd_) {
            return;
        }
        _arm_pub.publish(traj_);
        ros::spinOnce();
    }
}

void SimArm::PosFeedback(const control_msgs::JointTrajectoryControllerState& jointstate)
{
    msg::arm_data motorData;
    for (size_t i = 0; i < jointstate.actual.positions.size(); i++) {
        motorData.motor[i].alpha = jointstate.actual.positions[i];
        motorData.motor[i].torq = jointstate.actual.velocities[i];
    }
    MsgTrySend("arm::arm_data", motorData);
}

//////////////电机特殊处理部分/////////////////////////
void SimArm::SendMotorCmd()
{
    auto ret = MsgTryRecv<msg::arm_cmd>("arm::arm_cmd", this);
    if (ret) {
        recieved_cmd_ = true;
        cmd_ = ret.value();
    }
}
