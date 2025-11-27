
#include "simQuadruped.hpp"

#include "baseline.hpp"
#include "mathTools.hpp"
#include "orientation_tools.h"
using namespace std;

SimQuadruped::SimQuadruped()
{
    qInitMit << -0.3582, 0.3582, -0.3582, 0.3582, 0.3279, 0.3279, 0.3279, 0.3279, 2.818, 2.818, 2.818, 2.818;
    InitRecv();
    usleep(300000);  // wait for subscribers start
    InitSend();
}

void SimQuadruped::InitSend()
{
    _servo_pub[0] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/FR_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/FR_thigh_controller/command", 1);
    _servo_pub[2] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/FR_calf_controller/command", 1);
    _servo_pub[3] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/FL_hip_controller/command", 1);
    _servo_pub[4] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/FL_thigh_controller/command", 1);
    _servo_pub[5] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/FL_calf_controller/command", 1);
    _servo_pub[6] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/RR_hip_controller/command", 1);
    _servo_pub[7] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/RR_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/RR_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/RL_hip_controller/command", 1);
    _servo_pub[10] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/RL_thigh_controller/command", 1);
    _servo_pub[11] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/RL_calf_controller/command", 1);
    _servo_pub[12] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/FR_wheel_controller/command", 1);
    _servo_pub[13] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/FL_wheel_controller/command", 1);
    _servo_pub[14] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/RR_wheel_controller/command", 1);
    _servo_pub[15] = _nm.advertise<robot_msgs::MotorCommand>("/" + _robot_name + "_gazebo/RL_wheel_controller/command", 1);
}

void SimQuadruped::InitRecv()
{
    _imu_sub = _nm.subscribe("/trunk_imu", 1, &SimQuadruped::imuCallback, this);
    _model_sub = _nm.subscribe("/gazebo/model_states", 1, &SimQuadruped::ModelStateCallback, this);
    _servo_sub[0] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_hip_controller/state", 1, &SimQuadruped::FRhipCallback, this);
    _servo_sub[1] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_thigh_controller/state", 1, &SimQuadruped::FRthighCallback, this);
    _servo_sub[2] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_calf_controller/state", 1, &SimQuadruped::FRcalfCallback, this);
    _servo_sub[3] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_hip_controller/state", 1, &SimQuadruped::FLhipCallback, this);
    _servo_sub[4] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_thigh_controller/state", 1, &SimQuadruped::FLthighCallback, this);
    _servo_sub[5] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_calf_controller/state", 1, &SimQuadruped::FLcalfCallback, this);
    _servo_sub[6] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_hip_controller/state", 1, &SimQuadruped::RRhipCallback, this);
    _servo_sub[7] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_thigh_controller/state", 1, &SimQuadruped::RRthighCallback, this);
    _servo_sub[8] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_calf_controller/state", 1, &SimQuadruped::RRcalfCallback, this);
    _servo_sub[9] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_hip_controller/state", 1, &SimQuadruped::RLhipCallback, this);
    _servo_sub[10] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_thigh_controller/state", 1, &SimQuadruped::RLthighCallback, this);
    _servo_sub[11] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_calf_controller/state", 1, &SimQuadruped::RLcalfCallback, this);
    _servo_sub[12] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_wheel_controller/state", 1, &SimQuadruped::FRwheelCallback, this);
    _servo_sub[13] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_wheel_controller/state", 1, &SimQuadruped::FLwheelCallback, this);
    _servo_sub[14] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_wheel_controller/state", 1, &SimQuadruped::RRwheelCallback, this);
    _servo_sub[15] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_wheel_controller/state", 1, &SimQuadruped::RLwheelCallback, this);
}

void SimQuadruped::Run()
{
    while (ros::ok()) {
        // 发送给模拟器的电机命令
        auto ret = MsgTryRecv<msg::qr::motor_cmd>("qr::motor_cmd", this);
        if (ret.has_value()) {  // 关节控制的发送方式
            auto msgMotionCmd = ret.value();
            SetJointCmd(msgMotionCmd);
        }

        auto ret2 = MsgTryRecv<msg::wheel::motor_cmd>("wheel::motor_cmd", this);
        if (ret2.has_value()) {  // 轮式控制的发送方式
            SetJointCmd(ret2.value());
        }

        // 从模拟器获取数据返回
        MsgTrySend("qr::motor_ret", motorData_);
        MsgTrySend("wheel::motor_ret", motorWheelRet_);
        ros::spinOnce();
    }
}

void SimQuadruped::SetJointCmd(const msg::qr::motor_cmd& msgMotionCmd)
{
    robot_msgs::MotorCommand unitreeMsg;

    // unitreeMsg.mode = 10;
    //  LOG_DEBUG("msgMotionCmd {}", msgMotionCmd.q_des_abad[0]);
    unitreeMsg.alpha = msgMotionCmd.leg[0][0].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[0][0].torq;
    unitreeMsg.blta = msgMotionCmd.leg[0][0].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[0][0].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[0][0].k1;
    // 发送前右abad FR_hip
    // std::cout << "abad0:" << unitreeMsg.alpha << std::endl;
    // std::cout << "abad0:" << unitreeMsg.torq << std::endl;
    _servo_pub[0].publish(unitreeMsg);

    //-----------------------------//
    unitreeMsg.alpha = msgMotionCmd.leg[0][1].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[0][1].torq;
    unitreeMsg.blta = msgMotionCmd.leg[0][1].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[0][1].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[0][1].k1;
    // 发送前右hip FR_thigh
    _servo_pub[1].publish(unitreeMsg);

    //-----------------------------//
    unitreeMsg.alpha = msgMotionCmd.leg[0][2].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[0][2].torq;
    unitreeMsg.blta = msgMotionCmd.leg[0][2].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[0][2].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[0][2].k1;
    // 发送前右knee FR_calf
    // std::cout << "knee0:" << unitreeMsg.alpha << std::endl;
    // std::cout << "knee0:" << unitreeMsg.torq << std::endl;
    _servo_pub[2].publish(unitreeMsg);

    unitreeMsg.alpha = msgMotionCmd.leg[1][0].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[1][0].torq;
    unitreeMsg.blta = msgMotionCmd.leg[1][0].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[1][0].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[1][0].k1;
    // 发送前左abad FL_hip
    // std::cout << "abad1:" << unitreeMsg.alpha << std::endl;
    // std::cout << "abad1:" << unitreeMsg.torq << std::endl;
    _servo_pub[3].publish(unitreeMsg);

    unitreeMsg.alpha = msgMotionCmd.leg[1][1].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[1][1].torq;
    unitreeMsg.blta = msgMotionCmd.leg[1][1].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[1][1].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[1][1].k1;
    // 发送前左hip FL_thigh
    // std::cout << "hip1:" << unitreeMsg.alpha << std::endl;
    // std::cout << "hip1:" << unitreeMsg.torq << std::endl;
    _servo_pub[4].publish(unitreeMsg);

    unitreeMsg.alpha = msgMotionCmd.leg[1][2].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[1][2].torq;
    unitreeMsg.blta = msgMotionCmd.leg[1][2].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[1][2].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[1][2].k1;
    // 发送前左knee FL_calf
    // std::cout << "knee1:" << unitreeMsg.alpha << std::endl;
    // std::cout << "knee1:" << unitreeMsg.torq << std::endl;
    _servo_pub[5].publish(unitreeMsg);

    unitreeMsg.alpha = msgMotionCmd.leg[2][0].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[2][0].torq;
    unitreeMsg.blta = msgMotionCmd.leg[2][0].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[2][0].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[2][0].k1;
    // 发送后右abad RR_hip
    // std::cout << "abad2:" << unitreeMsg.alpha << std::endl;
    // std::cout << "abad2:" << unitreeMsg.torq << std::endl;
    _servo_pub[6].publish(unitreeMsg);

    unitreeMsg.alpha = msgMotionCmd.leg[2][1].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[2][1].torq;
    unitreeMsg.blta = msgMotionCmd.leg[2][1].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[2][1].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[2][1].k1;
    // 发送后右hip RR_thigh
    // std::cout << "hip2:" << unitreeMsg.alpha << std::endl;
    // std::cout << "hip2:" << unitreeMsg.torq << std::endl;
    _servo_pub[7].publish(unitreeMsg);

    unitreeMsg.alpha = msgMotionCmd.leg[2][2].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[2][2].torq;
    unitreeMsg.blta = msgMotionCmd.leg[2][2].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[2][2].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[2][2].k1;
    // 发送后右knee RR_calf
    // std::cout << "knee2:" << unitreeMsg.alpha << std::endl;
    // std::cout << "knee2:" << unitreeMsg.torq << std::endl;
    _servo_pub[8].publish(unitreeMsg);

    unitreeMsg.alpha = msgMotionCmd.leg[3][0].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[3][0].torq;
    unitreeMsg.blta = msgMotionCmd.leg[3][0].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[3][0].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[3][0].k1;
    // 发送后左abad RL_hip
    // std::cout << "abad3:" << unitreeMsg.alpha << std::endl;
    // std::cout << "abad3:" << unitreeMsg.torq << std::endl;
    _servo_pub[9].publish(unitreeMsg);

    unitreeMsg.alpha = msgMotionCmd.leg[3][1].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[3][1].torq;
    unitreeMsg.blta = msgMotionCmd.leg[3][1].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[3][1].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[3][1].k1;
    // 发送后左hip RL_thigh
    // std::cout << "hip3:" << unitreeMsg.alpha << std::endl;
    // std::cout << "hip3:" << unitreeMsg.torq << std::endl;
    _servo_pub[10].publish(unitreeMsg);

    unitreeMsg.alpha = msgMotionCmd.leg[3][2].alpha;
    unitreeMsg.torq = msgMotionCmd.leg[3][2].torq;
    unitreeMsg.blta = msgMotionCmd.leg[3][2].blta;
    unitreeMsg.k2 = msgMotionCmd.leg[3][2].k2;
    unitreeMsg.k1 = msgMotionCmd.leg[3][2].k1;
    // 发送后左knee RL_thigh
    // std::cout << "knee3:" << unitreeMsg.alpha << std::endl;
    // std::cout << "knee3:" << unitreeMsg.torq << std::endl;
    _servo_pub[11].publish(unitreeMsg);
}

void SimQuadruped::SetJointCmd(const msg::wheel::motor_cmd& cmd)
{
    robot_msgs::MotorCommand unitreeMsg;

    unitreeMsg.alpha = cmd.leg[0][0].alpha;
    unitreeMsg.torq = cmd.leg[0][0].torq;
    unitreeMsg.blta = cmd.leg[0][0].blta;
    unitreeMsg.k2 = cmd.leg[0][0].k2;
    unitreeMsg.k1 = cmd.leg[0][0].k1;
    // 发送前右abad FR_hip
    _servo_pub[0].publish(unitreeMsg);

    //-----------------------------//
    unitreeMsg.alpha = cmd.leg[0][1].alpha;
    unitreeMsg.torq = cmd.leg[0][1].torq;
    unitreeMsg.blta = cmd.leg[0][1].blta;
    unitreeMsg.k2 = cmd.leg[0][1].k2;
    unitreeMsg.k1 = cmd.leg[0][1].k1;
    // 发送前右hip FR_thigh
    _servo_pub[1].publish(unitreeMsg);

    //-----------------------------//
    unitreeMsg.alpha = cmd.leg[0][2].alpha;
    unitreeMsg.torq = cmd.leg[0][2].torq;
    unitreeMsg.blta = cmd.leg[0][2].blta;
    unitreeMsg.k2 = cmd.leg[0][2].k2;
    unitreeMsg.k1 = cmd.leg[0][2].k1;
    // 发送前右knee FR_calf
    _servo_pub[2].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[1][0].alpha;
    unitreeMsg.torq = cmd.leg[1][0].torq;
    unitreeMsg.blta = cmd.leg[1][0].blta;
    unitreeMsg.k2 = cmd.leg[1][0].k2;
    unitreeMsg.k1 = cmd.leg[1][0].k1;
    // 发送前左abad FL_hip
    _servo_pub[3].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[1][1].alpha;
    unitreeMsg.torq = cmd.leg[1][1].torq;
    unitreeMsg.blta = cmd.leg[1][1].blta;
    unitreeMsg.k2 = cmd.leg[1][1].k2;
    unitreeMsg.k1 = cmd.leg[1][1].k1;
    // 发送前左hip FL_thigh
    _servo_pub[4].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[1][2].alpha;
    unitreeMsg.torq = cmd.leg[1][2].torq;
    unitreeMsg.blta = cmd.leg[1][2].blta;
    unitreeMsg.k2 = cmd.leg[1][2].k2;
    unitreeMsg.k1 = cmd.leg[1][2].k1;
    // 发送前左knee FL_calf
    _servo_pub[5].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[2][0].alpha;
    unitreeMsg.torq = cmd.leg[2][0].torq;
    unitreeMsg.blta = cmd.leg[2][0].blta;
    unitreeMsg.k2 = cmd.leg[2][0].k2;
    unitreeMsg.k1 = cmd.leg[2][0].k1;
    // 发送后右abad RR_hip
    _servo_pub[6].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[2][1].alpha;
    unitreeMsg.torq = cmd.leg[2][1].torq;
    unitreeMsg.blta = cmd.leg[2][1].blta;
    unitreeMsg.k2 = cmd.leg[2][1].k2;
    unitreeMsg.k1 = cmd.leg[2][1].k1;
    // 发送后右hip RR_thigh
    _servo_pub[7].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[2][2].alpha;
    unitreeMsg.torq = cmd.leg[2][2].torq;
    unitreeMsg.blta = cmd.leg[2][2].blta;
    unitreeMsg.k2 = cmd.leg[2][2].k2;
    unitreeMsg.k1 = cmd.leg[2][2].k1;
    // 发送后右knee RR_calf
    _servo_pub[8].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[3][0].alpha;
    unitreeMsg.torq = cmd.leg[3][0].torq;
    unitreeMsg.blta = cmd.leg[3][0].blta;
    unitreeMsg.k2 = cmd.leg[3][0].k2;
    unitreeMsg.k1 = cmd.leg[3][0].k1;
    // 发送后左abad RL_hip
    _servo_pub[9].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[3][1].alpha;
    unitreeMsg.torq = cmd.leg[3][1].torq;
    unitreeMsg.blta = cmd.leg[3][1].blta;
    unitreeMsg.k2 = cmd.leg[3][1].k2;
    unitreeMsg.k1 = cmd.leg[3][1].k1;
    // 发送后左hip RL_thigh
    _servo_pub[10].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[3][2].alpha;
    unitreeMsg.torq = cmd.leg[3][2].torq;
    unitreeMsg.blta = cmd.leg[3][2].blta;
    unitreeMsg.k2 = cmd.leg[3][2].k2;
    unitreeMsg.k1 = cmd.leg[3][2].k1;
    // 发送后左knee RL_thigh
    _servo_pub[11].publish(unitreeMsg);

    // 发送轮子
    unitreeMsg.alpha = cmd.leg[0][3].alpha;
    unitreeMsg.torq = cmd.leg[0][3].torq;
    unitreeMsg.blta = cmd.leg[0][3].blta;
    unitreeMsg.k2 = cmd.leg[0][3].k2;
    unitreeMsg.k1 = cmd.leg[0][3].k1;
    _servo_pub[12].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[1][3].alpha;
    unitreeMsg.torq = cmd.leg[1][3].torq;
    unitreeMsg.blta = cmd.leg[1][3].blta;
    unitreeMsg.k2 = cmd.leg[1][3].k2;
    unitreeMsg.k1 = cmd.leg[1][3].k1;
    _servo_pub[13].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[2][3].alpha;
    unitreeMsg.torq = cmd.leg[2][3].torq;
    unitreeMsg.blta = cmd.leg[2][3].blta;
    unitreeMsg.k2 = cmd.leg[2][3].k2;
    unitreeMsg.k1 = cmd.leg[2][3].k1;
    _servo_pub[14].publish(unitreeMsg);

    unitreeMsg.alpha = cmd.leg[3][3].alpha;
    unitreeMsg.torq = cmd.leg[3][3].torq;
    unitreeMsg.blta = cmd.leg[3][3].blta;
    unitreeMsg.k2 = cmd.leg[3][3].k2;
    unitreeMsg.k1 = cmd.leg[3][3].k1;
    _servo_pub[15].publish(unitreeMsg);
}

void SimQuadruped::ModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // 遍历模型列表
    for (size_t i = 0; i < msg->name.size(); i++) {
        // 如果不确定模型的名字，可以先打印出来看一看
        // std::cout << "model_name: " << msg->name[i] << std::endl;
        // 如果当前模型是 our_model
        if (msg->name[i] == _robot_name + "_gazebo") {
            // 获取模型的位置和姿态
            // geometry_msgs::Pose pose = msg->pose[i];
            // 处理信息
            // double x = pose.position.x;
            // double y = pose.position.y;
            // double z = pose.position.z;

            //  std::cout << "x:" << x << "y" << y << "z" << z << std::endl;
            // // 获取模型的速度和角速度
            // geometry_msgs::Twist twist = msg->twist[i];
            // // 处理信息
            // double linear_x = twist.linear.x;    // 模型线速度在 x 轴的分量
            // double linear_y = twist.linear.y;    // 模型线速度在 y 轴的分量
            // double linear_z = twist.linear.z;    // 模型线速度在 z 轴的分量
            // double angular_x = twist.angular.x;  // 模型角速度在 x 轴的分量
            // double angular_y = twist.angular.y;  // 模型角速度在 y 轴的分量
            // double angular_z = twist.angular.z;  // 模型角速度在 z 轴的分量
        }
    }
}
Vec3<double> SimQuadruped::quat2RPY(Vec4<double> q)
{
    Vec3<double> rpy;
    double as = min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
    rpy(2) = atan2(2 * (q[1] * q[2] + q[0] * q[3]), pow(q[0], 2) + pow(q[1], 2) - pow(q[2], 2) - pow(q[3], 2));
    rpy(1) = asin(as);
    rpy(0) = atan2(2 * (q[2] * q[3] + q[0] * q[1]), pow(q[0], 2) - pow(q[1], 2) - pow(q[2], 2) + pow(q[3], 2));
    return rpy;
}
/**
 * @description: IMU数据回调，注意没有RPY，但多了个四元数
 * @param msg
 * @return {}
 */
void SimQuadruped::imuCallback(const sensor_msgs::Imu& msg)
{
#if 0
    // 注意没有RPY，但多了个四元数
    imu.quaternion[0] = msg.orientation.w;
    imu.quaternion[1] = msg.orientation.x;
    imu.quaternion[2] = msg.orientation.y;
    imu.quaternion[3] = msg.orientation.z;

    imu.gyroscope[0] = msg.angular_velocity.x;
    imu.gyroscope[1] = msg.angular_velocity.y;
    imu.gyroscope[2] = msg.angular_velocity.z;

    imu.accelerometer[0] = msg.linear_acceleration.x;
    imu.accelerometer[1] = msg.linear_acceleration.y;
    imu.accelerometer[2] = msg.linear_acceleration.z;
#endif
    msg::imu_data imuData;
    // 数据转换
    Vec4<double> _ang;
    _ang[0] = msg.orientation.w;
    _ang[1] = msg.orientation.x;
    _ang[2] = msg.orientation.y;
    _ang[3] = msg.orientation.z;

    Vec3<double> _angRPY = quat2RPY(_ang);

    imuData.ang[0] = _angRPY(0);
    imuData.ang[1] = _angRPY(1);
    imuData.ang[2] = _angRPY(2);

    imuData.gyro[0] = msg.angular_velocity.x;
    imuData.gyro[1] = msg.angular_velocity.y;
    imuData.gyro[2] = msg.angular_velocity.z;
    // for (int i = 0; i < 3; i++) {
    //     std::cout << "gyro" << i << ":" << imuData.gyro[i] << std::endl;
    // }

    imuData.acc[0] = msg.linear_acceleration.x;
    imuData.acc[1] = msg.linear_acceleration.y;
    imuData.acc[2] = msg.linear_acceleration.z;

    imuData.quat[0] = msg.orientation.w;
    imuData.quat[1] = msg.orientation.x;
    imuData.quat[2] = msg.orientation.y;
    imuData.quat[3] = msg.orientation.z;

    // for (int i = 0; i < 3; i++) {
    //     std::cout << "acc" << i << ":" << imuData.acc[i] << std::endl;
    // }
    // LOG_DEBUG("imu {} {} {} {}", imuData.quat[0],imuData.quat[1],imuData.quat[2],imuData.quat[3]);
    MsgTrySend("qr::imuData", imuData);
    // std::cout << "2222" << std::endl;
    // LOG_INFO("IMU gyro:{} {} {}", imuData.gyro[0], imuData.gyro[1], imuData.gyro[2]);
    // LOG_INFO("IMU acc :{} {} {}", imuData.acc[0], imuData.acc[1], imuData.acc[2]);
}

/**
 * @description: 前右hip
 * @param msg
 * @return {}
 */
void SimQuadruped::FRhipCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[0][0].alpha = msg.alpha;
    motorData_.leg[0][0].torq = msg.torq;
    motorData_.leg[0][0].blta = msg.blta;

    motorWheelRet_.leg[0][0].alpha = msg.alpha;
    motorWheelRet_.leg[0][0].torq = msg.torq;
    motorWheelRet_.leg[0][0].blta = msg.blta;
}

/**
 * @description: 前右knee
 * @param msg
 * @return {}
 */
void SimQuadruped::FRthighCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[0][1].alpha = msg.alpha;
    motorData_.leg[0][1].torq = msg.torq;
    motorData_.leg[0][1].blta = msg.blta;

    motorWheelRet_.leg[0][1].alpha = msg.alpha;
    motorWheelRet_.leg[0][1].torq = msg.torq;
    motorWheelRet_.leg[0][1].blta = msg.blta;
}

/**
 * @description: 前右abad
 * @param msg
 * @return {}
 */
void SimQuadruped::FRcalfCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[0][2].alpha = msg.alpha;
    motorData_.leg[0][2].torq = msg.torq;
    motorData_.leg[0][2].blta = msg.blta;

    motorWheelRet_.leg[0][2].alpha = msg.alpha;
    motorWheelRet_.leg[0][2].torq = msg.torq;
    motorWheelRet_.leg[0][2].blta = msg.blta;
}

void SimQuadruped::FLhipCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[1][0].alpha = msg.alpha;
    motorData_.leg[1][0].torq = msg.torq;
    motorData_.leg[1][0].blta = msg.blta;

    motorWheelRet_.leg[1][0].alpha = msg.alpha;
    motorWheelRet_.leg[1][0].torq = msg.torq;
    motorWheelRet_.leg[1][0].blta = msg.blta;
}

void SimQuadruped::FLthighCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[1][1].alpha = msg.alpha;
    motorData_.leg[1][1].torq = msg.torq;
    motorData_.leg[1][1].blta = msg.blta;

    motorWheelRet_.leg[1][1].alpha = msg.alpha;
    motorWheelRet_.leg[1][1].torq = msg.torq;
    motorWheelRet_.leg[1][1].blta = msg.blta;
}

void SimQuadruped::FLcalfCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[1][2].alpha = msg.alpha;
    motorData_.leg[1][2].torq = msg.torq;
    motorData_.leg[1][2].blta = msg.blta;

    motorWheelRet_.leg[1][2].alpha = msg.alpha;
    motorWheelRet_.leg[1][2].torq = msg.torq;
    motorWheelRet_.leg[1][2].blta = msg.blta;
}

void SimQuadruped::RRhipCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[2][0].alpha = msg.alpha;
    motorData_.leg[2][0].torq = msg.torq;
    motorData_.leg[2][0].blta = msg.blta;

    motorWheelRet_.leg[2][0].alpha = msg.alpha;
    motorWheelRet_.leg[2][0].torq = msg.torq;
    motorWheelRet_.leg[2][0].blta = msg.blta;
}

void SimQuadruped::RRthighCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[2][1].alpha = msg.alpha;
    motorData_.leg[2][1].torq = msg.torq;
    motorData_.leg[2][1].blta = msg.blta;

    motorWheelRet_.leg[2][1].alpha = msg.alpha;
    motorWheelRet_.leg[2][1].torq = msg.torq;
    motorWheelRet_.leg[2][1].blta = msg.blta;
}

void SimQuadruped::RRcalfCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[2][2].alpha = msg.alpha;
    motorData_.leg[2][2].torq = msg.torq;
    motorData_.leg[2][2].blta = msg.blta;

    motorWheelRet_.leg[2][2].alpha = msg.alpha;
    motorWheelRet_.leg[2][2].torq = msg.torq;
    motorWheelRet_.leg[2][2].blta = msg.blta;
}

void SimQuadruped::RLhipCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[3][0].alpha = msg.alpha;
    motorData_.leg[3][0].torq = msg.torq;
    motorData_.leg[3][0].blta = msg.blta;

    motorWheelRet_.leg[3][0].alpha = msg.alpha;
    motorWheelRet_.leg[3][0].torq = msg.torq;
    motorWheelRet_.leg[3][0].blta = msg.blta;
}

void SimQuadruped::RLthighCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[3][1].alpha = msg.alpha;
    motorData_.leg[3][1].torq = msg.torq;
    motorData_.leg[3][1].blta = msg.blta;

    motorWheelRet_.leg[3][1].alpha = msg.alpha;
    motorWheelRet_.leg[3][1].torq = msg.torq;
    motorWheelRet_.leg[3][1].blta = msg.blta;
}

void SimQuadruped::RLcalfCallback(const robot_msgs::MotorState& msg)
{
    motorData_.leg[3][2].alpha = msg.alpha;
    motorData_.leg[3][2].torq = msg.torq;
    motorData_.leg[3][2].blta = msg.blta;

    motorWheelRet_.leg[3][2].alpha = msg.alpha;
    motorWheelRet_.leg[3][2].torq = msg.torq;
    motorWheelRet_.leg[3][2].blta = msg.blta;
}

void SimQuadruped::FRwheelCallback(const robot_msgs::MotorState& msg)
{
    motorWheelRet_.leg[0][3].alpha = msg.alpha;
    motorWheelRet_.leg[0][3].torq = msg.torq;
    motorWheelRet_.leg[0][3].blta = msg.blta;
}
void SimQuadruped::FLwheelCallback(const robot_msgs::MotorState& msg)
{
    motorWheelRet_.leg[1][3].alpha = msg.alpha;
    motorWheelRet_.leg[1][3].torq = msg.torq;
    motorWheelRet_.leg[1][3].blta = msg.blta;
}
void SimQuadruped::RRwheelCallback(const robot_msgs::MotorState& msg)
{
    motorWheelRet_.leg[2][3].alpha = msg.alpha;
    motorWheelRet_.leg[2][3].torq = msg.torq;
    motorWheelRet_.leg[2][3].blta = msg.blta;
}
void SimQuadruped::RLwheelCallback(const robot_msgs::MotorState& msg)
{
    motorWheelRet_.leg[3][3].alpha = msg.alpha;
    motorWheelRet_.leg[3][3].torq = msg.torq;
    motorWheelRet_.leg[3][3].blta = msg.blta;
}
