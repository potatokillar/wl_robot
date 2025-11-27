
#include "fsmHandStand.hpp"

#include "baseline.hpp"
#include "eventMsg.hpp"
#include "fsmStand.hpp"
#include "orientation_tools.h"

using namespace std;
using namespace ori;

FsmHandStand::FsmHandStand(ContrContainer contr, std::shared_ptr<JpData> jpData) : FsmState(AlgoState::handStand, contr, jpData)
{
    isWheel_ = (GetQrParam().model.contains(QrModel::gazebo_w) || GetQrParam().model.contains(QrModel::linkV2_3_w));
    int joint_num = isWheel_ ? 16 : 12;
    input_tensor_size_ = isWheel_ ? 1 * 50 : 1 * 42;

    defaultJoint_ = std::vector<double>(joint_num, 0.0);
    actionsOut_ = std::vector<float>(joint_num, 0.0);
    lastActions_ = std::vector<float>(joint_num, 0.0);
    sendActions_ = std::vector<float>(joint_num, 0.0);
    lastSendActions_ = std::vector<float>(joint_num, 0.0);

    auto enable = GetRobotConfigDirect<bool>("enableParam").value();
    if (enable) {
        printInfo_ = GetRobotConfigDirect<bool>("printInfo").value();
        aiActionScale_ = GetRobotConfigDirect<double>("aiActionScale").value();
        onnxPath_ = GetRobotConfigDirect<std::string>("onnxPath", "handstand").value();
        defaultJoint_ = GetRobotConfigDirect<vector<double>>("defaultJoints", "handstand").value();
        kFilter_ = GetRobotConfigDirect<double>("kFilter", "handstand").value();
        if (!isWheel_) {
            aiSend_.k2.fill(GetRobotConfigDirect<double>("kp", "handstand").value());
            aiSend_.k1.fill(GetRobotConfigDirect<double>("kd", "handstand").value());
        } else {
            std::vector<double> kp = GetRobotConfigDirect<std::vector<double>>("kp", "handstand").value();
            std::vector<double> kd = GetRobotConfigDirect<std::vector<double>>("kd", "handstand").value();
            wheelAiSend_.k2 = Mat44<double>(kp.data());  // 此法构造的kpkd矩阵为toml文件中所写格式的转置
            wheelAiSend_.k1 = Mat44<double>(kd.data());
        }
    } else {
        LOG_WARN("Custom Param Disabled! Use Default.");
    }

    // TODO: lastActions needs initialize
    // lastSendActions_[0] = motorCmdOld.alpha(3);
    // lastSendActions_[1] = motorCmdOld.alpha(4);
    // lastSendActions_[2] = motorCmdOld.alpha(5);
    // lastSendActions_[3] = motorCmdOld.alpha(0);
    // lastSendActions_[4] = motorCmdOld.alpha(1);
    // lastSendActions_[5] = motorCmdOld.alpha(2);
    // lastSendActions_[6] = motorCmdOld.alpha(9);
    // lastSendActions_[7] = motorCmdOld.alpha(10);
    // lastSendActions_[8] = motorCmdOld.alpha(11);
    // lastSendActions_[9] = motorCmdOld.alpha(6);
    // lastSendActions_[10] = motorCmdOld.alpha(7);
    // lastSendActions_[11] = motorCmdOld.alpha(8);

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(8);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    try {
        session_ = make_unique<Ort::Session>(env, onnxPath_.c_str(), session_options);
    } catch (const std::exception& e) {
        LOG_ERROR("load model error: {}", e.what());
        return;
    }

    LOG_INFO("loaded model {}.", onnxPath_);
}

void FsmHandStand::OnEnter()
{
    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
        LOG_WARN("Enter HandStand. No Implementation for linkV2_3_w, using walking model.");
    } else if (qrParam.model.contains(QrModel::linkV2_3) || qrParam.model.contains(QrModel::gazebo)) {
        LOG_INFO("Enter HandStand.");
    } else if (qrParam.model.contains(QrModel::middleV3)) {
        LOG_WARN("Enter HandStand. No Implementation for middleV3, using walking model.");
    } else {  // if (qrParam.model.contains(QrModel::middleV3_w)}
        LOG_ERROR("ERROR robot type!");
    }

    if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {  // 轮式机器人的变量初始化

        actionsOut_ = std::vector<float>(16, 0.0);
        lastActions_ = std::vector<float>(16, 0.0);
        sendActions_ = std::vector<float>(16, 0.0);
        lastSendActions_ = std::vector<float>(16, 0.0);

        /* ----获取切换至当前状态时的关节角数据---- */
        // TODO: ERROR maybe
        Mat34<double> tempVecFoot = jpData_->pFootForStand;
        EnterQ_ = mathFun_.InverseKinematics_new(tempVecFoot, qrParam.pRoll);
        for (int i = 0; i < 4; i++) {
            EnterQ_(2, i) *= -1;
            EnterQ_(1, i) -= 1.57;
            EnterQ_(1, i) *= -1;
        }

    } else {  // 足式机器人的变量初始化

        kSmooth = 0.0;
        actionsOut_ = std::vector<float>(12, 0.0);
        lastActions_ = std::vector<float>(12, 0.0);
        sendActions_ = std::vector<float>(12, 0.0);
        lastSendActions_ = std::vector<float>(12, 0.0);

        /* ----获取切换至当前状态时的关节角数据---- */
        Mat34<double> tempVecFoot = jpData_->pFootForStand;
        EnterQ_ = mathFun_.InverseKinematics_new(tempVecFoot, qrParam.pRoll);
        for (int i = 0; i < 4; i++) {
            EnterQ_(2, i) *= -1;
            EnterQ_(1, i) -= 1.57;
            EnterQ_(1, i) *= -1;
        }
        // TODO: 每次Enter时处理EnterQ的数据作为部分AI初始化数据, 可能的数据：lastSendActions_
    }
    done_ = true;
}

void FsmHandStand::OnExit()
{
    LOG_INFO("handstand exit");
    iRun_ = 0;
    interval_ = 0;
}

void FsmHandStand::Run()
{
    if (interval_ % 10 == 0) {
        if (qrParam.model.contains(QrModel::linkV2_3_w) || qrParam.model.contains(QrModel::gazebo_w)) {
            LeggedWheelRun();
        } else {
            LeggedRun();
        }
    }
    interval_++;
}

void FsmHandStand::LeggedRun()
{
    auto start = std::chrono::high_resolution_clock::now();

    /* ----获取指令并预处理---- */
    float vx = cmd_->GetLinear()(0);
    float vy = cmd_->GetLinear()(1);
    float wz = cmd_->GetAngular()(2);
    if (std::hypot(vx, vy) < 0.2) {
        vx = 0;
        vy = 0;
    }
    vx = 2.0 * vx;  // 为了解决AI训练时平衡观测值添加的系数，使用用户可以1：1收到指令
    vy = -2.0 * vy;
    wz = 0.25 * wz;

    /* ----获取imu数据并预处理---- */
    auto imu = rxData_->GetImuData();
    auto tImu = transXYZ(imu.quat(1), imu.quat(2), imu.quat(3), imu.quat(0));

    /* ----获取电机返回数据---- */
    DataCmd motorRet = rxData_->GetMotorDataMit();

    /* ----神经网络前向传播---- */
    NetWorkResult(tImu, vx, vy, wz, motorRet);

    /* ----设置PID---- */
    DataCmd motorCmd = SetLeggedPID();

    /* ----模式切换时的平滑过渡---- */
    if (kSmooth <= 1.0) {
        motorCmd.alpha = EnterQ_ + kSmooth * (motorCmd.alpha - EnterQ_);
        kSmooth += 0.1;
        // motorCmdOld.alpha = motorCmd.alpha;
    } else {
        // motorCmdOld.alpha = motorCmd.alpha;
    }

    /* ----相关迭代数据记录---- */
    for (int i = 0; i < 12; i++) {  // 刷新上一次的sendActions_的记录值
        lastSendActions_[i] = sendActions_[i];
    }

    /* ----下发PID电机命令---- */
    aiSend_.alpha = motorCmd.alpha;
    txData_->SendMotorCmd2(aiSend_);

    /* ----计时结束---- */
    if (printInfo_) {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        LOG_INFO("Onnx Running cost: {} us", duration.count());
    }
}

void FsmHandStand::LeggedWheelRun()
{
    auto start = std::chrono::high_resolution_clock::now();

    /* ----获取指令并预处理---- */
    float vx = cmd_->GetLinear()(0);
    float vy = cmd_->GetLinear()(1);
    float wz = cmd_->GetAngular()(2);
    if (std::hypot(vx, vy) < 0.2) {
        vx = 0;
        vy = 0;
    }
    vx = 2.0 * vx;  // 为了解决AI训练时平衡观测值添加的系数，使用用户可以1：1收到指令
    vy = -2.0 * vy;
    wz = 0.25 * wz;

    /* ----获取imu数据并预处理---- */
    auto imu = rxData_->GetImuData();
    auto tImu = transXYZ(imu.quat(1), imu.quat(2), imu.quat(3), imu.quat(0));

    /* ----获取电机返回数据---- */
    CmdVal4 motorRet = rxData_->GetMotorDataWheelMit();

    /* ----神经网络前向传播---- */
    NetWorkResult(tImu, vx, vy, wz, motorRet);

    /* ----设置PID---- */
    CmdVal4 motorCmdWheel = SetLeggedWheelPID();

    /* ----模式切换时的平滑过渡---- */
    if (kSmooth <= 1.0) {
        motorCmdWheel.alpha.block(0, 0, 3, 4) = EnterQ_ + kSmooth * (motorCmdWheel.alpha.block(0, 0, 3, 4) - EnterQ_);
        kSmooth += 0.1;
        motorCmdWheelOld.alpha = motorCmdWheel.alpha;
    } else {
        motorCmdWheelOld.alpha = motorCmdWheel.alpha;
    }

    /* ----下发PID电机命令---- */
    wheelAiSend_.alpha = motorCmdWheel.alpha;
    wheelAiSend_.torq = motorCmdWheel.torq;
    txData_->SendMotorCmdFixed(wheelAiSend_);

    /* ----计时结束---- */
    if (printInfo_) {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        LOG_INFO("Onnx Running cost: {} us", duration.count());
    }
}

void FsmHandStand::NetWorkResult(std::vector<double> imu, float vx, float vy, float wz, DataCmd motorRet)
{
    std::vector<const char*> output_node_names = {"output"};
    std::vector<const char*> input_node_names = {"input"};
    std::vector<float> out_cmd_val(input_tensor_size_);
    std::vector<int64_t> input_node_dims = {1, input_tensor_size_};

    // 存放42个维度的数据
    out_cmd_val[0] = imu[0];
    out_cmd_val[1] = imu[1];
    out_cmd_val[2] = imu[2];

    vxTemp = 0.95 * vxTemp + 0.05 * vx;  // TODO: 能否直接使用vx_, vy_, wz_ ?
    vyTemp = 0.97 * vyTemp + 0.03 * vy;
    wzTemp = 0.8 * wzTemp + 0.2 * wz;

    out_cmd_val[3] = vxTemp;
    out_cmd_val[4] = vyTemp;
    out_cmd_val[5] = wzTemp;

    out_cmd_val[6] = motorRet.alpha(0, 1) - defaultJoint_[0];  //      - defaultJoint_[0];
    out_cmd_val[7] = motorRet.alpha(1, 1) - defaultJoint_[1];  //      - defaultJoint_[1];
    out_cmd_val[8] = motorRet.alpha(2, 1) - defaultJoint_[2];  //      - defaultJoint_[2];

    out_cmd_val[9] = motorRet.alpha(0, 0) - defaultJoint_[3];   //      - defaultJoint_[0];
    out_cmd_val[10] = motorRet.alpha(1, 0) - defaultJoint_[4];  //      - defaultJoint_[1];
    out_cmd_val[11] = motorRet.alpha(2, 0) - defaultJoint_[5];  //      - defaultJoint_[2];

    out_cmd_val[12] = motorRet.alpha(0, 3) - defaultJoint_[6];  //      - defaultJoint_[0];
    out_cmd_val[13] = motorRet.alpha(1, 3) - defaultJoint_[7];  //      - defaultJoint_[1];
    out_cmd_val[14] = motorRet.alpha(2, 3) - defaultJoint_[8];  //      - defaultJoint_[2];

    out_cmd_val[15] = motorRet.alpha(0, 2) - defaultJoint_[9];   //      - defaultJoint_[0];
    out_cmd_val[16] = motorRet.alpha(1, 2) - defaultJoint_[10];  //      - defaultJoint_[1];
    out_cmd_val[17] = motorRet.alpha(2, 2) - defaultJoint_[11];  //      - defaultJoint_[2];

    // qd
    motorRet.torq = motorRet.torq * 0.05;
    out_cmd_val[18] = motorRet.torq(0, 1);
    out_cmd_val[19] = motorRet.torq(1, 1);
    out_cmd_val[20] = motorRet.torq(2, 1);

    out_cmd_val[21] = motorRet.torq(0, 0);
    out_cmd_val[22] = motorRet.torq(1, 0);
    out_cmd_val[23] = motorRet.torq(2, 0);

    out_cmd_val[24] = motorRet.torq(0, 3);
    out_cmd_val[25] = motorRet.torq(1, 3);
    out_cmd_val[26] = motorRet.torq(2, 3);

    out_cmd_val[27] = motorRet.torq(0, 2);
    out_cmd_val[28] = motorRet.torq(1, 2);
    out_cmd_val[29] = motorRet.torq(2, 2);

    // 最后12个维度为上一次AI计算出的动作
    for (int i = 0; i < 12; i++) {
        out_cmd_val[i + 30] = actionsOut_[i];
    }

    // create input tensor object from data values
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, out_cmd_val.data(), input_tensor_size_, input_node_dims.data(), 2);
    assert(input_tensor.IsTensor());

    // cout << "name " << input_node_names_[0] << endl;
    // score model & input tensor, get back output tensor
    auto val_session = session_->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names.data(), 1);
    assert(val_session.size() == 1 && val_session.front().IsTensor());

    // Get pointer to output tensor float values
    float* floatarr = val_session.front().GetTensorMutableData<float>();

    // print result
    for (int i = 0; i < 12; i++) {
        actionsOut_[i] = floatarr[i];
        // std::cout << "Score for class [" << i << "] =  " << floatarr[i] << '\n';
    }

    if (printInfo_) PrintNetworkResult(out_cmd_val);
}

void FsmHandStand::NetWorkResult(std::vector<double> imu, float vx, float vy, float wz, CmdVal4 wheelMotorRet)
{
    std::vector<const char*> output_node_names = {"output"};
    std::vector<const char*> input_node_names = {"input"};
    std::vector<float> out_cmd_val(input_tensor_size_);
    std::vector<int64_t> input_node_dims = {1, input_tensor_size_};

    // 存放54个维度的数据
    out_cmd_val[0] = std::clamp(imu[0], -100.0, 100.0);
    out_cmd_val[1] = std::clamp(imu[1], -100.0, 100.0);
    out_cmd_val[2] = std::clamp(imu[2], -100.0, 100.0);

    vxTemp = 0.95 * vxTemp + 0.05 * vx;
    vyTemp = 0.97 * vyTemp + 0.03 * vy;
    wzTemp = 0.8 * wzTemp + 0.2 * wz;

    out_cmd_val[3] = std::clamp(vxTemp, -100.0, 100.0);
    out_cmd_val[4] = std::clamp(vyTemp, -100.0, 100.0);
    out_cmd_val[5] = std::clamp(wzTemp, -100.0, 100.0);

    out_cmd_val[6] = std::clamp(wheelMotorRet.alpha(0, 1) - defaultJoint_[0], -100.0, 100.0);
    out_cmd_val[7] = std::clamp(wheelMotorRet.alpha(1, 1) - defaultJoint_[1], -100.0, 100.0);
    out_cmd_val[8] = std::clamp(wheelMotorRet.alpha(2, 1) - defaultJoint_[2], -100.0, 100.0);

    out_cmd_val[9] = std::clamp(wheelMotorRet.alpha(0, 0) - defaultJoint_[4], -100.0, 100.0);
    out_cmd_val[10] = std::clamp(wheelMotorRet.alpha(1, 0) - defaultJoint_[5], -100.0, 100.0);
    out_cmd_val[11] = std::clamp(wheelMotorRet.alpha(2, 0) - defaultJoint_[6], -100.0, 100.0);

    out_cmd_val[12] = std::clamp(wheelMotorRet.alpha(0, 3) - defaultJoint_[8], -100.0, 100.0);
    out_cmd_val[13] = std::clamp(wheelMotorRet.alpha(1, 3) - defaultJoint_[9], -100.0, 100.0);
    out_cmd_val[14] = std::clamp(wheelMotorRet.alpha(2, 3) - defaultJoint_[10], -100.0, 100.0);

    out_cmd_val[15] = std::clamp(wheelMotorRet.alpha(0, 2) - defaultJoint_[12], -100.0, 100.0);
    out_cmd_val[16] = std::clamp(wheelMotorRet.alpha(1, 2) - defaultJoint_[13], -100.0, 100.0);
    out_cmd_val[17] = std::clamp(wheelMotorRet.alpha(2, 2) - defaultJoint_[14], -100.0, 100.0);

    // qd
    wheelMotorRet.torq = wheelMotorRet.torq * 0.05;
    out_cmd_val[18] = std::clamp(wheelMotorRet.torq(0, 1), -100.0, 100.0);
    out_cmd_val[19] = std::clamp(wheelMotorRet.torq(1, 1), -100.0, 100.0);
    out_cmd_val[20] = std::clamp(wheelMotorRet.torq(2, 1), -100.0, 100.0);
    out_cmd_val[21] = std::clamp(wheelMotorRet.torq(3, 1), -100.0, 100.0);

    out_cmd_val[22] = std::clamp(wheelMotorRet.torq(0, 0), -100.0, 100.0);
    out_cmd_val[23] = std::clamp(wheelMotorRet.torq(1, 0), -100.0, 100.0);
    out_cmd_val[24] = std::clamp(wheelMotorRet.torq(2, 0), -100.0, 100.0);
    out_cmd_val[25] = std::clamp(wheelMotorRet.torq(3, 0), -100.0, 100.0);

    out_cmd_val[26] = std::clamp(wheelMotorRet.torq(0, 3), -100.0, 100.0);
    out_cmd_val[27] = std::clamp(wheelMotorRet.torq(1, 3), -100.0, 100.0);
    out_cmd_val[28] = std::clamp(wheelMotorRet.torq(2, 3), -100.0, 100.0);
    out_cmd_val[29] = std::clamp(wheelMotorRet.torq(3, 3), -100.0, 100.0);

    out_cmd_val[30] = std::clamp(wheelMotorRet.torq(0, 2), -100.0, 100.0);
    out_cmd_val[31] = std::clamp(wheelMotorRet.torq(1, 2), -100.0, 100.0);
    out_cmd_val[32] = std::clamp(wheelMotorRet.torq(2, 2), -100.0, 100.0);
    out_cmd_val[33] = std::clamp(wheelMotorRet.torq(3, 2), -100.0, 100.0);

    // 最后12个维度为上一次AI计算出的动作
    for (int i = 0; i < 16; i++) {
        out_cmd_val[i + 34] = std::clamp(actionsOut_[i], float(-100.0), float(100.0));
    }

    // create input tensor object from data values
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input_tensor = Ort::Value::CreateTensor<float>(memory_info, out_cmd_val.data(), input_tensor_size_, input_node_dims.data(), 2);
    assert(input_tensor.IsTensor());

    // score model & input tensor, get back output tensor
    auto val_session = session_->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names.data(), 1);
    assert(val_session.size() == 1 && val_session.front().IsTensor());

    // Get pointer to output tensor float values
    float* floatarr = val_session.front().GetTensorMutableData<float>();

    // print result
    for (int i = 0; i < 16; i++) {
        actionsOut_[i] = std::clamp(floatarr[i], float(-100.0), float(100.0));
    }

    if (printInfo_) PrintNetworkResult(out_cmd_val);
}

DataCmd FsmHandStand::SetLeggedPID()
{
    // 生成下发指令的数据结构
    DataCmd motorCmd;

    // 对下发关节指令进行滤波
    sendActions_[0] = kFilter_ * (actionsOut_[0] * aiActionScale_ + defaultJoint_[0]) + (1 - kFilter_) * lastSendActions_[0];
    sendActions_[1] = kFilter_ * (actionsOut_[1] * aiActionScale_ + defaultJoint_[1]) + (1 - kFilter_) * lastSendActions_[1];
    sendActions_[2] = kFilter_ * (actionsOut_[2] * aiActionScale_ + defaultJoint_[2]) + (1 - kFilter_) * lastSendActions_[2];
    sendActions_[3] = kFilter_ * (actionsOut_[3] * aiActionScale_ + defaultJoint_[3]) + (1 - kFilter_) * lastSendActions_[3];
    sendActions_[4] = kFilter_ * (actionsOut_[4] * aiActionScale_ + defaultJoint_[4]) + (1 - kFilter_) * lastSendActions_[4];
    sendActions_[5] = kFilter_ * (actionsOut_[5] * aiActionScale_ + defaultJoint_[5]) + (1 - kFilter_) * lastSendActions_[5];
    sendActions_[6] = kFilter_ * (actionsOut_[6] * aiActionScale_ + defaultJoint_[6]) + (1 - kFilter_) * lastSendActions_[6];
    sendActions_[7] = kFilter_ * (actionsOut_[7] * aiActionScale_ + defaultJoint_[7]) + (1 - kFilter_) * lastSendActions_[7];
    sendActions_[8] = kFilter_ * (actionsOut_[8] * aiActionScale_ + defaultJoint_[8]) + (1 - kFilter_) * lastSendActions_[8];
    sendActions_[9] = kFilter_ * (actionsOut_[9] * aiActionScale_ + defaultJoint_[9]) + (1 - kFilter_) * lastSendActions_[9];
    sendActions_[10] = kFilter_ * (actionsOut_[10] * aiActionScale_ + defaultJoint_[10]) + (1 - kFilter_) * lastSendActions_[10];
    sendActions_[11] = kFilter_ * (actionsOut_[11] * aiActionScale_ + defaultJoint_[11]) + (1 - kFilter_) * lastSendActions_[11];

    motorCmd.alpha(0, 1) = sendActions_[0];
    motorCmd.alpha(1, 1) = sendActions_[1];
    motorCmd.alpha(2, 1) = sendActions_[2];

    motorCmd.alpha(0, 0) = sendActions_[3];
    motorCmd.alpha(1, 0) = sendActions_[4];
    motorCmd.alpha(2, 0) = sendActions_[5];

    motorCmd.alpha(0, 3) = sendActions_[6];
    motorCmd.alpha(1, 3) = sendActions_[7];
    motorCmd.alpha(2, 3) = sendActions_[8];

    motorCmd.alpha(0, 2) = sendActions_[9];
    motorCmd.alpha(1, 2) = sendActions_[10];
    motorCmd.alpha(2, 2) = sendActions_[11];

    return motorCmd;
}

CmdVal4 FsmHandStand::SetLeggedWheelPID()
{
    CmdVal4 motorCmdWheel;

    motorCmdWheel.alpha(0, 1) = actionsOut_[0] * aiActionScale_ + defaultJoint_[0];
    motorCmdWheel.alpha(1, 1) = actionsOut_[1] * aiActionScale_ + defaultJoint_[1];
    motorCmdWheel.alpha(2, 1) = actionsOut_[2] * aiActionScale_ + defaultJoint_[2];

    motorCmdWheel.alpha(0, 0) = actionsOut_[4] * aiActionScale_ + defaultJoint_[4];
    motorCmdWheel.alpha(1, 0) = actionsOut_[5] * aiActionScale_ + defaultJoint_[5];
    motorCmdWheel.alpha(2, 0) = actionsOut_[6] * aiActionScale_ + defaultJoint_[6];

    motorCmdWheel.alpha(0, 3) = actionsOut_[8] * aiActionScale_ + defaultJoint_[8];
    motorCmdWheel.alpha(1, 3) = actionsOut_[9] * aiActionScale_ + defaultJoint_[9];
    motorCmdWheel.alpha(2, 3) = actionsOut_[10] * aiActionScale_ + defaultJoint_[10];

    motorCmdWheel.alpha(0, 2) = actionsOut_[12] * aiActionScale_ + defaultJoint_[12];
    motorCmdWheel.alpha(1, 2) = actionsOut_[13] * aiActionScale_ + defaultJoint_[13];
    motorCmdWheel.alpha(2, 2) = actionsOut_[14] * aiActionScale_ + defaultJoint_[14];

    motorCmdWheel.alpha(3, 1) = 0;
    motorCmdWheel.alpha(3, 0) = 0;
    motorCmdWheel.alpha(3, 3) = 0;
    motorCmdWheel.alpha(3, 2) = 0;

    // TODO: read from config files '*5'
    motorCmdWheel.torq(3, 1) = actionsOut_[3] * 5;
    motorCmdWheel.torq(3, 0) = actionsOut_[7] * 5;
    motorCmdWheel.torq(3, 3) = actionsOut_[11] * 5;
    motorCmdWheel.torq(3, 2) = actionsOut_[15] * 5;

    return motorCmdWheel;
}

std::vector<double> FsmHandStand::transXYZ(double x, double y, double z, double w)
{
    double pgx = 2.0 * y * w - 2.0 * z * x;
    double pgy = -2.0 * x * w - 2.0 * z * y;
    double pgz = 1.0 - 2.0 * w * w - 2.0 * z * z;
    std::vector<double> proj_g = {pgx, pgy, pgz};
    return proj_g;
}

void FsmHandStand::PrintNetworkResult(std::vector<float> out_cmd_val)
{
    if (!isWheel_) {
        std::stringstream inputInfo;
        inputInfo << std::fixed << std::setprecision(4);
        inputInfo << "\nGravity Vector: \n  ";
        for (int i = 0; i < 3; i++) {
            inputInfo << std::to_string(out_cmd_val[i]) + "\t";
        }
        inputInfo << "\n\nInput Command: \n  ";
        for (int i = 3; i < 6; i++) {
            inputInfo << std::to_string(out_cmd_val[i]) + "\t";
        }
        inputInfo << "\n\nJoint Postion: \n  ";
        for (int i = 6; i < 18; i++) {
            inputInfo << std::to_string(out_cmd_val[i]) + "\t";
            if (i % 3 == 2) inputInfo << "\n  ";
        }
        inputInfo << "\nJoint Velocity: \n  ";
        for (int i = 18; i < 30; i++) {
            inputInfo << std::to_string(out_cmd_val[i]) + "\t";
            if (i % 3 == 2) inputInfo << "\n  ";
        }
        inputInfo << "\nLast Actions Output: \n  ";
        for (int i = 30; i < 42; i++) {
            inputInfo << std::to_string(out_cmd_val[i]) + "\t";
            if (i % 3 == 2) inputInfo << "\n  ";
        }

        std::stringstream outputInfo;
        outputInfo << std::fixed << std::setprecision(4);
        outputInfo << "\nActions Output: \n  ";
        for (int i = 0; i < 12; i++) {
            outputInfo << std::to_string(actionsOut_[i]) + "\t";
            if (i % 3 == 2) outputInfo << "\n  ";
        }

        LOG_INFO(inputInfo.str());
        LOG_INFO(outputInfo.str());
    } else {
        std::stringstream inputInfo;
        inputInfo << std::fixed << std::setprecision(4);
        inputInfo << "\nGravity Vector: \n  ";
        for (int i = 0; i < 3; i++) {
            inputInfo << std::to_string(out_cmd_val[i]) + "\t";
        }
        inputInfo << "\n\nInput Command: \n  ";
        for (int i = 3; i < 6; i++) {
            inputInfo << std::to_string(out_cmd_val[i]) + "\t";
        }
        inputInfo << "\n\nJoint Postion: \n  ";
        for (int i = 6; i < 18; i++) {
            inputInfo << std::to_string(out_cmd_val[i]) + "\t";
            if (i % 3 == 2) inputInfo << "\n  ";
        }
        inputInfo << "\nJoint Velocity: \n  ";
        for (int i = 18; i < 34; i++) {
            inputInfo << std::to_string(out_cmd_val[i]) + "\t";
            if (i % 4 == 1) inputInfo << "\n  ";
        }
        inputInfo << "\nLast Actions Output: \n  ";
        for (int i = 34; i < 50; i++) {
            inputInfo << std::to_string(out_cmd_val[i]) + "\t";
            if (i % 4 == 1) inputInfo << "\n  ";
        }

        std::stringstream outputInfo;
        outputInfo << std::fixed << std::setprecision(4);
        outputInfo << "\nActions Output: \n  ";
        for (int i = 0; i < 16; i++) {
            outputInfo << std::to_string(actionsOut_[i]) + "\t";
            if (i % 4 == 3) outputInfo << "\n  ";
        }

        LOG_INFO(inputInfo.str());
        LOG_INFO(outputInfo.str());
    }
}