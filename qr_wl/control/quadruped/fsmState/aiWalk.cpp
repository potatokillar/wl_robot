#include "aiWalk.hpp"

#include "baseline.hpp"
#include "quadrupedParam.hpp"

using namespace std;

/**
 * @description: 构造函数，程序中只会启用一次
 * @return {}
 */
AiWalk::AiWalk()
{
    isWheel_ = (GetQrParam().model.contains(QrModel::gazebo_w) || GetQrParam().model.contains(QrModel::linkV2_3_w));
    int joint_num = isWheel_ ? 16 : 12;
    input_tensor_size_ = isWheel_ ? 1 * 52 : 1 * 42;

    //  std::string onnxPath_;
    // defaultJoint_ = std::vector<double>{0.0, 0.9, -1.8};
    defaultJoint_ = std::vector<double>(joint_num, 0.0);
    actionsOut_ = std::vector<float>(joint_num, 0.0);
    lastActions_ = std::vector<float>(joint_num, 0.0);
    sendActions_ = std::vector<float>(joint_num, 0.0);
    lastSendActions_ = std::vector<float>(joint_num, 0.0);

    wheelMotorRet_.alpha << 0.0, -0.0, 0.0, -0.0, 0.9, 0.9, 0.9, 0.9, -1.8, -1.8, -1.8, -1.8, 0.0, 0.0, 0.0, 0.0;
    wheelMotorRet_.torq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    auto enable = GetRobotConfigDirect<bool>("enableParam").value();
    if (enable) {
        printInfo_ = GetRobotConfigDirect<bool>("printInfo").value();
        aiActionScale_ = GetRobotConfigDirect<double>("aiActionScale").value();
        kmodify_ = GetRobotConfigDirect<double>("kFilter").value();
        // aiSend_.k2.fill(GetRobotConfigDirect<double>("kp").value());
        // aiSend_.k1.fill(GetRobotConfigDirect<double>("kd").value());
        // kmodify_ = GetRobotConfigDirect<double>("kFilter").value();
        // defaultJoint_[0] = GetRobotConfigDirect<double>("defaultHip").value();
        // defaultJoint_[1] = GetRobotConfigDirect<double>("defaultThigh").value();
        // defaultJoint_[2] = GetRobotConfigDirect<double>("defaultCalf").value();
        // aiSend_.k2.fill(GetRobotConfigDirect<double>("kp", "nml_clb").value());
        // aiSend_.k1.fill(GetRobotConfigDirect<double>("kd", "nml_clb").value());
    } else {
        // TODO: set default value when not enable toml
        //       set stable default values here, make them invisible to user.
    }

    task_ = make_unique<PeriodicMemberFunction<AiWalk>>("aiWalk", 0.02, this, &AiWalk::Run);
}
AiWalk::~AiWalk() { Stop(); }

void AiWalk::Start()
{
    if (kSmooth > 1.01) {
        kSmooth = 0.5;
    }

    iRun = 0;
    vxTemp = 0;
    vyTemp = 0;
    wzTemp = 0;

    if (!isWheel_) {
        lastSendActions_[0] = motorCmdOld.alpha(3);
        lastSendActions_[1] = motorCmdOld.alpha(4);
        lastSendActions_[2] = motorCmdOld.alpha(5);
        lastSendActions_[3] = motorCmdOld.alpha(0);
        lastSendActions_[4] = motorCmdOld.alpha(1);
        lastSendActions_[5] = motorCmdOld.alpha(2);
        lastSendActions_[6] = motorCmdOld.alpha(9);
        lastSendActions_[7] = motorCmdOld.alpha(10);
        lastSendActions_[8] = motorCmdOld.alpha(11);
        lastSendActions_[9] = motorCmdOld.alpha(6);
        lastSendActions_[10] = motorCmdOld.alpha(7);
        lastSendActions_[11] = motorCmdOld.alpha(8);
    } else {
        // TODO: read from toml file
        wheelMotorRet_.alpha << 0.0, -0.0, 0.0, -0.0, 0.9, 0.9, 0.9, 0.9, -1.8, -1.8, -1.8, -1.8, 0.0, 0.0, 0.0, 0.0;
        wheelMotorRet_.torq << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        actionsOut_ = std::vector<float>(16, 0.0);
    }
    for (int i = 0; i < 4; i++) {
        standQ_(2, i) *= -1;
        standQ_(1, i) -= 1.57;
        standQ_(1, i) *= -1;
    }
    aiSend_.alpha = standQ_;
    wheelAiSend_.alpha.block(0, 0, 3, 4) = standQ_;  // TODO: ERROR maybe
    task_->Start();
}

void AiWalk::Start(WalkMode mode)
{
    string onnxPath;
    walkMode_ = mode;
    if (mode == WalkMode::aiClimb) {
        onnxPath = GetRobotConfigDirect<std::string>("onnxPath", "climb").value();
        defaultJoint_ = GetRobotConfigDirect<vector<double>>("defaultJoints", "climb").value();
        kmodify_ = GetRobotConfigDirect<double>("kFilter", "climb").value();
        // aiSend_.k2.fill(GetRobotConfigDirect<double>("kp", "nml_clb").value());
        // aiSend_.k1.fill(GetRobotConfigDirect<double>("kd", "nml_clb").value());
    } else if (mode == WalkMode::aiNormal) {
        onnxPath = GetRobotConfigDirect<std::string>("onnxPath", "normal").value();
        defaultJoint_ = GetRobotConfigDirect<vector<double>>("defaultJoints", "normal").value();
        kmodify_ = GetRobotConfigDirect<double>("kFilter", "normal").value();
        // aiSend_.k2.fill(GetRobotConfigDirect<double>("kp", "nml_clb").value());
        // aiSend_.k1.fill(GetRobotConfigDirect<double>("kd", "nml_clb").value());
    } else {
        LOG_ERROR("no this onnx mode");
    }
    if (!isWheel_) {
        aiSend_.k2.fill(GetRobotConfigDirect<double>("kp", "nml_clb").value());
        aiSend_.k1.fill(GetRobotConfigDirect<double>("kd", "nml_clb").value());
    } else {
        std::vector<double> kp = GetRobotConfigDirect<std::vector<double>>("kp", "nml_clb").value();
        std::vector<double> kd = GetRobotConfigDirect<std::vector<double>>("kd", "nml_clb").value();
        wheelAiSend_.k2 = Mat44<double>(kp.data());  // 此法构造的kpkd矩阵为toml文件中所写格式的转置
        wheelAiSend_.k1 = Mat44<double>(kd.data());
    }

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test");
    // const auto& api = Ort::GetApi();
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(8);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    try {
        session_ = make_unique<Ort::Session>(env, onnxPath.c_str(), session_options);
    } catch (const std::exception& e) {
        LOG_ERROR("load model error: {}", e.what());
        return;
    }

    LOG_INFO("loaded model {}.", onnxPath);

    Start();
}

/**
 * @description: 停止AI算法
 * @return {}
 */
void AiWalk::Stop() { task_->Stop(); }

void AiWalk::Run()
{
    // time record start
    auto start = std::chrono::high_resolution_clock::now();

    // cout << "name11 " << input_node_names[0] << endl;
    // cout << "init end" << endl;
    // 预定义变量，待优化
    ImuData imu;
    float vx, vy, wz;
    DataCmd motorRet;
    CmdVal4 wheelMotorRet;

    imu = imu_;
    vx = vx_;
    // vy = vy_;
    vy = 0;
    if (GetQrParam().model.contains(QrModel::middleV3) == true) {
        if (vx_ >= 0) {
            wz = wz_ - 0.023 * fabs(vx_);
        } else {
            wz = wz_ - 0.03 * fabs(vx_);
        }
    }
    wz = wz_;
    if (std::hypot(vx, vy) < 0.2) {
        vx = 0;
        vy = 0;
    }

    vx = 2.5 * vx;
    vy = 2.5 * vy;
    wz = 0.3 * wz;

    if (!isWheel_) {
        motorRet = motorRet_;
    } else {
        wheelMotorRet = wheelMotorRet_;
    }

    // 0:3 是IMU数据
    auto tImu = transXYZ(imu.quat(1), imu.quat(2), imu.quat(3), imu.quat(0));

    if (!isWheel_) {
        NetWorkResult(tImu, vx, vy, wz, motorRet);
        DataCmd ratorCmd = ReceiceForSend();
        if (kSmooth <= 1.0) {
            ratorCmd.alpha = standQ_ + kSmooth * (ratorCmd.alpha - standQ_);
            kSmooth += 0.1;
            // cout << kSmooth << endl;
            motorCmdOld.alpha = ratorCmd.alpha;
        } else {
            motorCmdOld.alpha = ratorCmd.alpha;
        }

        aiSend_ = ratorCmd;

        for (int i = 0; i < 12; i++) {
            lastSendActions_[i] = sendActions_[i];
        }
        // TODO: maybe need filter for wheels in future
        motorCmdOld.alpha = ratorCmd.alpha;
        // cout << "enter legged !" << endl;
    } else {
        NetWorkResult(tImu, vx, vy, wz, wheelMotorRet);
        CmdVal4 cmd3 = t1Flag3();
        if (kSmooth <= 1.0) {
            cmd3.alpha.block(0, 0, 3, 4) = standQ_ + kSmooth * (cmd3.alpha.block(0, 0, 3, 4) - standQ_);
            kSmooth += 0.02;
            // cout << kSmooth << endl;
            wheelMotorCmdOld.alpha = cmd3.alpha;
        } else {
            wheelMotorCmdOld.alpha = cmd3.alpha;
        }

        wheelAiSend_ = cmd3;

        if (printInfo_) {
            std::ostringstream info_string;
            info_string << "\nq: \n" << wheelAiSend_.alpha.block(0, 0, 4, 4) << "\n";
            info_string << "qd: \n" << wheelAiSend_.torq.block(0, 0, 4, 4) << "\n";
            info_string << "kp: \n" << wheelAiSend_.k2.block(0, 0, 4, 4) << "\n";
            info_string << "kd: \n" << wheelAiSend_.k1.block(0, 0, 4, 4) << "\n";
            LOG_INFO(info_string.str());
        }
        // std::cout << "q: \n" << wheelAiSend_.alpha.block(0, 0, 3, 4) << "\n\n";
        // std::cout << "qd: \n" << wheelAiSend_.torq.row(3) << "\n\n";
        // std::cout << "tau: \n" << wheelAiSend_.blta << "\n\n";
    }

    if (printInfo_) {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        LOG_INFO("Onnx Running cost: {} us", duration.count());
    }
}

std::vector<double> AiWalk::transXYZ(double x, double y, double z, double w)
{
    // v = {0, 0, -1}
    // std::vector<double> a = {0.0, 0.0, 1.0 - 2.0 * w * w};
    // std::vector<double> b = {-2.0 * y * w, 2.0 * x * w, 0.0};
    // std::vector<double> c = {-2.0 * z * x, -2.0 * z * y, -2.0 * z * z};
    // std::vector<double> proj_g = {a[0] - b[0] + c[0], a[1] - b[1] + c[1], a[2] - b[2] + c[2]};

    double pgx = 2.0 * y * w - 2.0 * z * x;
    double pgy = -2.0 * x * w - 2.0 * z * y;
    double pgz = 1.0 - 2.0 * w * w - 2.0 * z * z;
    std::vector<double> proj_g = {pgx, pgy, pgz};
    return proj_g;
}

void AiWalk::SetTsrV3(const ImuData& imu) { imu_ = imu; }
void AiWalk::SetCxdV3(float vx, float vy, float wz)
{
    vx_ = vx;
    vy_ = vy;
    wz_ = wz;
}
void AiWalk::SetCxdV2(const DataCmd& ret) { motorRet_ = ret; }

void AiWalk::SetCxdV2(const CmdVal4& ret)
{
    //
    // transform progress ..
    wheelMotorRet_ = ret;
    // std::cout << "q: \n" << wheelMotorRet_.alpha << "\n\n";
    // std::cout << "qd: \n" << wheelMotorRet_.torq << "\n\n";
    // std::cout << "tau: \n" << wheelMotorRet_.blta << "\n\n";
    //     int a;
    //     std::cin >> a;
}

void AiWalk::SetStandQ(const Mat34<double>& standQ) { standQ_ = standQ; }

CmdVal2 AiWalk::ret2flagX() const { return aiSend_; }

DataValue2 AiWalk::t2flagY() const { return wheelAiSend_; }

void AiWalk::NetWorkResult(std::vector<double> imu, float vx, float vy, float wz, DataCmd motorRet)
{
    std::vector<float> out_cmd_val(input_tensor_size_);
    std::vector<const char*> input_node_names = {"input"};
    std::vector<const char*> output_node_names = {"output"};
    std::vector<int64_t> input_node_dims = {1, input_tensor_size_};

    out_cmd_val[0] = imu[0];
    out_cmd_val[1] = imu[1];
    out_cmd_val[2] = imu[2];

    vxTemp = 0.95 * vxTemp + 0.05 * vx;
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
}

void AiWalk::NetWorkResult(std::vector<double> imu, float vx, float vy, float wz, CmdVal4 wheelMotorRet)
{
    std::vector<float> out_cmd_val(input_tensor_size_);
    std::vector<const char*> input_node_names = {"input"};
    std::vector<int64_t> input_node_dims = {1, input_tensor_size_};
    std::vector<const char*> output_node_names = {"output"};

    vxTemp = 0.95 * vxTemp + 0.05 * vx;
    vyTemp = 0.97 * vyTemp + 0.03 * vy;
    wzTemp = 0.8 * wzTemp + 0.2 * wz;

    out_cmd_val[0] = GetPhase();
    out_cmd_val[1] = GetRefAirRatio(vxTemp, wzTemp);

    out_cmd_val[2] = std::clamp(imu[0], -100.0, 100.0);
    out_cmd_val[3] = std::clamp(imu[1], -100.0, 100.0);
    out_cmd_val[4] = std::clamp(imu[2], -100.0, 100.0);

    out_cmd_val[5] = std::clamp(vxTemp, -100.0, 100.0);
    out_cmd_val[6] = std::clamp(vyTemp, -100.0, 100.0);
    out_cmd_val[7] = std::clamp(wzTemp, -100.0, 100.0);

    out_cmd_val[8] = std::clamp(wheelMotorRet.alpha(0, 1) - defaultJoint_[0], -100.0, 100.0);
    out_cmd_val[9] = std::clamp(wheelMotorRet.alpha(1, 1) - defaultJoint_[1], -100.0, 100.0);
    out_cmd_val[10] = std::clamp(wheelMotorRet.alpha(2, 1) - defaultJoint_[2], -100.0, 100.0);

    out_cmd_val[11] = std::clamp(wheelMotorRet.alpha(0, 0) - defaultJoint_[4], -100.0, 100.0);
    out_cmd_val[12] = std::clamp(wheelMotorRet.alpha(1, 0) - defaultJoint_[5], -100.0, 100.0);
    out_cmd_val[13] = std::clamp(wheelMotorRet.alpha(2, 0) - defaultJoint_[6], -100.0, 100.0);

    out_cmd_val[14] = std::clamp(wheelMotorRet.alpha(0, 3) - defaultJoint_[8], -100.0, 100.0);
    out_cmd_val[15] = std::clamp(wheelMotorRet.alpha(1, 3) - defaultJoint_[9], -100.0, 100.0);
    out_cmd_val[16] = std::clamp(wheelMotorRet.alpha(2, 3) - defaultJoint_[10], -100.0, 100.0);

    out_cmd_val[17] = std::clamp(wheelMotorRet.alpha(0, 2) - defaultJoint_[12], -100.0, 100.0);
    out_cmd_val[18] = std::clamp(wheelMotorRet.alpha(1, 2) - defaultJoint_[13], -100.0, 100.0);
    out_cmd_val[19] = std::clamp(wheelMotorRet.alpha(2, 2) - defaultJoint_[14], -100.0, 100.0);

    // qd
    wheelMotorRet.torq = wheelMotorRet.torq * 0.05;
    out_cmd_val[20] = std::clamp(wheelMotorRet.torq(0, 1), -100.0, 100.0);
    out_cmd_val[21] = std::clamp(wheelMotorRet.torq(1, 1), -100.0, 100.0);
    out_cmd_val[22] = std::clamp(wheelMotorRet.torq(2, 1), -100.0, 100.0);
    out_cmd_val[23] = std::clamp(wheelMotorRet.torq(3, 1), -100.0, 100.0);

    out_cmd_val[24] = std::clamp(wheelMotorRet.torq(0, 0), -100.0, 100.0);
    out_cmd_val[25] = std::clamp(wheelMotorRet.torq(1, 0), -100.0, 100.0);
    out_cmd_val[26] = std::clamp(wheelMotorRet.torq(2, 0), -100.0, 100.0);
    out_cmd_val[27] = std::clamp(wheelMotorRet.torq(3, 0), -100.0, 100.0);

    out_cmd_val[28] = std::clamp(wheelMotorRet.torq(0, 3), -100.0, 100.0);
    out_cmd_val[29] = std::clamp(wheelMotorRet.torq(1, 3), -100.0, 100.0);
    out_cmd_val[30] = std::clamp(wheelMotorRet.torq(2, 3), -100.0, 100.0);
    out_cmd_val[31] = std::clamp(wheelMotorRet.torq(3, 3), -100.0, 100.0);

    out_cmd_val[32] = std::clamp(wheelMotorRet.torq(0, 2), -100.0, 100.0);
    out_cmd_val[33] = std::clamp(wheelMotorRet.torq(1, 2), -100.0, 100.0);
    out_cmd_val[34] = std::clamp(wheelMotorRet.torq(2, 2), -100.0, 100.0);
    out_cmd_val[35] = std::clamp(wheelMotorRet.torq(3, 2), -100.0, 100.0);

    for (int i = 0; i < 16; i++) {
        out_cmd_val[i + 36] = std::clamp(actionsOut_[i], float(-100.0), float(100.0));
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
        // std::cout << "Score for class [" << i << "] =  " << floatarr[i] << '\n';
    }
}

DataCmd AiWalk::ReceiceForSend()
{
    DataCmd ratorCmd;

    sendActions_[0] = kmodify_ * (actionsOut_[0] * aiActionScale_ + defaultJoint_[0]) + (1 - kmodify_) * lastSendActions_[0];
    sendActions_[1] = kmodify_ * (actionsOut_[1] * aiActionScale_ + defaultJoint_[1]) + (1 - kmodify_) * lastSendActions_[1];
    sendActions_[2] = kmodify_ * (actionsOut_[2] * aiActionScale_ + defaultJoint_[2]) + (1 - kmodify_) * lastSendActions_[2];
    sendActions_[3] = kmodify_ * (actionsOut_[3] * aiActionScale_ + defaultJoint_[3]) + (1 - kmodify_) * lastSendActions_[3];
    sendActions_[4] = kmodify_ * (actionsOut_[4] * aiActionScale_ + defaultJoint_[4]) + (1 - kmodify_) * lastSendActions_[4];
    sendActions_[5] = kmodify_ * (actionsOut_[5] * aiActionScale_ + defaultJoint_[5]) + (1 - kmodify_) * lastSendActions_[5];
    sendActions_[6] = kmodify_ * (actionsOut_[6] * aiActionScale_ + defaultJoint_[6]) + (1 - kmodify_) * lastSendActions_[6];
    sendActions_[7] = kmodify_ * (actionsOut_[7] * aiActionScale_ + defaultJoint_[7]) + (1 - kmodify_) * lastSendActions_[7];
    sendActions_[8] = kmodify_ * (actionsOut_[8] * aiActionScale_ + defaultJoint_[8]) + (1 - kmodify_) * lastSendActions_[8];
    sendActions_[9] = kmodify_ * (actionsOut_[9] * aiActionScale_ + defaultJoint_[9]) + (1 - kmodify_) * lastSendActions_[9];
    sendActions_[10] = kmodify_ * (actionsOut_[10] * aiActionScale_ + defaultJoint_[10]) + (1 - kmodify_) * lastSendActions_[10];
    sendActions_[11] = kmodify_ * (actionsOut_[11] * aiActionScale_ + defaultJoint_[11]) + (1 - kmodify_) * lastSendActions_[11];

    ratorCmd.alpha(0, 1) = sendActions_[0];
    ratorCmd.alpha(1, 1) = sendActions_[1];
    ratorCmd.alpha(2, 1) = sendActions_[2];

    ratorCmd.alpha(0, 0) = sendActions_[3];
    ratorCmd.alpha(1, 0) = sendActions_[4];
    ratorCmd.alpha(2, 0) = sendActions_[5];

    ratorCmd.alpha(0, 3) = sendActions_[6];
    ratorCmd.alpha(1, 3) = sendActions_[7];
    ratorCmd.alpha(2, 3) = sendActions_[8];

    ratorCmd.alpha(0, 2) = sendActions_[9];
    ratorCmd.alpha(1, 2) = sendActions_[10];
    ratorCmd.alpha(2, 2) = sendActions_[11];

    return ratorCmd;
}

CmdVal4 AiWalk::t1Flag3()
{
    CmdVal4 radRxdata;

    radRxdata.alpha(0, 1) = actionsOut_[0] * aiActionScale_ + defaultJoint_[0];
    radRxdata.alpha(1, 1) = actionsOut_[1] * aiActionScale_ + defaultJoint_[1];
    radRxdata.alpha(2, 1) = actionsOut_[2] * aiActionScale_ + defaultJoint_[2];

    radRxdata.alpha(0, 0) = actionsOut_[4] * aiActionScale_ + defaultJoint_[4];
    radRxdata.alpha(1, 0) = actionsOut_[5] * aiActionScale_ + defaultJoint_[5];
    radRxdata.alpha(2, 0) = actionsOut_[6] * aiActionScale_ + defaultJoint_[6];

    radRxdata.alpha(0, 3) = actionsOut_[8] * aiActionScale_ + defaultJoint_[8];
    radRxdata.alpha(1, 3) = actionsOut_[9] * aiActionScale_ + defaultJoint_[9];
    radRxdata.alpha(2, 3) = actionsOut_[10] * aiActionScale_ + defaultJoint_[10];

    radRxdata.alpha(0, 2) = actionsOut_[12] * aiActionScale_ + defaultJoint_[12];
    radRxdata.alpha(1, 2) = actionsOut_[13] * aiActionScale_ + defaultJoint_[13];
    radRxdata.alpha(2, 2) = actionsOut_[14] * aiActionScale_ + defaultJoint_[14];

    radRxdata.alpha(3, 1) = 0;
    radRxdata.alpha(3, 0) = 0;
    radRxdata.alpha(3, 3) = 0;
    radRxdata.alpha(3, 2) = 0;

    radRxdata.torq(3, 1) = actionsOut_[3] * 10;
    radRxdata.torq(3, 0) = actionsOut_[7] * 10;
    radRxdata.torq(3, 3) = actionsOut_[11] * 10;
    radRxdata.torq(3, 2) = actionsOut_[15] * 10;

    return radRxdata;
}

double AiWalk::GetPhase()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto time_s = std::chrono::duration<double>(duration).count();
    return sin(2.0 * M_PI * (time_s / 0.8));
}


double AiWalk::GetRefAirRatio(double vx, double wz)
{
    double vx_ratio = 1.0 - 2 * pow(fabs(vx / 1.0) - 0.5, 2);
    double wz_ratio = fabs(wz / 1.0);
    double air_ratio = vx_ratio * wz_ratio;
    return sin((1 - air_ratio) / 2 * 3.1415926);
}