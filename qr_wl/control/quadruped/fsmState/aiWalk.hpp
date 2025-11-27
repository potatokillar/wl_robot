
#pragma once
#include <onnx/onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <thread>

#include "baseline.hpp"
#include "contrType.hpp"
#include "imuData.hpp"
#include "innerType.hpp"
#include "jpCtrlData.hpp"
#include "mathTools.hpp"

class AiWalk
{
public:
    AiWalk();
    ~AiWalk();
    void Start();
    void Start(WalkMode mode);
    void Run();
    void Stop();

    void SetStandQ(const Mat34<double>& standQ);
    void SetTsrV3(const ImuData& imu);
    void SetCxdV3(float vx, float vy, float wz);
    void SetCxdV2(const DataCmd& ret);
    void SetCxdV2(const CmdVal4& ret);  // todo需要实现

    CmdVal2 ret2flagX() const;
    DataValue2 t2flagY() const;  // todo 需要实现

private:
    std::vector<double> transXYZ(double x, double y, double z, double w);
    void NetWorkResult(std::vector<double> imu, float vx, float vy, float wz, DataCmd motorRet);
    void NetWorkResult(std::vector<double> imu, float vx, float vy, float wz, CmdVal4 motorRet);
    DataCmd ReceiceForSend();
    CmdVal4 t1Flag3();
    double GetPhase();
    double GetRefAirRatio(double vx, double wz);

    // 线程相关变量
    std::unique_ptr<PeriodicMemberFunction<AiWalk>> task_;

    std::unique_ptr<Ort::Session> session_;
    int64_t input_tensor_size_;

    std::vector<double> defaultJoint_;
    double aiActionScale_{0.25};
    double kmodify_{0.3};

    ImuData imu_;
    float vx_{0}, vy_{0}, wz_{0};
    std::vector<float> actionsOut_;
    std::vector<float> lastActions_;
    std::vector<float> sendActions_;
    std::vector<float> lastSendActions_;

    DataCmd motorRet_;
    DataCmd motorCmdOld;
    CmdVal2 aiSend_;
    Mat34<double> standQ_;
    Mat34<double> qStandForAI0, qStandForAI1, qStandForAI2;

    CmdVal4 wheelMotorRet_;
    CmdVal4 wheelMotorCmdOld;
    DataValue2 wheelAiSend_;
    Mat44<double> wheelStandQ_;

    // 滤波操作相关变量
    int N = 3;
    int iRun = 0;
    double kSmooth = 0.0;
    double vxTemp = 0;
    double vyTemp = 0;
    double wzTemp = 0;

    // 其他
    bool isWheel_;
    WalkMode walkMode_;
    bool printInfo_{false};
    const QuadrupedParam& qrParam{GetQrParam()};
    MathFun mathFun_;
};