
#pragma once
#include <memory>

#include "qrMotor.hpp"
#include "quadCmd.hpp"
#include "quadDataChange.hpp"
#include "quadKey.hpp"
#include "quadSdk.hpp"
#include "quadWeb.hpp"

class UserQuadNode : public Singleton<UserQuadNode>
{
public:
    UserQuadNode();
    ~UserQuadNode();

    void Loop() override;
    RetState SetCallback(const std::string& type, QrCallbackFunc func);

private:
    std::shared_ptr<QrMotor> GetRealMotor();
    std::shared_ptr<QrMotor> GetSimMotor();
    std::shared_ptr<QrMotor> GetRealMotorV3();

    std::shared_ptr<QrMotor> GetSimLegWheelMotor();  // 轮腿式
    std::shared_ptr<QrMotor> GetRealLegWheelMotor();

    std::any ZeroPosWriteComplete(const std::any& data);
    std::any JointErr(const std::any& anyData);

private:
    std::shared_ptr<QrMotor> motor_;
    std::unique_ptr<QuadSdk> sdk_;
    std::unique_ptr<QuadKey> key_;
    std::unique_ptr<QuadCmd> cmd_;
    std::unique_ptr<QuadDataChange> dataChange_;
    std::unique_ptr<QuadWeb> web_;
};

inline UserQuadNode& GetQuadNode() { return UserQuadNode::GetInstance(); }