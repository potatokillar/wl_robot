
#pragma once

#include <memory>

#include "armKey.hpp"
#include "armSdk.hpp"
#include "driver.hpp"

class UserArmNode : public Singleton<UserArmNode>
{
public:
    UserArmNode();
    ~UserArmNode();

    void Loop() override;

private:
    std::unique_ptr<ArmSdk> sdk_;
    std::unique_ptr<ArmKey> key_;
};

inline UserArmNode& GetArmNode() { return UserArmNode::GetInstance(); }