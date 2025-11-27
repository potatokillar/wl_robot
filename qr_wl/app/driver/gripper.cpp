#include "gripper.hpp"

#include "deviceParam.hpp"

void ControlGripperAPI::GetGripperPowerOff()
{
    uint8_t buff[10];
    buff[2] = 0xF0;

    GripperData can(buff);
    canProtocol_.Send(can);
}
/**
 * @brief 夹爪自动关闭
 *
 * @param speed 范围0-1，不包括0
 */
void ControlGripperAPI::AotuGetGripperOff(double speed)
{
    uint8_t buff[10];
    u8 setSpeed;
    setSpeed = (u8)(speed * 100);

    buff[0] = 0;
    buff[1] = 0;
    buff[2] = 0xF0;
    buff[3] = 2;
    buff[4] = 0xAA;
    buff[5] = setSpeed;
    buff[6] = 0;
    buff[7] = 0;
    buff[8] = 0;
    buff[9] = 0;
    GripperData can(buff);
    canProtocol_.Send(can);
}

void ControlGripperAPI::AotuGetGripperOn(double speed)
{
    uint8_t buff[10];
    u8 setSpeed;
    setSpeed = (u8)(speed * 100);

    buff[0] = 0;
    buff[1] = 0;
    buff[2] = 0xF0;

    buff[3] = 2;
    buff[4] = 0x55;
    buff[5] = setSpeed;

    GripperData can(buff);
    canProtocol_.Send(can);
}

void ControlGripperAPI::PositionPercentageNew(uint8_t percentage, double speed)
{
    // uint16_t temp;
    if (percentage < 1) {
        percentage = 1;
    }
    if (percentage > 99) {
        percentage = 99;
    }

    Gripper01New(percentage, speed);
}

bool ControlGripperAPI::Gripper01New(u8 percentage, double speed)
{
    uint8_t buff[10];
    if (percentage <= 100) {
        u8 setSpeed;
        setSpeed = (u8)(speed * 100);
        buff[0] = 0;
        buff[1] = 0;
        buff[2] = 0xF0;
        buff[3] = 1;
        buff[4] = percentage;
        buff[5] = setSpeed;

        GripperData can(buff);

        canProtocol_.Send(can);

        return true;
    } else {
        return false;
    }
}
