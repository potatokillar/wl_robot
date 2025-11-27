
#pragma once
#include <map>
#include <string>

struct MediaFile
{
    const std::string name;
    const std::string zh_cn;
    const std::string en_us;
};

const MediaFile mediaFile[] = {
    {"lowBattery", "resource/audio/zh_cn/lowBattery.wav", "resource/audio/en_us/"},
    {"lowBatteryToShutDown", "resource/audio/zh_cn/lowBatteryToShutDown.wav", "resource/audio/en_us/"},
    {"readyToStart", "resource/audio/zh_cn/readyToStart.wav", "resource/audio/en_us/"},
    {"sensorError", "resource/audio/zh_cn/sensorError.wav", "resource/audio/en_us/"},
    {"shutDown", "resource/audio/zh_cn/shutDown.wav", "resource/audio/en_us/"},
    {"sleeping", "resource/audio/zh_cn/sleeping.wav", "resource/audio/en_us/"},
    {"errorToReboot", "resource/audio/zh_cn/errorToReboot.wav", "resource/audio/en_us/"},
    {"jointError", "resource/audio/zh_cn/jointError.wav", "resource/audio/en_us/"},
    {"pleaseConnectJoystick", "resource/audio/zh_cn/pleaseConnectJoystick.wav", "resource/audio/en_us/"},
    {"paramWrite", "resource/audio/zh_cn/paramWrite.wav", "resource/audio/en_us/"},
    {"setLoadMass", "resource/audio/zh_cn/setLoadMass.wav", "resource/audio/en_us/"},
    {"kg", "resource/audio/zh_cn/unitKg.wav", "resource/audio/en_us/"},
    {"0", "resource/audio/zh_cn/0.wav", "resource/audio/en_us/"},
    {"1", "resource/audio/zh_cn/1.wav", "resource/audio/en_us/"},
    {"2", "resource/audio/zh_cn/2.wav", "resource/audio/en_us/"},
    {"3", "resource/audio/zh_cn/3.wav", "resource/audio/en_us/"},
    {"4", "resource/audio/zh_cn/4.wav", "resource/audio/en_us/"},
    {"5", "resource/audio/zh_cn/5.wav", "resource/audio/en_us/"},
    {"6", "resource/audio/zh_cn/6.wav", "resource/audio/en_us/"},
    {"7", "resource/audio/zh_cn/7.wav", "resource/audio/en_us/"},
    {"8", "resource/audio/zh_cn/8.wav", "resource/audio/en_us/"},
    {"9", "resource/audio/zh_cn/9.wav", "resource/audio/en_us/"},
    {"point", "resource/audio/zh_cn/point.wav", "resource/audio/en_us/"},
};