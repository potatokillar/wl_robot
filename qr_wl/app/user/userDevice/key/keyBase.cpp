
#include "keyBase.hpp"

#include "apiDevice.hpp"
#include "gamepad.hpp"
using namespace std;

std::set<std::string> key_just_pressed = {};  // 暂存由哪2个键构成多键的容器，用于解决交叉触发的bug

/**
 * @description: 设置键值
 * @param press
 * @return {}
 */
void KeyRecord::SetValue(bool press)
{
    // 触发按下
    if ((isPress_ == false) && (press == true)) {
        isTriggerPress_ = true;
        isTriggerRelease_ = false;
    } else if ((isPress_ == true) && (press == false)) {
        // 触发弹起
        isTriggerRelease_ = true;
        isTriggerPress_ = false;
    }
    isPress_ = press;
}

/**
 * @description: 当前是否处于按下中
 * @return {}
 */
bool KeyRecord::IsPress() { return isPress_; }

/**
 * @description: 是否按下，即下降沿
 * @return {}
 */
bool KeyRecord::IsTriggerPress()
{
    if (isTriggerPress_ == true) {
        isTriggerPress_ = false;
        return true;
    }
    return false;
}

/**
 * @description: 是否弹起，即上升沿
 * @return {}
 */
bool KeyRecord::IsTriggerRelease()
{
    if (isTriggerRelease_ == true) {
        isTriggerRelease_ = false;
        return true;
    }
    return false;
}

/////////////////////////////////////////////////////////////////////
KeyBase::KeyBase()
{
    msg::bs::gamepad_cmd cmd;
    SaveGamepadBtn(cmd);

    for (auto item : btn_) {
        if (key_plus_.find(item.first) == key_plus_.end()) {
            key_plus_[item.first] = KeyPlus();
        }
    }
}

/**
 * @description: 只保存按键值
 * @return {}
 */
void KeyBase::SaveGamepadBtn(const msg::bs::gamepad_cmd& key)
{
    btn_["a"].SetValue(key.btnA);
    btn_["b"].SetValue(key.btnB);
    btn_["x"].SetValue(key.btnX);
    btn_["y"].SetValue(key.btnY);
    btn_["lb"].SetValue(key.btnLb);
    btn_["rb"].SetValue(key.btnRb);
    btn_["back"].SetValue(key.btnBack);
    btn_["start"].SetValue(key.btnStart);
    btn_["lp"].SetValue(key.btnLpress);
    btn_["rp"].SetValue(key.btnRpress);
    btn_["up"].SetValue(key.btnUp);
    btn_["down"].SetValue(key.btnDown);
    btn_["left"].SetValue(key.btnLeft);
    btn_["right"].SetValue(key.btnRight);

    btn_["lb_rb"].SetValue(key.btnLb && key.btnRb);

    btn_["a_rb"].SetValue(key.btnA && key.btnRb);
    btn_["b_rb"].SetValue(key.btnB && key.btnRb);
    btn_["x_rb"].SetValue(key.btnX && key.btnRb);
    btn_["y_rb"].SetValue(key.btnY && key.btnRb);

    btn_["a_lb"].SetValue(key.btnA && key.btnLb);
    btn_["b_lb"].SetValue(key.btnB && key.btnLb);
    btn_["x_lb"].SetValue(key.btnX && key.btnLb);
    btn_["y_lb"].SetValue(key.btnY && key.btnLb);

    btn_["rt_lb"].SetValue((key.stickRt > 0.4) && key.btnLb);
    btn_["rt"].SetValue(key.stickRt > 0.4);  // 将joystick抽象为按键
    btn_["lt"].SetValue(key.stickLt > 0.4);

    stick_["lx"] = key.stickLx;
    stick_["ly"] = key.stickLy;
    stick_["rx"] = key.stickRx;
    stick_["ry"] = key.stickRy;
    stick_["lt"] = key.stickLt;
    stick_["rt"] = key.stickRt;
}

bool KeyBase::UpdateGamepadKey()
{
    auto keyRet = MsgTryRecv<msg::bs::gamepad_cmd>("bs::gamepad", this);
    if (keyRet) {
        SaveGamepadBtn(keyRet.value());
        return true;
    }
    return false;
}

/**
 * @description: 短按的Is接口
 * @param key
 * @return {}
 */
bool KeyBase::IsShortRelease(std::string key)
{
    if (!key_plus_.at(key).GetMultiEnable()) {
        if (btn_.at(key).IsTriggerRelease()) {
            if (key_plus_.at(key).HasBeenLongPressed()) {
                key_plus_.at(key).SetLongPressed(false);
            } else {
                if (!key_plus_.at(key).GetMultiEnable()) {
                    key_plus_.at(key).ClearAll();
                    return true;
                }
            }
        }
    }
    return false;
};

/**
 * @description:  多键的Is接口，传入参数是2个用字符串表示的按键名
 * @param key1
 * @param key2
 * @return {}
 */
bool KeyBase::IsMultiKey(std::string key1, std::string key2)
{
    bool res = false;
    if (btn_.at(key1).IsPress() && btn_.at(key2).IsPress()) {
        key_plus_.at(key1).SetMultiEnable(true);
        key_plus_.at(key2).SetMultiEnable(true);
        PutInJustPressed(key1, key2);
    }
    if (IsInJustPressed(key1, key2)) {
        SecondReleaseInMulti(key1, key2, res);
        SecondReleaseInMulti(key2, key1, res);
    }
    return res;
};

/**
 * @description: 长按的Is接口。长按业务的逻辑是以固定周期持续执行某个动作。
 * @param key
 * @return {} 长按的Is接口以固定周期返回true，其他时刻一直保持为false。
 */
bool KeyBase::IsLongPress(std::string key)
{
    // 首先是来自LongPressingCount()的代码，如果长按则进行累加，如果构成多键则清空
    if (btn_.at(key).IsPress()) {
        key_plus_.at(key).Count();
    }
    if (key_plus_.at(key).GetMultiEnable()) {
        key_plus_.at(key).ClearAll();
    }

    if (key_plus_.at(key).IsLongPress()) {
        key_plus_.at(key).AfterLongPress();
        return true;
    } else {
        return false;
    }
};

/**
 * @description: 处理按键松开的函数
 * 按键松开需要区分3种情况，分别是“长按之后松开，构成多键时的松开，短按松开”。其中“短按松开”是最简单的情况。
 * “长按之后松开”，不需要执行任何逻辑，仅需复位按键即可。
 * “构成多键时松开”，检查构成了哪个组合的多键，并按当前键-另外一个键的顺序，调用多键的接口
 * @param key
 * @return {}
 */
void KeyBase::RunRelease(std::string key)
{
    if (key_plus_.at(key).HasBeenLongPressed()) {
        key_plus_.at(key).ClearAll();
    } else if (key_plus_.at(key).GetMultiEnable()) {
        if (key == multi_act.first) {
            MultiCallback(key, multi_act.second);
        }
        if (key == multi_act.second) {
            MultiCallback(key, multi_act.first);
        }
    } else {
        ShortPressCallback(key);
    }
};

/**
 * @description: 回调式接口，多键的处理逻辑
 * 两个参数的顺序需要注意，第一个参数代表当前松开的键，第二个参数代表与他组合的另一个键
 * @param key1
 * @param key2
 * @return {}
 */
void KeyBase::MultiCallback(std::string key1, std::string key2)
{
    // “第一键松开”，如果另外一个键，还未松开(未松开就是多键使能还为true)，则仅释放当前键的多键使能
    if (key_plus_.at(key2).GetMultiEnable()) {
        key_plus_.at(key1).SetMultiEnable(false);
    } else {
        // “第二键松开”，如果另外一个键已释放了其多键使能，则当前键是多键中的第二个键，他松开之时就要执行多键动作
        // 如果有不存在的多键动作，则跳过调用过程，仅将这两个键的多键属性复位即可
        if (multi_func_.find(multi_act) != multi_func_.end()) {
            multi_func_.at(multi_act)();
        }
        key_plus_.at(key1).SetMultiEnable(false);
        key_plus_.at(key1).ClearAll();
        // 多键触发一次后，将暂存的多键组合给清空掉
        multi_act = {};
    }
};

/**
 * @description: 回调式接口，长按的处理逻辑
 * @param key
 * @return {}
 */
void KeyBase::LongPressingCallback(std::string key)
{
    LongPressingCount(key);
    if (key_plus_.at(key).IsLongPress()) {
        // 判断该按键在func_中有没有提前注册长按动作
        if (func_.find(key) != func_.end()) {
            const auto& inner_map = func_[key];
            if (inner_map.find(KeyMode::Short) != inner_map.end()) {
                std::function<void()> func = func_.at(key).at(KeyMode::Long);
                func();
            }
        }
        key_plus_.at(key).AfterLongPress();
    }
};

/**
 * @description: 回调式接口，短按的处理逻辑
 * @param key
 * @return {}
 */
void KeyBase::ShortPressCallback(std::string key)
{
    if (!key_plus_.at(key).HasBeenLongPressed()) {
        // 判断该按键在func_中有没有提前注册短按动作
        if (func_.find(key) != func_.end()) {
            const auto& inner_map = func_[key];
            if (inner_map.find(KeyMode::Short) != inner_map.end()) {
                std::function<void()> func = func_.at(key).at(KeyMode::Short);
                func();
            }
        }
        key_plus_.at(key).ClearAll();
    }
};

/**
 * @description: 检测当前时刻是否构成多键，如果构成多键，将之存放到成员multi_act中
 * 两个键存放的先后顺序并不重要，多键仅关注是谁和谁构成了多键，而不关注顺序
 * 构成多键时，将按键的其他属性复位，消除了松开时触发长按信号的bug
 * @return {}
 */
void KeyBase::MultiEnable()
{
    std::vector<std::string> res;
    for (auto item : multi_key_set) {
        if (btn_.at(item).IsPress()) {
            res.push_back(item);
        }
    }
    if (res.size() >= 2) {
        multi_act = MakePairMultiFunc(res.at(0), res.at(1));
        key_plus_.at(multi_act.first).SetMultiEnable(true);
        key_plus_.at(multi_act.second).SetMultiEnable(true);
    }
};

/**
 * @description: 注册多键动作，将动作存放入容器multi_key_set中
 * 第1和第2的参数是按键名，先后顺序并不重要
 * @param key1
 * @param key2
 * @param func
 * @return {}
 */
void KeyBase::AddMultiFunc(std::string key1, std::string key2, std::function<void()> func)
{
    multi_key_set.insert(key1);
    multi_key_set.insert(key2);
    multi_func_[MakePairMultiFunc(key1, key2)] = func;
};

/**
 * @description: 添加短按、长按动作，通过type参数指定是短按还是长按
 * 如果重复添加，会覆盖掉之前的相同type的回调函数
 * @param key
 * @param mode
 * @param func
 * @return {}
 */
void KeyBase::AddFunc(std::string key, KeyMode mode, std::function<void()> func)
{
    func_[key][mode] = func;
    key_plus_[key] = KeyPlus();
};

/**
 * @description: 长按时，将指定按键的计数器进行累加
 * 如果处于多键使能状态，则清空计数器，此时计数器不起作用
 * 一旦构成了多键，则长按信号就发不出来了
 * @param key
 * @return {}
 */
void KeyBase::LongPressingCount(std::string key)
{
    if (btn_.at(key).IsPress()) {
        key_plus_.at(key).Count();
    }
    if (key_plus_.at(key).GetMultiEnable()) {
        key_plus_.at(key).ClearAll();
    }
};

/**
 * @description: 创建multi_func_容器存放的pair
 * 确保唯一性，以字典序来排列传入的2个按键名(字符串)
 * @param key1
 * @param key2
 * @return {}
 */
std::pair<std::string, std::string> KeyBase::MakePairMultiFunc(std::string key1, std::string key2)
{
    if (key1 < key2) {
        return {key1, key2};
    } else {
        return {key2, key1};
    }
};

/**
 * @description: 判断是否是第二键抬起，只有第二键抬起才发出多键动作
 * @param key1 松开的键
 * @param key2 与他构成多键的另外一个按键。根据key2的状态，来判断key1此时松开，是否是第二键松开
 * @param value
 * @return {}
 */
void KeyBase::SecondReleaseInMulti(std::string key1, std::string key2, bool& value)
{
    if (btn_.at(key1).IsTriggerRelease()) {
        if (key_plus_.at(key1).GetMultiEnable() && !key_plus_.at(key2).GetMultiEnable()) {
            value = true;
        }
        key_plus_.at(key1).SetMultiEnable(false);
    }
}

/**
 * @description: 将2个键存放到成功构成多键的容器中。用专门容器存储是为了避免发生交叉触发的bug
 * @param key1
 * @param key2
 * @return {}
 */
void KeyBase::PutInJustPressed(std::string key1, std::string key2)
{
    key_just_pressed.clear();
    key_just_pressed.insert(key1);
    key_just_pressed.insert(key2);
}

/**
 * @description: 判断这两个键是否是刚刚构成多键。同样是为了避免交叉触发的bug
 * @param key1
 * @param key2
 * @return {}
 */
bool KeyBase::IsInJustPressed(std::string key1, std::string key2)
{
    return key_just_pressed.find(key1) != key_just_pressed.end() && key_just_pressed.find(key2) != key_just_pressed.end();
};