#include "sdkDebug.hpp"

#include "baseline.hpp"
#include "robotMedia.hpp"  // 使用语音功能所用

using namespace nlohmann;
using namespace std;

SdkDebug::SdkDebug(std::shared_ptr<SdkProtocolServer> sdk, std::unique_ptr<DeviceDebugServer> ptr_) : sdk_(sdk)
{
    RegisterSdkCall();
    ptr_dbg_ = std::move(ptr_);
}

void SdkDebug::RegisterSdkCall()
{
    // 兼容处理
    sdk_->AddMethodCall("debug", "SetCustomParam", [this](json params) -> json { return this->SetCustomParam(params); });
    sdk_->AddMethodCall("debug", "GetCustomParam", [this](json params) -> json { return this->GetCustomParam(params); });
    sdk_->AddMethodCall("debug", "GetCustomParamInfo", [this](json params) -> json { return this->GetCustomParamInfo(params); });

    // todo 兼容旧调试协议，<= qrChart2.3.3
    sdk_->AddMethodCall("debug", "Subscribe", [this](json params) -> json { return this->GetDebugData(params); });
    sdk_->AddTimeoutCall("debugData", 2, [this]() { this->PubDebugData(); });
}

nlohmann::json SdkDebug::SetCustomParam(const nlohmann::json& params)
{
    std::map<string, vector<ParamValue>> reqParams;
    for (const auto& item : params) {
        try {
            string name = item.at("name");

            if (item.at("value").is_array()) {
                // 不允许空数组，因此不判断长度，直接抛异常
                if (item.at("value")[0].is_number()) {
                    vector<double> value = item.at("value");

                    auto tag = paramUpdateClient.GetWhichTagCanDeal(name);
                    if (tag.first == true) {
                        reqParams[tag.second].emplace_back(name, value);
                    } else {
                        return ErrState2json(RetState::noSupport);  // 有一个不支持就是失败
                    }
                }

                else if (item.at("value")[0].is_string()) {
                    vector<string> value = item.at("value");
                    auto tag = paramUpdateClient.GetWhichTagCanDeal(name);
                    if (tag.first == true) {
                        reqParams[tag.second].emplace_back(name, value);
                    } else {
                        return ErrState2json(RetState::noSupport);  // 有一个不支持就是失败
                    }
                }
            } else if (item.at("value").is_string()) {
                string tmp = item.at("value");
                vector<string> value;
                value.push_back(tmp);
                auto tag = paramUpdateClient.GetWhichTagCanDeal(name);
                if (tag.first == true) {
                    reqParams[tag.second].emplace_back(name, value);
                } else {
                    return ErrState2json(RetState::noSupport);  // 有一个不支持就是失败
                }
            } else if (item.at("value").is_number()) {
                std::string tmp = item.at("value");
                vector<std::string> value;
                value.push_back(tmp);
                auto tag = paramUpdateClient.GetWhichTagCanDeal(name);
                if (tag.first == true) {
                    reqParams[tag.second].emplace_back(name, value);
                } else {
                    return ErrState2json(RetState::noSupport);  // 有一个不支持就是失败
                }
            }

        } catch (const std::exception&) {
            return ErrState2json(RetState::parseErr);
        }
    }

    for (auto& v : reqParams) {
        RetState err = paramUpdateClient.ReqSetParameter(v.first, v.second);
        // 有一个失败，剩下的不再执行
        if (err != RetState::ok) {
            return ErrState2json(err);
        }
    }
    AddMediaPackage(MediaTaskPackage("paramWrite"));
    return ErrState2json(RetState::ok);
}

nlohmann::json SdkDebug::GetCustomParam(const nlohmann::json& params)
{
    map<string, vector<string>> names;
    for (const auto& item : params) {
        try {
            auto tag = paramUpdateClient.GetWhichTagCanDeal(item);
            if (tag.first == true) {
                names[tag.second].push_back(item);
            } else {
                return ErrState2json(RetState::noSupport);
            }
        } catch (const std::exception&) {
            return ErrState2json(RetState::parseErr);
        }
    }

    json ret;
    for (auto& v : names) {
        vector<ParamValue> result;
        if (paramUpdateClient.ReqGetParameter(v.first, v.second, result) == RetState::ok) {
            for (const auto& item : result) {
                json var;
                var["name"] = item.name;
                if (item.num.empty() == false) {
                    std::vector<double> str_vec = {};
                    for (auto& element : item.num) {
                        str_vec.push_back(element);
                    }
                    var["value"] = str_vec;
                } else if (item.str.empty() == false) {
                    var["value"] = item.str;
                }

                ret.push_back(var);
            }
        } else {
            return ErrState2json(RetState::error);
        }
    }
    return ret;
}

nlohmann::json SdkDebug::GetCustomParamInfo(const nlohmann::json& params)
{
    (void)params;

    json ret;
    auto tags = paramUpdateClient.GetAllTag();
    if (tags.first == true) {
        for (auto& tag : tags.second) {
            vector<ParamInfo> result;
            if (paramUpdateClient.ReqGetParameterInfo(tag, result) == RetState::ok) {
                for (const auto& item : result) {
                    json var;
                    var["name"] = item.name;
                    if (item.type == ParamType::range) {
                        std::vector<double> str_vec = {};
                        for (auto& element : item.num) {
                            str_vec.push_back(element);
                        }
                        var["range"] = str_vec;
                    } else if (item.type == ParamType::restrict) {
                        std::vector<double> str_vec = {};
                        for (auto& element : item.num) {
                            str_vec.push_back(element);
                        }
                        var["restrict"] = str_vec;
                    } else if (item.type == ParamType::text) {
                        var["text"] = item.str;
                    }
                    ret.push_back(var);
                }
            }
        }
    }
    return ret;
}

nlohmann::json SdkDebug::GetDebugData(const nlohmann::json& params)
{
    (void)params;
    debugClients_.insert(sdk_->GetClientInfo());
    return ErrState2json(RetState::ok);
}

void SdkDebug::PubDebugData()
{
    auto [ret, vData] = ptr_dbg_->GetData();
    if (ret == true) {
        for (const auto& info : debugClients_) {
            sdk_->RawSend(info, 1, 2, vData);
        }
    }
}