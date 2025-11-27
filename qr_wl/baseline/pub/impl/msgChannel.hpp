
#pragma once
#include "commTools.hpp"
#include "msgCenterImpl.hpp"

class MsgSend
{
public:
    /**
     * @description: 构造函数，参数是发送name
     * @param name
     * @return {}
     */
    explicit MsgSend(const std::string& tag) : tag_(tag) {}

    /**
     * @description: 发送数据，类型是T
     * @return {}
     */
    template <typename T>
    bool TrySend(const T& buf)
    {
        return GetMsgFast().TrySend(tag_, MsgType(buf));
    }

    template <typename T>
    bool ForceSend(const T& buf)
    {
        return GetMsgFast().ForceSend(tag_, MsgType(buf));
    }

    template <typename T>
    bool BlockSend(const T& buf, u32 ms)
    {
        return GetMsgFast().BlockSend(tag_, MsgType(buf), ms);
    }

    /**
     * @description: buf参数只是为了标明类型使用
     * @return {}
     */
    void Clear() { GetMsgFast().Clear(tag_); }

private:
    std::string tag_;
};

// 单发送多接收，接收接口
class MsgRecv
{
public:
    /**
     * @description:
     * @return {}
     */
    template <typename S>
    MsgRecv(const std::string& tag, S hash) : tag_(tag)
    {
        hash_ = GetHash(hash);
    }

    template <typename T>
    std::optional<T> TryRecv()
    {
        auto ret = GetMsgFast().TryRecv(tag_, hash_);
        if (ret) {
            return std::make_optional(ret.value().GetType<T>());
        }
        return std::nullopt;
    }

    template <typename T>
    std::optional<T> BlockRecv(u32 ms)
    {
        auto ret = GetMsgFast().BlockRecv(tag_, hash_, ms);
        if (ret) {
            return std::make_optional(ret.value().GetType<T>());
        }
        return std::nullopt;
    }

    void Connect() { GetMsgFast().Connect(tag_, hash_); }

    void DisConnect() { GetMsgFast().DisConnect(tag_, hash_); }

    void Clear() { GetMsgFast().Clear(tag_, hash_); }

private:
    std::string tag_;
    std::size_t hash_;
};

template <typename T>
bool MsgTrySend(const std::string& tag, const T& buf)
{
    MsgSend msg(tag);
    return msg.ForceSend(buf);  // msg的fifo策略默认强塞
}

template <typename T>
bool MsgBlockSend(const std::string& tag, const T& buf, u32 ms)
{
    MsgSend msg(tag);
    return msg.BlockSend(buf, ms);
}

template <typename T, typename S>
std::optional<T> MsgTryRecv(const std::string& tag, S hash)
{
    MsgRecv msg(tag, hash);
    msg.Connect();
    return msg.TryRecv<T>();
}

template <typename T, typename S>
std::optional<T> MsgBlockRecv(const std::string& tag, S hash, u32 ms)
{
    MsgRecv msg(tag, hash);
    msg.Connect();
    return msg.BlockRecv<T>(ms);
}