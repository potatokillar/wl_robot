
#pragma once
#include "commTools.hpp"
#include "msgCenterImpl.hpp"

/**
 * @description: RPC接口
 * REQ，请求的数据结构
 * RESP，回应的数据结构
 * @return {}
 */
class RpcSend
{
public:
    /**
     * @description:
     * @param tag
     * @param hash
     * @return {}
     */
    RpcSend(const std::string& tag)
    {
        reqTag_ = tag + "REQ";
        respTag_ = tag + "RESP";

        hash_ = GetHash(respTag_);
    }

    std::optional<MsgType> Request(const MsgType& request, uint32_t ms)
    {
        if (GetMsgRpc().BlockSend(reqTag_, request, ms) == true) {
            return GetMsgRpc().BlockRecv(respTag_, hash_, ms);
        }
        return std::nullopt;
    }

    /**
     * @description: 一个空请求，阻塞型
     * @param ms
     * @return {}
     */
    std::optional<MsgType> Request(uint32_t ms)
    {
        MsgType request(true);
        if (GetMsgRpc().BlockSend(reqTag_, request, ms) == true) {
            return GetMsgRpc().BlockRecv(respTag_, hash_, ms);
        }
        return std::nullopt;
    }

    void Connect()
    {
        // 对应发送来说，关心的是接收REQ
        GetMsgRpc().Connect(respTag_, hash_);
    }

    void DisConnect() { GetMsgRpc().DisConnect(respTag_, hash_); }

private:
    std::string reqTag_;
    std::string respTag_;
    std::size_t hash_;
};

class RpcRecv
{
public:
    RpcRecv(const std::string& tag, std::function<MsgType(MsgType)> func)
    {
        call_ = func;
        Construct(tag);
    }

    RpcRecv(const std::string& tag, std::function<MsgType()> func)
    {
        callNoParam_ = func;
        Construct(tag);
    }

    void Run()
    {
        auto req = TryGetRequest();
        if (req) {
            if (call_) {
                MsgType resp = call_(req.value());
                Response(resp);
            } else if (callNoParam_) {
                MsgType resp = callNoParam_();
                Response(resp);
            }
        }
    }

    void Run(u32 ms)
    {
        auto req = GetMsgRpc().BlockRecv(reqTag_, hash_, ms);
        if (req) {
            if (call_) {
                MsgType resp = call_(req.value());
                GetMsgRpc().BlockSend(respTag_, resp, ms);
            } else if (callNoParam_) {
                MsgType resp = callNoParam_();
                GetMsgRpc().BlockSend(respTag_, resp, ms);
            }
        }
    }

    void Connect() { GetMsgRpc().Connect(reqTag_, hash_); }

    void DisConnect()
    {
        //    GetMsgRpc().DisConnect(reqTag_, hash_);
    }

private:
    // 尝试获取请求
    std::optional<MsgType> TryGetRequest() { return GetMsgRpc().TryRecv(reqTag_, hash_); }

    // 回应请求
    void Response(const MsgType& arg) { GetMsgRpc().TrySend(respTag_, arg); }

    void Construct(const std::string& tag)
    {
        reqTag_ = tag + "REQ";
        respTag_ = tag + "RESP";
        hash_ = GetHash(reqTag_);  // hash不是随机值，因为rpc永远是一对一的
    }

private:
    std::string reqTag_;
    std::string respTag_;
    std::size_t hash_;

    std::function<MsgType(MsgType)> call_;
    std::function<MsgType()> callNoParam_;  // 无参数的rpc接收
};
////////////////////////////////////////////////////////////

template <typename REQ, typename RESP>
bool RpcTrySend(const std::string& tag, const REQ& request, RESP* response)
{
    RpcSend rpc(tag);
    rpc.Connect();
    auto resp = rpc.Request(MsgType(request), 20);
    if (resp) {
        *response = resp.value().GetType<RESP>();
        return true;
    }
    rpc.DisConnect();
    return false;
}

template <typename RESP>
bool RpcTrySend(const std::string& tag, RESP* response)
{
    RpcSend rpc(tag);
    rpc.Connect();
    auto resp = rpc.Request(20);
    if (resp) {
        *response = resp.value().GetType<RESP>();
        return true;
    }
    rpc.DisConnect();
    return false;
}

template <typename REQ, typename RESP>
bool RpcBlockSend(const std::string& tag, REQ request, RESP* response, u32 ms)
{
    RpcSend rpc(tag);
    rpc.Connect();
    auto resp = rpc.Request(MsgType(request), ms);
    if (resp) {
        *response = resp.value().GetType<RESP>();
        return true;
    }
    rpc.DisConnect();
    return false;
}

template <typename RESP>
bool RpcBlockSend(const std::string& tag, RESP* response, u32 ms)
{
    RpcSend rpc(tag);
    rpc.Connect();
    auto resp = rpc.Request(ms);
    if (resp) {
        *response = resp.value().GetType<RESP>();
        return true;
    }
    rpc.DisConnect();
    return false;
}

template <typename REQ, typename RESP>
void RpcTryRecv(const std::string& tag, std::function<RESP(REQ)> func)
{
    RpcRecv rpc(tag, func);
    rpc.Connect();
    rpc.Run();
}
template <typename RESP>
void RpcTryRecv(const std::string& tag, std::function<RESP()> func)
{
    RpcRecv rpc(tag, func);
    rpc.Connect();
    rpc.Run();
}
template <typename REQ, typename RESP>
void RpcBlockRecv(const std::string& tag, std::function<RESP(REQ)> func, u32 ms)
{
    RpcRecv rpc(tag, func);
    rpc.Connect();
    rpc.Run(ms);
}
template <typename RESP>
void RpcBlockRecv(const std::string& tag, std::function<RESP()> func, u32 ms)
{
    RpcRecv rpc(tag, func);
    rpc.Connect();
    rpc.Run(ms);
}