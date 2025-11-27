/*
 * @Author: 唐文浩
 * @Date: 2022-05-07
 * @LastEditors: 唐文浩
 * @LastEditTime: 2025-02-19
 * @Description: 智研院协议封包和解包
 *
 * Copyright (c) 2022-2022 by 西湖大学智能产业研究院, All Rights Reserved.
 */
#pragma once

#include <arpa/inet.h>

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

class BinPackConstruct;
class ProtocolConstruct;
// 协议版本号
// 1，最初版本
// 2，精简了头部，TCP在应用层不需要关心分包，故删去
constexpr uint8_t MSG_VERSION = 0; // 当前协议版本号

// 网络传输数据包头
struct MsgHeader
{
    uint8_t magicNum[4]; // 魔数，WLIR，西湖大学智研院缩写

    uint8_t version;    // 版本号，当头修改时，需要更新版本号
    uint8_t option{0};  // 报文选项，0-json字符串；1-二进制
    uint16_t optExt{0}; // 额外报文选项，根据option的不同意义不同，二进制时表示具体二进制的指令。目前1代表debug数据

    uint32_t sequence{0}; // 同一个条消息同一个序列号，接收者可以根据该值判断消息是新消息还是重复消息

    uint16_t len{0};    // 数据长度
    uint16_t crcSum{0}; // 后面数据累加和校验

    MsgHeader()
    {
        magicNum[0] = 'I';
        magicNum[1] = 'I';
        magicNum[2] = 'R';
        magicNum[3] = 'I';
        version = MSG_VERSION;
    }
    size_t size() const { return sizeof(MsgHeader); }
};
constexpr uint8_t MSG_HEADER_SIZE = sizeof(MsgHeader);

// 构造一个通讯二进制流
class ProtocolConstruct
{
public:
    // 根据二进制数据流构造消息
    explicit ProtocolConstruct(const uint8_t *data, uint16_t len);
    // 根据字符串构造消息
    explicit ProtocolConstruct(const std::string &str);
    explicit ProtocolConstruct(const nlohmann::json &j);
    explicit ProtocolConstruct(const std::vector<uint8_t> &data);
    explicit ProtocolConstruct(const BinPackConstruct &data);
    explicit ProtocolConstruct(const std::vector<BinPackConstruct> &data);

    virtual ~ProtocolConstruct() = default;

    uint16_t GetLen() const;
    const std::vector<uint8_t> &GetData() const;

    void SetOption(uint8_t option, uint16_t optExt);
    void SetSequence(uint32_t sequence);

private:
    void Construct(const uint8_t *data, uint16_t len);
    std::vector<uint8_t> msgData_;
};

// 协议解析类
class ProtocolParse
{
public:
    // 未解析数据指针，数据长度
    explicit ProtocolParse(const uint8_t *data, uint16_t len);
    explicit ProtocolParse(const std::vector<uint8_t> &data);
    virtual ~ProtocolParse() {};

    const std::vector<uint8_t> &GetData() const;
    bool Empty() { return bodyData_.empty(); }

    uint8_t GetOption() const { return option_; }
    uint16_t GetExtOption() const { return optExt_; }
    uint8_t GetVersion() const { return version_; }
    uint32_t GetSequence() const { return sequence_; }

private:
    uint8_t option_;
    uint8_t version_;
    uint16_t optExt_;
    uint32_t sequence_;

private:
    int32_t FindHeaderPos(const uint8_t *data, uint16_t len);
    void Parse(const uint8_t *data, uint16_t len);
    std::vector<uint8_t> bodyData_;
};

// 二进制包的信息头
struct BinPacketHeader
{
    char name[12];
    uint32_t size;
    BinPacketHeader(const std::string &nameIn, uint32_t sizeIn)
    {
        auto len = nameIn.size();
        if (len > 12)
        {
            len = 11;
        }
        size_t i = 0;
        for (i = 0; i < len; i++)
        {
            name[i] = nameIn[i];
        }
        name[i] = '\0';

        size = htonl(sizeIn);
    }
};

// 构造一个二进制包协议，当协议头表示协议内容为二进制时，其包也需要符合sdk的二进制协议
// 二进制包是可以自由组合级联的，即如下
// msgHeader - BinPackConstruct - BinPackConstruct -...
// 这里是构造一个，通过vector级联成多个
class BinPackConstruct
{
public:
    // 根据二进制数据流构造消息
    explicit BinPackConstruct(const std::string &tag, const uint8_t *data, uint16_t len);
    explicit BinPackConstruct(const std::string &tag, const std::vector<uint8_t> &data);
    virtual ~BinPackConstruct() = default;

    const std::vector<uint8_t> &GetData() const { return binData_; }
    size_t GetSize() const { return binData_.size(); }

private:
    std::vector<uint8_t> binData_;
};

class BinPackParse
{
public:
    explicit BinPackParse(const std::vector<uint8_t> &rawData);
    explicit BinPackParse(const ProtocolParse &rawData);
    virtual ~BinPackParse() {};

    uint32_t GetSize() const;
    std::string GetTag(uint32_t idx) const;
    const std::vector<uint8_t> &GetData(uint32_t idx) const;

    template <typename T>
    const T &GetData(uint32_t idx) const
    {
        return reinterpret_cast<T &>(binDatas_.at(idx));
    }

private:
    std::vector<std::string> tags_;
    uint16_t count_{0}; // 有多少个二进制包
    std::vector<std::vector<uint8_t>> binDatas_;
};
