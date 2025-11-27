
#pragma once

#include <arpa/inet.h>

#include <string>
#include <vector>

#include "baseline.hpp"
#include "nlohmann/json.hpp"

class BinPackConstruct;
class ProtocolConstruct;

constexpr uint8_t MSG_VERSION = 0;

struct MsgHeader
{
    uint8_t magicNum[4];

    uint8_t version;
    uint8_t option{0};
    uint16_t optExt{0};

    uint32_t sequence{0};

    uint16_t len{0};
    uint16_t crcSum{0};

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

class ProtocolConstruct
{
public:
    explicit ProtocolConstruct(const uint8_t *data, uint16_t len);

    explicit ProtocolConstruct(const std::string &str);
    explicit ProtocolConstruct(const std::vector<uint8_t> &data);

    virtual ~ProtocolConstruct() = default;

    uint16_t GetLen() const;
    const std::vector<uint8_t> &GetData() const;

    void SetOption(uint8_t option, uint16_t optExt);
    void SetSequence(uint32_t sequence);
    uint32_t GetSequence();

private:
    void Construct(const uint8_t *data, uint16_t len);
    std::vector<uint8_t> msgData_;
};

class ProtocolParse
{
public:
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

struct BinPacketHeader
{
    char name[12];
    uint32_t size;
    BinPacketHeader(const std::string &nameIn, uint32_t sizeIn);
};

class BinPackConstruct
{
public:
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
    uint16_t count_{0};
    std::vector<std::vector<uint8_t>> binDatas_;
};
