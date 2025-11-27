#include "protocol.hpp"

#include <arpa/inet.h>

using namespace ::std;

//////////协议构造///////////////////////
ProtocolConstruct::ProtocolConstruct(const uint8_t *data, uint16_t len) { Construct(data, len); }

ProtocolConstruct::ProtocolConstruct(const std::string &str) { Construct((uint8_t *)str.c_str(), str.size()); }

ProtocolConstruct::ProtocolConstruct(const std::vector<uint8_t> &data) { Construct(data.data(), data.size()); }

void ProtocolConstruct::Construct(const uint8_t *data, uint16_t len)
{
    MsgHeader header;
    header.len = htons(len);  // 网络序
    header.sequence = htonl(rand());
    // 校验码是有序的
    for (int i = 0; i < len; i++) {
        header.crcSum += data[i];
    }
    header.crcSum = htons(header.crcSum);

    auto *pHead = reinterpret_cast<uint8_t *>(&header);
    for (size_t i = 0; i < header.size(); i++) {
        msgData_.push_back(pHead[i]);
    }

    for (auto i = 0; i < len; i++) {
        msgData_.push_back(data[i]);
    }
}

void ProtocolConstruct::SetOption(uint8_t option, uint16_t optExt)
{
    MsgHeader *header = (MsgHeader *)msgData_.data();
    header->option = option;
    header->optExt = htons(optExt);
}

void ProtocolConstruct::SetSequence(uint32_t sequence)
{
    MsgHeader *header = (MsgHeader *)msgData_.data();
    header->sequence = htonl(sequence);
}

const std::vector<uint8_t> &ProtocolConstruct::GetData() const { return msgData_; }

uint16_t ProtocolConstruct::GetLen() const { return msgData_.size(); }

////////////////////协议解析////////////////////////
ProtocolParse::ProtocolParse(const uint8_t *data, uint16_t len) { Parse(data, len); }

ProtocolParse::ProtocolParse(const std::vector<uint8_t> &data) { Parse(data.data(), data.size()); }

void ProtocolParse::Parse(const uint8_t *data, uint16_t len)
{
    int32_t pos = FindHeaderPos(data, len);
    if (pos >= 0) {
        MsgHeader *head = (MsgHeader *)&data[pos];
        uint16_t bodyLen = ntohs(head->len);
        option_ = head->option;
        optExt_ = ntohs(head->optExt);
        version_ = head->version;
        sequence_ = ntohl(head->sequence);

        // 数据长度不够
        if ((pos + head->size() + bodyLen) > len) {
            return;
        }

        // 校验并存储数据
        uint16_t crc = 0;
        for (int i = 0; i < bodyLen; i++) {
            crc += data[pos + head->size() + i];
            bodyData_.push_back(data[pos + head->size() + i]);
        }
        if (crc != ntohs(head->crcSum)) {
            bodyData_.clear();
            return;
        }
    }
}

const std::vector<uint8_t> &ProtocolParse::GetData() const { return bodyData_; }

/****************************************************************
 * 功能：在数据流中查找数据头的偏移位置，目前只支持一个数据包
 * 输入：@1 数据流，务必确保大于最小头长度
 * 输出：@1 第几个字节是头, 负数表示未查到
 ****************************************************************/
int32_t ProtocolParse::FindHeaderPos(const uint8_t *data, uint16_t len)
{
    int32_t offset = -1;
    for (int i = 0; i < len; i++) {
        // 只判断头魔数正确即可
        MsgHeader *header = (MsgHeader *)&data[i];
        if ((header->magicNum[0] == 'I') && (header->magicNum[1] == 'I') && (header->magicNum[2] == 'R') && (header->magicNum[3] == 'I')) {
            offset = i;
            break;
        }
    }
    return offset;
}

/////////////二进制包的构造和解析//////////////////////
BinPackConstruct::BinPackConstruct(const std::string &tag, const uint8_t *data, uint16_t len)
{
    BinPacketHeader header(tag, len);

    const uint8_t *headData = reinterpret_cast<const uint8_t *>(&header);
    for (size_t i = 0; i < sizeof(header); i++) {
        binData_.push_back(headData[i]);
    }
    for (size_t i = 0; i < len; i++) {
        binData_.push_back(data[i]);
    }
}

BinPackConstruct::BinPackConstruct(const std::string &tag, const std::vector<uint8_t> &data)
{
    BinPacketHeader header(tag, data.size());

    const uint8_t *headData = reinterpret_cast<const uint8_t *>(&header);
    for (size_t i = 0; i < sizeof(header); i++) {
        binData_.push_back(headData[i]);
    }
    binData_.insert(binData_.end(), data.begin(), data.end());
}

/**
 * @description: 二进制包解析
 * @param &data
 * @return {}
 */
BinPackParse::BinPackParse(const std::vector<uint8_t> &rawData)
{
    auto data = rawData;  // 需要把数据切除，因此去除const属性
    while (data.size() >= sizeof(BinPacketHeader)) {
        const BinPacketHeader *binHead = reinterpret_cast<const BinPacketHeader *>(data.data());

        // 接收的数据满足头规定的长度
        uint32_t len = ntohl(binHead->size);
        if (len > data.size() - sizeof(BinPacketHeader)) {
            break;  // 不满足退出
        }

        string tag(binHead->name);
        const uint8_t *bodyData = reinterpret_cast<const uint8_t *>(binHead + 1);
        vector<uint8_t> binData;
        for (uint32_t i = 0; i < len; i++) {
            binData.push_back(bodyData[i]);
        }

        tags_.push_back(tag);
        binDatas_.push_back(binData);

        data.erase(data.begin(), data.begin() + sizeof(BinPacketHeader) + len);  // 移除该数据
    }
}

uint32_t BinPackParse::GetSize() const { return tags_.size(); }

std::string BinPackParse::GetTag(uint32_t idx) const { return tags_.at(idx); }

const std::vector<uint8_t> &BinPackParse::GetData(uint32_t idx) const { return binDatas_.at(idx); }