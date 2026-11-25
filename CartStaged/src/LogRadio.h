#pragma once
#include "CartCommon.h"
namespace log_radio {
constexpr uint8_t REQUEST_TYPE=4, DATA_TYPE=5;
constexpr size_t PAYLOAD=160;
enum Operation : uint8_t { DUMP=1, LIST=2, ACK=3, CANCEL=4, RAM=5, RELEASE=6 };
#pragma pack(push,1)
struct Request { cart::Header header; uint32_t session, transfer, id, offset; uint8_t operation; };
struct Data { cart::Header header; uint32_t session, transfer, offset; uint16_t length; uint8_t bytes[PAYLOAD]; };
#pragma pack(pop)
inline cart::Header header(uint8_t type) { return {cart::MAGIC,cart::VERSION,type}; }
inline bool valid(const Request &r) {
 return r.header.magic==cart::MAGIC && r.header.version==cart::VERSION && r.header.type==REQUEST_TYPE &&
 r.session && r.transfer && r.operation>=DUMP && r.operation<=RELEASE &&
 (r.operation!=DUMP || (r.id && r.id<=99999999)) && (r.operation!=RAM || r.id==0) && (r.operation!=RELEASE || r.id);
}
inline bool valid(const Data &d, int length) {
 return length==static_cast<int>(sizeof(Data)) && d.header.magic==cart::MAGIC &&
 d.header.version==cart::VERSION && d.header.type==DATA_TYPE && d.session && d.transfer && d.length && d.length<=PAYLOAD;
}
// Host/dongle commits only contiguous bytes; duplicates get re-ACKed, never printed twice.
struct Receiver {
 uint32_t next=0;
 bool accept(uint32_t offset, uint16_t length) const { return length && length<=PAYLOAD && offset==next; }
 bool duplicate(uint32_t offset,uint16_t length) const { return length && length<=PAYLOAD && offset<next && next-offset==length; }
};
static_assert(sizeof(Data)<=250 && sizeof(Request)<=sizeof(cart::Command),"ESP-NOW size bounds");
}
