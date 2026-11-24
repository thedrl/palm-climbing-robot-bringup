#pragma once
#include <stdint.h>
#include <string.h>
namespace actuator_persistence {
struct Snapshot {
    uint8_t current[16]={}; // M1 max/regen, then M2 max/regen, unchanged raw fields.
    uint8_t voltage[5]={},config[2]={},pins[5]={},timeout=0;
    char version[49]={};
};
inline uint32_t be32(const uint8_t*p){return uint32_t(p[0])<<24|uint32_t(p[1])<<16|uint32_t(p[2])<<8|p[3];}
inline uint16_t crcByte(uint16_t crc,uint8_t b){
    crc^=uint16_t(b)<<8;
    for(int i=0;i<8;++i)crc=(crc&0x8000)?uint16_t((crc<<1)^0x1021):uint16_t(crc<<1);
    return crc;
}
inline void savePacket(uint8_t address,uint8_t(&packet)[8]){
    packet[0]=address;packet[1]=94;packet[2]=0xE2;packet[3]=0x2E;packet[4]=0xAB;packet[5]=0x7A;
    uint16_t crc=0;for(uint8_t i=0;i<6;++i)crc=crcByte(crc,packet[i]);
    packet[6]=uint8_t(crc>>8);packet[7]=uint8_t(crc);
}
inline bool supported(const Snapshot&s){
    // readVersion removes CR/LF only after validating the complete wire CRC.
    return strcmp(s.version,"USB Roboclaw 2x7A v4.4.8")==0;
}
inline bool safeStartup(const Snapshot&s){
    // WriteNVM resets the controller. For this installation only the known
    // packet-serial configuration and unassigned auxiliary pins are allowed;
    // automatic homing/RC/analog startup behavior could move after the reset.
    if(s.config[0]!=0||s.config[1]!=0x63)return false;
    for(uint8_t pin:s.pins)if(pin!=0)return false;
    return true;
}
inline bool bounded(const Snapshot&s){const uint32_t n=be32(s.current);return n>=670&&n<=675;}
inline bool equal(const Snapshot&a,const Snapshot&b){
    return memcmp(a.current,b.current,sizeof(a.current))==0&&memcmp(a.voltage,b.voltage,sizeof(a.voltage))==0&&
        memcmp(a.config,b.config,sizeof(a.config))==0&&memcmp(a.pins,b.pins,sizeof(a.pins))==0&&
        a.timeout==b.timeout&&strcmp(a.version,b.version)==0;
}
enum class Result { VERIFIED, PRECONDITION, CANCELLED, READ_FAILED, UNSUPPORTED, UNSAFE_STARTUP, CURRENT_NOT_PREPARED,
                    NOT_STOPPED, ALREADY_ATTEMPTED, TRANSMIT_FAILED, ACK_UNCERTAIN, RECONNECT_FAILED, CHANGED };
enum class SendResult { NOT_SENT, UNCERTAIN, ACKNOWLEDGED };
// IO never writes a current limit or motion command. Once saveOnce is called,
// even an ambiguous ACK cannot trigger a second NVM write in this transaction.
template<class IO> Result save(IO&io,Snapshot&before,Snapshot&after){
    if(!io.eligible())return Result::PRECONDITION;
    if(io.attempted())return Result::ALREADY_ATTEMPTED;
    if(io.cancelled())return Result::CANCELLED;
    if(!io.capture(before))return io.cancelled()?Result::CANCELLED:Result::READ_FAILED;
    if(!supported(before))return Result::UNSUPPORTED;
    if(!safeStartup(before))return Result::UNSAFE_STARTUP;
    if(!bounded(before))return Result::CURRENT_NOT_PREPARED;
    if(!io.stopped())return io.cancelled()?Result::CANCELLED:Result::NOT_STOPPED;
    if(io.cancelled())return Result::CANCELLED;
    const SendResult sent=io.saveOnce();
    if(sent==SendResult::NOT_SENT)return Result::TRANSMIT_FAILED;
    const uint32_t began=io.nowMs();
    // The official command resets the RoboClaw. Allow reset to finish before
    // independently querying settings; never fill this interval with retries.
    while(uint32_t(io.nowMs()-began)<750){
        if(io.cancelled())return Result::CANCELLED;
        io.waitMs(2);
    }
    while(uint32_t(io.nowMs()-began)<3000){
        if(io.cancelled())return Result::CANCELLED;
        if(io.capture(after)){
            if(!equal(before,after)||!bounded(after))return Result::CHANGED;
            if(!io.stopped())return io.cancelled()?Result::CANCELLED:Result::NOT_STOPPED;
            if(io.cancelled())return Result::CANCELLED;
            return sent==SendResult::ACKNOWLEDGED?Result::VERIFIED:Result::ACK_UNCERTAIN;
        }
        io.waitMs(20);
    }
    return Result::RECONNECT_FAILED;
}
}
