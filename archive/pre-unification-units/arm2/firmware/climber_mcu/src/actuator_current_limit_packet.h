#pragma once
#include "actuator_current_limit.h"
namespace actuator_current {
// Only the motion-preparation guard calls this writer. No NVM operation.
// Keep the raw second word bit-for-bit, including signed/negative encodings.
template<class IO>
bool writeLimitPacket(IO&io,uint8_t address,uint8_t channel,uint32_t maximum,uint32_t return_limit){
    if((channel!=1&&channel!=2)||maximum!=TARGET_RAW)return false;
    uint8_t packet[12]={address,uint8_t(channel==1?133:134)};
    for(uint8_t i=0;i<4;++i){
        packet[2+i]=uint8_t(maximum>>(24-8*i));
        packet[6+i]=uint8_t(return_limit>>(24-8*i));
    }
    uint16_t crc=0;
    for(uint8_t i=0;i<10;++i){
        crc^=uint16_t(packet[i])<<8;
        for(uint8_t bit=0;bit<8;++bit)
            crc=(crc&0x8000)?uint16_t((crc<<1)^0x1021):uint16_t(crc<<1);
    }
    packet[10]=uint8_t(crc>>8);packet[11]=uint8_t(crc);
    if(io.availableForWrite()<int(sizeof(packet)))return false;
    for(uint8_t i=0;i<64&&io.available();++i)io.read();
    if(io.available())return false;
    const uint32_t began=io.nowMicros();
    if(io.write(packet,sizeof(packet))!=sizeof(packet))return false;
    // One complete transaction, strict ACK, and a deadline including TX time.
    // Do not consume USB input: a pending Stop remains for its normal handler.
    while(uint32_t(io.nowMicros()-began)<12000){
        if(io.available())return io.read()==0xff;
        io.wait();
    }
    return false;
}
}
