#pragma once
#include "actuator_voltage_limit.h"
namespace actuator_voltage {
// CMD57 includes the original minimum and automatic-voltage-offset byte.
// No legacy shorter payload, NVM save, configuration write or reset.
template<class IO>
bool writeLimitPacket(IO&io,uint8_t address,const Limits&limit){
    if(limit.maximum!=TARGET_MAX_RAW||limit.minimum>=limit.maximum)return false;
    uint8_t packet[9]={address,57,uint8_t(limit.minimum>>8),uint8_t(limit.minimum),
        uint8_t(limit.maximum>>8),uint8_t(limit.maximum),limit.offset,0,0};
    uint16_t crc=0;
    for(uint8_t i=0;i<7;++i){
        crc^=uint16_t(packet[i])<<8;
        for(uint8_t bit=0;bit<8;++bit)
            crc=(crc&0x8000)?uint16_t((crc<<1)^0x1021):uint16_t(crc<<1);
    }
    packet[7]=uint8_t(crc>>8);packet[8]=uint8_t(crc);
    if(io.availableForWrite()<int(sizeof(packet)))return false;
    for(uint8_t i=0;i<64&&io.available();++i)io.read();
    if(io.available())return false;
    const uint32_t began=io.nowMicros();
    if(io.write(packet,sizeof(packet))!=sizeof(packet))return false;
    while(uint32_t(io.nowMicros()-began)<12000){
        if(io.available())return io.read()==0xff;
        io.wait();
    }
    return false;
}
}
