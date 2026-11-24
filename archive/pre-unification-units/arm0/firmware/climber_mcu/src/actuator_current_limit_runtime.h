#pragma once
#include "actuator_current_limit_packet.h"
// Controller settings are changed only by this guarded motion preparation,
// never by the background telemetry poller. No NVM save or restart is sent.
struct ActuatorCurrentPacketIO {
    int availableForWrite(){return RoboSerial.availableForWrite();}
    int available(){return RoboSerial.available();}
    int read(){return RoboSerial.read();}
    size_t write(const uint8_t*p,size_t n){return RoboSerial.write(p,n);}
    uint32_t nowMicros(){return micros();}
    void wait(){delayMicroseconds(50);}
};
struct ActuatorCurrentIO {
    bool cancelled(){return Serial.available()>0 || !Serial;}
    bool readLimit(uint8_t channel,uint32_t&value,uint32_t&return_limit){
        if(channel!=1&&channel!=2)return false;
        uint8_t data[8]={};
        if(!actuator_live_io.readOnce(channel==1?135:136,8,data))return false;
        value=uint32_t(data[0])<<24|uint32_t(data[1])<<16|uint32_t(data[2])<<8|data[3];
        return_limit=uint32_t(data[4])<<24|uint32_t(data[5])<<16|uint32_t(data[6])<<8|data[7];
        if(channel==actuator_motor_channel)actuator_telemetry.limit.set(millis(),int32_t(value));
        return true;
    }
    bool zeroPwm(){
        uint8_t data[4]={};
        return actuator_live_io.readOnce(48,4,data) && data[0]==0 && data[1]==0 && data[2]==0 && data[3]==0;
    }
    bool writeLimit(uint8_t channel,uint32_t value,uint32_t return_limit){
        // The legacy library's two-argument setter writes zero as the second
        // field. Send the preserved pair explicitly; ACK alone grants no motion.
        ActuatorCurrentPacketIO io;
        return actuator_current::writeLimitPacket(io,roboclaw_addr,channel,value,return_limit);
    }
} actuator_current_io;

bool ensureActuatorCurrentCeiling(uint8_t channel){
    if(actuator_current_guard.allow(channel,actuator_current_io))return true;
    if(actuator_current_guard.cancelled()){
        Serial.println("Actuator command cancelled by USB input/disconnect.");
        return false;
    }
    actuator_telemetry.fault=true;
    actuator_telemetry.drain_required=true;
    actuator_telemetry.invalidate();
    Serial.printf("Actuator M%u blocked: %s. Press Stop.\n",channel,actuator_current_guard.reason());
    if(actuator_current_guard.observed()){
        Serial.printf("Last limit read: raw=%lu (%.2f A); required 6.70-6.75 A.\n",
            (unsigned long)actuator_current_guard.lastLimit(),actuator_current_guard.lastLimit()/100.0f);
        Serial.printf("Last regen/min field: raw=0x%08lX unsigned=%lu signed=%lld.\n",
            (unsigned long)actuator_current_guard.lastReturnLimit(),
            (unsigned long)actuator_current_guard.lastReturnLimit(),
            (long long)(int64_t(actuator_current_guard.lastReturnLimit())-
                ((actuator_current_guard.lastReturnLimit()&0x80000000U)?0x100000000LL:0)));
    }
    return false;
}

void printActuatorCurrentLimits(){
    Serial.println("Actuator current-limit fields (read only; no settings changed):");
    for(uint8_t channel=1;channel<=2;++channel){
        uint32_t maximum=0,return_limit=0;
        if(!actuator_current_io.readLimit(channel,maximum,return_limit)){
            Serial.printf("M%u: current-limit read failed.\n",channel);
            continue;
        }
        Serial.printf("M%u: max raw=%lu (%.2f A); regen/min raw=0x%08lX unsigned=%lu signed=%lld\n",
            channel,(unsigned long)maximum,maximum/100.0f,
            (unsigned long)return_limit,(unsigned long)return_limit,
            (long long)(int64_t(return_limit)-((return_limit&0x80000000U)?0x100000000LL:0)));
    }
}
