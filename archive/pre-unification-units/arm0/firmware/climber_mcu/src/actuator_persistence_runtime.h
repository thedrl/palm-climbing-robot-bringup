#pragma once
#include "actuator_persistence.h"
// Include after actuatorStop, all actuator guards, and telemetry adapters are
// defined. Caller must use a foreground ActuatorCommandScope with touches=false.
// No call here clears a prior fault, stops/starts a motor, or changes a setting.
namespace actuator_persistence_runtime {
static bool nvm_attempted=false;
struct IO {
    bool eligible()const{
        return actuator_motor_channel==1&&actuator_telemetry.foreground&&actuator_telemetry.known_idle&&
            !actuator_telemetry.manual&&actuator_telemetry.command_power==0&&!actuator_telemetry.fault&&
            !actuator_current_guard.failed()&&!actuator_voltage_guard.failed();
    }
    bool attempted()const{return nvm_attempted;}
    bool cancelled()const{return Serial.available()>0||!Serial;}
    uint32_t nowMs()const{return millis();}
    void waitMs(uint32_t ms){pumpCanOnce();delay(ms);}
    bool clearOld(){
        for(int i=0;i<64&&RoboSerial.available();++i)RoboSerial.read();
        return !RoboSerial.available();
    }
    bool request(uint8_t cmd){
        if(cancelled()||!clearOld()||RoboSerial.availableForWrite()<2)return false;
        const uint8_t packet[2]={roboclaw_addr,cmd};
        return !cancelled()&&RoboSerial.write(packet,2)==2;
    }
    // This separate allowlist permits only the snapshot/idle reads needed by
    // the explicit save command; it does not broaden background telemetry.
    bool read(uint8_t cmd,uint8_t count,uint8_t*out){
        if(!((cmd==135||cmd==136)&&count==8)&&!((cmd==59||cmd==75)&&count==5)&&
           !(cmd==99&&count==2)&&!(cmd==15&&count==1)&&!((cmd==48||cmd==49||cmd==90)&&count==4))return false;
        const uint32_t began=micros();
        if(!request(cmd))return false;
        uint8_t reply[10]={};uint8_t n=0;
        while(n<count+2&&uint32_t(micros()-began)<30000){
            if(cancelled())return false;
            if(RoboSerial.available())reply[n++]=uint8_t(RoboSerial.read());
            else delayMicroseconds(50);
        }
        if(n!=count+2)return false;
        uint16_t crc=actuator_persistence::crcByte(actuator_persistence::crcByte(0,roboclaw_addr),cmd);
        for(uint8_t i=0;i<count;++i)crc=actuator_persistence::crcByte(crc,reply[i]);
        if(crc!=(uint16_t(reply[count])<<8|reply[count+1])||cancelled())return false;
        memcpy(out,reply,count);return true;
    }
    bool readVersion(char(&text)[49]){
        const uint32_t began=micros();
        if(!request(21))return false;
        uint8_t response[51]={};uint8_t n=0,end=0;
        while(n<sizeof(response)&&uint32_t(micros()-began)<50000){
            if(cancelled())return false;
            if(!RoboSerial.available()){delayMicroseconds(50);continue;}
            response[n++]=uint8_t(RoboSerial.read());
            if(end==0&&response[n-1]==0)end=n;
            if(end!=0&&n==end+2)break;
        }
        if(end==0||end>49||n!=end+2)return false;
        uint16_t crc=actuator_persistence::crcByte(actuator_persistence::crcByte(0,roboclaw_addr),21);
        for(uint8_t i=0;i<end;++i)crc=actuator_persistence::crcByte(crc,response[i]);
        if(crc!=(uint16_t(response[end])<<8|response[end+1])||cancelled())return false;
        memcpy(text,response,end);
        for(int i=int(end)-2;i>=0&&(text[i]=='\r'||text[i]=='\n');--i)text[i]=0;
        return true;
    }
    bool capture(actuator_persistence::Snapshot&s){
        return readVersion(s.version)&&read(135,8,s.current)&&read(136,8,s.current+8)&&
            read(59,5,s.voltage)&&read(99,2,s.config)&&read(75,5,s.pins)&&read(15,1,&s.timeout);
    }
    bool stopped(){
        uint8_t pwm[4]={},current[4]={},status[4]={};
        if(!read(48,4,pwm)||!read(49,4,current)||!read(90,4,status))return false;
        // M2 is physically unused; its known idle-current offset is not a fault.
        // Actual PWM must still be zero on BOTH channels. M1 current is exact 0.
        return pwm[0]==0&&pwm[1]==0&&pwm[2]==0&&pwm[3]==0&&current[0]==0&&current[1]==0&&
            (actuator_persistence::be32(status)&~uint32_t(0x03000000))==0;
    }
    actuator_persistence::SendResult saveOnce(){
        using namespace actuator_persistence;
        if(cancelled()||nvm_attempted||!clearOld()||RoboSerial.availableForWrite()<8)return SendResult::NOT_SENT;
        uint8_t packet[8];savePacket(roboclaw_addr,packet);
        if(cancelled())return SendResult::NOT_SENT;
        nvm_attempted=true; // latch BEFORE the irreversible write; no retry on ambiguity.
        actuator_telemetry.known_idle=false;
        actuator_telemetry.invalidate();
        roboclaw_safety_timeout_ok=false; // controller reset invalidates cached acknowledgement.
        if(RoboSerial.write(packet,sizeof(packet))!=sizeof(packet))return SendResult::UNCERTAIN;
        const uint32_t began=micros();
        while(uint32_t(micros()-began)<250000){
            if(cancelled())return SendResult::UNCERTAIN;
            if(RoboSerial.available())return RoboSerial.read()==0xFF?SendResult::ACKNOWLEDGED:SendResult::UNCERTAIN;
            delayMicroseconds(50);
        }
        return SendResult::UNCERTAIN;
    }
};
}
bool saveActuatorCurrentLimits(){
    using namespace actuator_persistence;
    actuator_persistence_runtime::IO io;Snapshot before,after;
    const Result result=save(io,before,after);
    if(result==Result::VERIFIED){
        Serial.printf("Actuator current settings saved; post-save M1 limit=%.2f A, regen/min raw=0x%08lX. M2/voltage/config/pins/timeout unchanged. No movement. Press Stop before moving.\n",
            be32(after.current)/100.0f,(unsigned long)be32(after.current+4));
        return true;
    }
    const char*why="unknown";
    switch(result){
        case Result::PRECONDITION:why="need prior acknowledged Stop, healthy guards, M1 selected";break;
        case Result::CANCELLED:why="USB input/disconnect; save may have occurred if already sent";break;
        case Result::READ_FAILED:why="snapshot read failed; no save sent";break;
        case Result::UNSUPPORTED:why="requires USB Roboclaw 2x7A v4.4.8; no save sent";break;
        case Result::UNSAFE_STARTUP:why="requires known packet config 0x0063 and all input modes zero; reset could start motion, no save sent";break;
        case Result::CURRENT_NOT_PREPARED:why="M1 current limit must already be 6.70-6.75 A; no save sent";break;
        case Result::NOT_STOPPED:why="zero PWM/M1 current and healthy status not confirmed";break;
        case Result::ALREADY_ATTEMPTED:why="one save was already attempted this ESP boot; no resend";break;
        case Result::TRANSMIT_FAILED:why="save packet not transmitted";break;
        case Result::ACK_UNCERTAIN:why="settings match but save ACK missing; persistence unverified, no resend";break;
        case Result::RECONNECT_FAILED:why="post-save readback timed out; persistence unverified, no resend";break;
        case Result::CHANGED:why="post-save settings changed; persistence unverified, no resend";break;
        default:break;
    }
    actuator_telemetry.fault=true;
    actuator_telemetry.drain_required=true;
    actuator_telemetry.invalidate();
    Serial.printf("Actuator current save blocked/unverified: %s. Press Stop.\n",why);
    return false;
}
