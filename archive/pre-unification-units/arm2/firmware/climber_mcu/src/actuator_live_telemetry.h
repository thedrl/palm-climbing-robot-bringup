#pragma once
#include <stdint.h>
#include <stdio.h>
#include "odrive_live_usb.h"
namespace actuator_live {
struct Value {
    uint32_t at=0; bool seen=false; int32_t a=0,b=0;
    long age(uint32_t now)const{return seen?long(uint32_t(now-at)):-1L;}
    void set(uint32_t now,int32_t x,int32_t y=0){at=now;a=x;b=y;seen=true;}
};
enum State { WAITING=0, IDLE=1, MOVING=2, PAUSED=3, READ_FAULT=4, STOP_UNCONFIRMED=5 };
class Telemetry {
public:
    bool enabled=true,foreground=false,manual=false,known_idle=false,fault=false;
    bool drain_required=false;
    uint8_t channel=1;
    int command_power=0,manual_power=40;
    static constexpr int MAX_MANUAL_POWER=60;
    uint32_t last_command=0,sequence=0;
    Value current,pwm,voltage,limit,error;
    void invalidate(){current={};pwm={};voltage={};limit={};error={};usb_.clear();}
    void commandResult(int power,bool ok,uint32_t now){
        known_idle=false;
        if(ok){command_power=power;last_command=now;}
        else {fault=true;invalidate();}
    }
    void stopResult(bool ok){known_idle=ok;command_power=0;if(!ok){fault=true;invalidate();}}
    void explicitStop(){if(known_idle){fault=false;invalidate();last_poll_=0;}}
    State state(uint32_t now)const {
        if(fault)return known_idle?READ_FAULT:STOP_UNCONFIRMED;
        if(!enabled)return PAUSED;
        if(manual && command_power!=0 && uint32_t(now-last_command)<100)return MOVING;
        if(!foreground && known_idle)return IDLE;
        return PAUSED;
    }
    bool fresh(const Value&v,uint32_t now,uint32_t max_age=600)const{return v.seen && uint32_t(now-v.at)<=max_age;}
    const char* raisedPowerCheck(int target,uint32_t now)const {
        if(!enabled || fault)return "Telemetry must be enabled and healthy; press Stop.";
        if(!fresh(voltage,now)||!fresh(current,now)||!fresh(pwm,now)||!fresh(error,now))return "Fresh voltage/current/PWM/status readings required.";
        if(!fresh(limit,now,30000)||limit.a<=0||limit.a>675)return "Read and verify the RoboClaw current limit (must be at most 6.75 A).";
        if((uint32_t(error.a)&~uint32_t(0x03000000))!=0)return "RoboClaw reports a fault/warning; inspect diagnostics.";
        if(voltage.a<=0 || int64_t(voltage.a)*target > 120LL*127)return "Requested PWM exceeds the nominal 12 V average-voltage ceiling.";
        return nullptr;
    }
    const char* setManualPower(int target,uint32_t now){
        if(target<1||target>MAX_MANUAL_POWER)return "Manual power must be 1..60 (maximum 47.2% drive).";
        if(!known_idle || manual)return "Stop the actuator before changing drive power.";
        if(target>40){
            if(const char* why=raisedPowerCheck(target,now))return why;
            if(pwm.a!=0||pwm.b!=0||current.a>50||current.a<-50||current.b>50||current.b<-50)return "Need zero PWM and a stable idle-current baseline before increasing power.";
        }
        manual_power=target;return nullptr;
    }
    template<class IO> void tick(uint32_t now,int unit,uint8_t selected,IO&io){
        if(channel!=selected){channel=selected;invalidate();}
        if(io.inputPending()){usb_.clear();return;}
        const State s=state(now);
        if((s==IDLE || s==MOVING) && uint32_t(now-last_poll_)>=50){
            last_poll_=now;
            const uint8_t slot=next_++%4;
            uint8_t cmd=slot==0?49:slot==1?48:slot==2?24:90;
            if(slot==3 && s==IDLE && (!limit.seen || uint32_t(now-limit.at)>=5000))cmd=channel==1?135:136;
            const uint8_t count=cmd==24?2:(cmd==135||cmd==136)?8:4;
            uint8_t data[8]={};
            if(!io.readOnce(cmd,count,data)){
                // No further UART reads until an explicit, acknowledged Stop.
                fault=true;drain_required=true;invalidate();
            }else{
                const uint32_t received=io.now();
                if(cmd==49) current.set(received,signed16(data),signed16(data+2));
                else if(cmd==48)pwm.set(received,signed16(data),signed16(data+2));
                else if(cmd==24)voltage.set(received,uint16_t(data[0])<<8|data[1]);
                else if(cmd==90)error.set(received,int32_t(big32(data)));
                else limit.set(received,int32_t(big32(data)));
            }
        }
        now=io.now();
        if(uint32_t(now-last_emit_)>=200){
            last_emit_=now;
            char line[256];
            int n=snprintf(line,sizeof(line),"ACT_LIVE,1,%d,%lu,%u,%d,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%lu,%d,%d",
                unit,(unsigned long)++sequence,channel,int(state(now)),voltage.age(now),current.age(now),pwm.age(now),limit.age(now),error.age(now),
                (long)voltage.a,(long)current.a,(long)current.b,(long)pwm.a,(long)pwm.b,(unsigned long)uint32_t(limit.a),(unsigned long)uint32_t(error.a),command_power,manual_power);
            if(n>0 && n<int(sizeof(line)))usb_.offer(now,unit,sequence,line,n);
        }
        usb_.pump(now,io);
    }
private:
    static int16_t signed16(const uint8_t*p){return int16_t(uint16_t(p[0])<<8|p[1]);}
    static uint32_t big32(const uint8_t*p){return uint32_t(p[0])<<24|uint32_t(p[1])<<16|uint32_t(p[2])<<8|p[3];}
    uint32_t last_poll_=0,last_emit_=0;
    uint8_t next_=0;
    odrive_live::UsbSnapshot usb_{"ACT_PART"};
};
}
