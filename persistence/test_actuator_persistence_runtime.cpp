#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <deque>
#include <cassert>
#include <iostream>
#include <cstdarg>
#include "actuator_persistence.h"
static uint32_t fake_us;
uint32_t micros(){return fake_us;}uint32_t millis(){return fake_us/1000;}
void delayMicroseconds(uint32_t us){fake_us+=us;}void delay(uint32_t ms){fake_us+=1000*ms;}void pumpCanOnce(){}
struct USB { bool on=true;int pending=0;int available(){return pending;}operator bool()const{return on;} void printf(const char*,...){} } Serial;
struct Guard {bool bad=false;bool failed(){return bad;}} actuator_current_guard,actuator_voltage_guard;
struct Telemetry {bool foreground=true,known_idle=true,manual=false,fault=false,drain_required=false;int command_power=0;void invalidate(){} } actuator_telemetry;
uint8_t actuator_motor_channel=1,roboclaw_addr=128;bool roboclaw_safety_timeout_ok=true;
struct Uart {
    std::deque<uint8_t>rx;std::vector<std::vector<uint8_t>>writes;
    bool corrupt=false,ack=true,silent=false,partial=false;int room=128;
    std::vector<uint8_t> config={0,0x63},pins={0,0,0,0,0};
    std::vector<uint8_t> current={0,0,2,162,0,0,3,148};
    int available(){return rx.size();}int read(){if(rx.empty())return -1;int n=rx.front();rx.pop_front();return n;}
    int availableForWrite(){return room;}
    size_t write(const uint8_t*p,size_t n){
        writes.emplace_back(p,p+n);if(partial)return n-1;
        if(n==8){if(ack)rx.push_back(0xff);return n;}
        assert(n==2);uint8_t cmd=p[1];std::vector<uint8_t>d;
        if(silent)return n;
        switch(cmd){
            case 21:{const char*v="USB Roboclaw 2x7A v4.4.8\r\n";d.assign(v,v+strlen(v)+1);break;}
            case 135:case 136:d=current;break;
            case 59:d={0,59,0,252,20};break;
            case 99:d=config;break;
            case 75:d=pins;break;
            case 15:d={2};break;
            case 48:case 49:case 90:d={0,0,0,0};break;
            default:assert(false);
        }
        uint16_t crc=actuator_persistence::crcByte(actuator_persistence::crcByte(0,p[0]),cmd);
        for(auto b:d){rx.push_back(b);crc=actuator_persistence::crcByte(crc,b);}
        if(corrupt)crc^=1;
        rx.push_back(crc>>8);rx.push_back(crc);return n;
    }
} RoboSerial;
#include "actuator_persistence_runtime.h"
void reset(){fake_us=0;RoboSerial=Uart{};Serial=USB{};actuator_telemetry=Telemetry{};actuator_current_guard=Guard{};actuator_voltage_guard=Guard{};actuator_motor_channel=1;roboclaw_safety_timeout_ok=true;actuator_persistence_runtime::nvm_attempted=false;}
int saves(){int n=0;for(const auto&w:RoboSerial.writes)if(w.size()==8){assert(w[1]==94);n++;}return n;}
int main(){
    reset();assert(saveActuatorCurrentLimits());assert(saves()==1&&!actuator_telemetry.known_idle&&!roboclaw_safety_timeout_ok);
    reset();RoboSerial.corrupt=true;assert(!saveActuatorCurrentLimits()&&saves()==0);
    reset();RoboSerial.ack=false;assert(!saveActuatorCurrentLimits()&&saves()==1);
    reset();RoboSerial.silent=true;assert(!saveActuatorCurrentLimits()&&saves()==0);assert(fake_us<=50000);
    reset();actuator_current_guard.bad=true;assert(!saveActuatorCurrentLimits()&&RoboSerial.writes.empty());
    reset();actuator_voltage_guard.bad=true;assert(!saveActuatorCurrentLimits()&&RoboSerial.writes.empty());
    reset();actuator_telemetry.fault=true;assert(!saveActuatorCurrentLimits()&&RoboSerial.writes.empty());
    reset();actuator_telemetry.known_idle=false;assert(!saveActuatorCurrentLimits()&&RoboSerial.writes.empty());
    reset();Serial.pending=1;assert(!saveActuatorCurrentLimits()&&RoboSerial.writes.empty()&&Serial.pending==1);
    reset();actuator_motor_channel=2;assert(!saveActuatorCurrentLimits()&&RoboSerial.writes.empty());
    reset();RoboSerial.current[3]=0xad;assert(!saveActuatorCurrentLimits()&&saves()==0);
    reset();RoboSerial.partial=true;assert(!saveActuatorCurrentLimits()&&saves()==0);
    for(int pin:{1,2}){for(uint8_t mode:{uint8_t(0xe2),uint8_t(0xf2)}){reset();RoboSerial.pins[pin]=mode;assert(!saveActuatorCurrentLimits()&&saves()==0);}}
    for(uint16_t config:{uint16_t(0),uint16_t(0x0062),uint16_t(0x0061),uint16_t(0x0073),uint16_t(0x0163)}){
        reset();RoboSerial.config={uint8_t(config>>8),uint8_t(config)};assert(!saveActuatorCurrentLimits()&&saves()==0);
    }
    std::cout<<"PASS 21 runtime adapter checks\n";
}
