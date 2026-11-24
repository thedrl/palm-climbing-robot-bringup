#pragma once
#include "actuator_voltage_limit_packet.h"
struct ActuatorVoltageIO {
    bool cancelled(){return Serial.available()>0||!Serial;}
    bool readConfig(uint16_t&value){
        uint8_t data[2]={};
        if(!actuator_live_io.readOnce(99,2,data))return false;
        value=uint16_t(data[0])<<8|data[1];return true;
    }
    bool readLimits(actuator_voltage::Limits&limit){
        uint8_t data[5]={};
        if(!actuator_live_io.readOnce(59,5,data))return false;
        limit.minimum=uint16_t(data[0])<<8|data[1];
        limit.maximum=uint16_t(data[2])<<8|data[3];limit.offset=data[4];return true;
    }
    bool zeroPwm(){return actuator_current_io.zeroPwm();}
    bool writeLimits(const actuator_voltage::Limits&limit){
        ActuatorCurrentPacketIO io;
        return actuator_voltage::writeLimitPacket(io,roboclaw_addr,limit);
    }
} actuator_voltage_io;
bool reportActuatorVoltageRefusal(){
    if(actuator_voltage_guard.cancelled()){
        Serial.println("Actuator voltage preparation cancelled by USB input/disconnect.");return false;
    }
    actuator_telemetry.fault=true;actuator_telemetry.drain_required=true;
    actuator_telemetry.invalidate();
    Serial.printf("Actuator blocked: %s. Press Stop.\n",actuator_voltage_guard.reason());
    return false;
}
bool ensureActuatorVoltageCeiling(bool require_idle=false){
    if(actuator_voltage_guard.allow(actuator_voltage_io,require_idle))return true;
    return reportActuatorVoltageRefusal();
}
bool restoreActuatorVoltageMinimum(uint16_t target){
    if(actuator_voltage_guard.restoreMinimum(actuator_voltage_io,target))return true;
    return reportActuatorVoltageRefusal();
}
// Refresh real status after preparing settings. A latched previous warning may
// appear once; never mask it or substitute an all-clear status.
bool refreshActuatorVoltageStatus(){
    uint8_t voltage[2]={},status[4]={};
    if(actuator_voltage_io.cancelled()){
        actuator_voltage_guard.cancelPreparation();return reportActuatorVoltageRefusal();
    }
    if(!actuator_live_io.readOnce(24,2,voltage)){
        actuator_voltage_guard.readFailure();return reportActuatorVoltageRefusal();
    }
    if(actuator_voltage_io.cancelled()){
        actuator_voltage_guard.cancelPreparation();return reportActuatorVoltageRefusal();
    }
    if(!actuator_live_io.readOnce(90,4,status)){
        actuator_voltage_guard.readFailure();return reportActuatorVoltageRefusal();
    }
    if(actuator_voltage_io.cancelled()){
        actuator_voltage_guard.cancelPreparation();return reportActuatorVoltageRefusal();
    }
    actuator_telemetry.voltage.set(millis(),uint16_t(voltage[0])<<8|voltage[1]);
    const uint32_t bits=uint32_t(status[0])<<24|uint32_t(status[1])<<16|uint32_t(status[2])<<8|status[3];
    actuator_telemetry.error.set(millis(),int32_t(bits));return true;
}

void printActuatorVoltageLimits(){
    actuator_voltage::Limits limit;uint16_t config=0;
    if(!actuator_voltage_io.readConfig(config)||!actuator_voltage_io.readLimits(limit)){
        Serial.println("Actuator voltage/configuration read failed; no settings changed.");return;
    }
    Serial.printf("Actuator voltage: min=%.1f V max=%.1f V offset_raw=%u config=0x%04X battery_mode_raw=0x%04X; target max=25.2 V.\n",
        limit.minimum/10.0f,limit.maximum/10.0f,unsigned(limit.offset),unsigned(config),unsigned(config&actuator_voltage::BATTERY_MODE_MASK));
}
