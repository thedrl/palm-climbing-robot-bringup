#pragma once
// Read-only, fixed-deadline packet queries. No library retry loop and no writes
// to motor/current-limit/configuration registers from the telemetry path.
struct ActuatorLiveIO {
    uint32_t now()const{return millis();}
    bool inputPending()const{return Serial.available()>0 || !Serial;}
    bool writeIfRoom(const char*p,int n)const{return live_telemetry_io.writeIfRoom(p,n);}
    static uint16_t crcByte(uint16_t crc,uint8_t b){
        crc^=uint16_t(b)<<8;
        for(int i=0;i<8;++i)crc=(crc&0x8000)?uint16_t((crc<<1)^0x1021):uint16_t(crc<<1);
        return crc;
    }
    bool readFailed(uint8_t cmd,uint8_t received,uint8_t expected,const char*reason)const{
        // Failure reporting must not delay Stop when the USB reader stalls.
        // Drop this detail if the FIFO is full; the telemetry fault stays latched.
        char line[64];
        const int n=snprintf(line,sizeof(line),"RoboClaw cmd=%u: %s, bytes=%u/%u\n",cmd,reason,received,expected);
        if(n>0 && n<int(sizeof(line)))live_telemetry_io.writeIfRoom(line,n);
        return false;
    }
    bool readOnce(uint8_t cmd,uint8_t count,uint8_t*out)const{
        if(!((cmd==49||cmd==48||cmd==90)&&count==4) && !(cmd==24&&count==2) && !((cmd==135||cmd==136)&&count==8) && !(cmd==59&&count==5) && !(cmd==99&&count==2))return false;
        if(RoboSerial.availableForWrite()<2)return readFailed(cmd,0,count+2,"TX full");
        // Discard stale bytes before a new read; no second reader owns this UART.
        for(int i=0;i<64 && RoboSerial.available();++i)RoboSerial.read();
        if(RoboSerial.available())return readFailed(cmd,0,count+2,"stale RX backlog");
        const uint8_t request[2]={roboclaw_addr,cmd};
        const uint32_t began=micros();
        if(RoboSerial.write(request,2)!=2)return readFailed(cmd,0,count+2,"request incomplete");
        uint8_t response[10]={};uint8_t n=0;
        while(n<count+2 && uint32_t(micros()-began)<12000){
            if(RoboSerial.available())response[n++]=uint8_t(RoboSerial.read());
            else delayMicroseconds(50);
        }
        if(n!=count+2)return readFailed(cmd,n,count+2,"12ms timeout");
        uint16_t crc=crcByte(crcByte(0,roboclaw_addr),cmd);
        for(uint8_t i=0;i<count;++i)crc=crcByte(crc,response[i]);
        if(crc!=(uint16_t(response[count])<<8|response[count+1]))return readFailed(cmd,n,count+2,"CRC mismatch");
        for(uint8_t i=0;i<count;++i)out[i]=response[i];
        return true;
    }
} actuator_live_io;

// Other actuator procedures have their own stopping/watchdog rules. Background
// UART queries are suspended throughout those procedures and afterwards until
// an explicit Stop confirms both channels. Normal manual fwd/rev is monitored.
struct ActuatorCommandScope {
    bool touches,normal,explicit_stop;
    ActuatorCommandScope(const String&command):
        touches((command.startsWith("act ") && !command.startsWith("act power") && command!="act diag" && command!="act config" && command!="act limits" && command!="act current save" && command!="act voltage" && command!="act voltage apply" && !command.startsWith("act voltage restore-min"))||command=="start"),
        normal(command=="act fwd"||command=="act rev"),
        explicit_stop(command=="act stop"||command=="stop"||command=="s"||command=="estop"){
        actuator_current_guard.beginCommand();
        actuator_voltage_guard.beginCommand();
        actuator_telemetry.foreground=true;
        actuator_telemetry.manual=normal;
        if(touches){actuator_telemetry.known_idle=false;if(!normal)actuator_telemetry.invalidate();}
    }
    ~ActuatorCommandScope(){
        actuator_telemetry.foreground=false;actuator_telemetry.manual=false;
        if(touches && !normal && !explicit_stop)actuator_telemetry.known_idle=false;
        if(explicit_stop){
            actuator_telemetry.explicitStop();
            if(actuator_telemetry.known_idle){actuator_current_guard.acknowledgeStop();actuator_voltage_guard.acknowledgeStop();}
        }
    }
};
