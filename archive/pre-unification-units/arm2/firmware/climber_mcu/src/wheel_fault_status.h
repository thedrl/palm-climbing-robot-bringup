#pragma once
#include "odrive_live_telemetry.h"

// Retain the last received samples before a fault heartbeat invalidates Iq.
// These are historical samples with explicit ages, not simultaneous readings.
struct WheelFaultStatus {
    bool valid=false,notice=false;
    uint32_t at=0,error=0,last_error=0;
    uint8_t state=0;
    odrive_live::Sample vi,iq,encoder;
    void observe(uint32_t id,uint8_t len,const uint8_t* bytes,uint32_t now,uint8_t node,
                 const odrive_live::Telemetry& live){
        if(!bytes || len!=8 || (id&~uint32_t(0x7ff)) || (id>>5)!=node || (id&0x1f)!=1)return;
        uint32_t bits=0;memcpy(&bits,bytes,4);
        if(bits && bits!=last_error){
            valid=notice=true;at=now;error=bits;state=bytes[4];
            vi=live.vi;iq=live.iq;encoder=live.encoder;
        }
        last_error=bits;
    }
    template<class IO> void emit(IO&io){
        if(!notice || io.inputPending())return;
        char line[64];
        const int n=(error&0x04000000)
            ?snprintf(line,sizeof(line),"\nWheel fault: SPINOUT_DETECTED. Run w0 diag.\n")
            :snprintf(line,sizeof(line),"\nWheel fault: 0x%08lx. Run w0 diag.\n",(unsigned long)error);
        if(n>0 && n<int(sizeof(line)) && io.writeIfRoom(line,n))notice=false;
    }
};
