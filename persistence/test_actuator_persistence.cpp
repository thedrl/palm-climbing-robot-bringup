#include <cassert>
#include <iostream>
#include <vector>
#include "actuator_persistence.h"
using namespace actuator_persistence;
struct Fake {
    bool can=true,has_attempt=false,cancel=false,stop=true,read_ok=true,changed=false;
    bool cancel_after_capture=false,cancel_after_write=false,post_unreadable=false;
    int reads=0,writes=0,stops=0;uint32_t time=0;
    SendResult sent=SendResult::ACKNOWLEDGED;
    Snapshot state;
    Fake(){strcpy(state.version,"USB Roboclaw 2x7A v4.4.8");state.current[2]=2;state.current[3]=162;state.timeout=2;state.config[1]=0x63;}
    bool eligible(){return can;}bool attempted(){return has_attempt;}bool cancelled(){return cancel;}
    bool capture(Snapshot&s){++reads;s=state;if(reads>1&&changed)s.pins[0]^=1;if(cancel_after_capture)cancel=true;return read_ok&&!(post_unreadable&&reads>1);}
    bool stopped(){++stops;return stop;}
    SendResult saveOnce(){++writes;has_attempt=true;if(cancel_after_write)cancel=true;return sent;}
    uint32_t nowMs(){return time;}void waitMs(uint32_t ms){time+=ms;}
};
int main(){
    unsigned checks=0;
    auto run=[&](Fake&f,Result r,int writes){Snapshot a,b;assert(save(f,a,b)==r);assert(f.writes==writes);++checks;};
    {uint8_t p[8];savePacket(128,p);const uint8_t expected[]={0x80,0x5e,0xe2,0x2e,0xab,0x7a,0xe4,0xa6};assert(!memcmp(p,expected,8));++checks;}
    {Fake f;run(f,Result::VERIFIED,1);assert(f.time>=750&&f.time<3000);++checks;run(f,Result::ALREADY_ATTEMPTED,1);}
    {Fake f;f.can=false;run(f,Result::PRECONDITION,0);}
    {Fake f;f.cancel=true;run(f,Result::CANCELLED,0);}
    {Fake f;f.read_ok=false;run(f,Result::READ_FAILED,0);}
    {Fake f;strcpy(f.state.version,"USB Roboclaw 2x7A v4.4.9");run(f,Result::UNSUPPORTED,0);}
    for(int pin=0;pin<5;++pin){for(uint8_t mode:{uint8_t(1),uint8_t(0xe2),uint8_t(0xf2)}){Fake f;f.state.pins[pin]=mode;run(f,Result::UNSAFE_STARTUP,0);}}
    for(uint16_t config:{uint16_t(0),uint16_t(0x0062),uint16_t(0x0061),uint16_t(0x0073),uint16_t(0x0163),uint16_t(0x8063)}){
        Fake f;f.state.config[0]=config>>8;f.state.config[1]=config;run(f,Result::UNSAFE_STARTUP,0);
    }
    for(uint32_t n:{0U,669U,676U,917U}){Fake f;f.state.current[0]=n>>24;f.state.current[1]=n>>16;f.state.current[2]=n>>8;f.state.current[3]=n;run(f,Result::CURRENT_NOT_PREPARED,0);}
    {Fake f;f.stop=false;run(f,Result::NOT_STOPPED,0);}
    {Fake f;f.cancel_after_capture=true;run(f,Result::CANCELLED,0);}
    {Fake f;f.cancel_after_write=true;run(f,Result::CANCELLED,1);}
    {Fake f;f.sent=SendResult::NOT_SENT;run(f,Result::TRANSMIT_FAILED,1);}
    {Fake f;f.sent=SendResult::UNCERTAIN;run(f,Result::ACK_UNCERTAIN,1);}
    {Fake f;f.post_unreadable=true;run(f,Result::RECONNECT_FAILED,1);assert(f.time>=3000&&f.time<3050);++checks;}
    {Fake f;f.changed=true;run(f,Result::CHANGED,1);}
    {Fake f;Snapshot a=f.state,b=a;b.current[7]=1;assert(!equal(a,b));b=a;b.timeout++;assert(!equal(a,b));b=a;b.voltage[4]++;assert(!equal(a,b));b=a;b.config[0]++;assert(!equal(a,b));++checks;}
    std::cout<<"PASS "<<checks<<" persistence policy checks\n";
}
