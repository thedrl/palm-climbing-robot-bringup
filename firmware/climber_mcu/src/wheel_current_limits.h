#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
namespace wheel_current {
constexpr float SOFT_AMPS = 35.0f;
constexpr float HARD_AMPS = 50.0f;
struct Map { uint16_t soft, hard; Map(uint16_t s=0,uint16_t h=0):soft(s),hard(h){} };
inline bool supported(uint8_t protocol, uint8_t hw_major, uint8_t hw_minor,
        uint8_t hw_variant, uint8_t major, uint8_t minor, uint8_t revision,
        uint8_t unreleased, uint32_t commit, Map& out) {
    out = {};
    if (protocol!=2 || hw_major!=5 || hw_minor!=2 || hw_variant!=0 ||
        major!=0 || minor!=6 || unreleased) return false;
    if (revision==11 && (commit==0xc063d552u || commit==0xb3c4cf45u))
        out={305,306};
    else if (revision==12 && (commit==0xd92a33feu || commit==0x5de70d7bu))
        out={325,326};
    return out.soft!=0;
}
inline bool close(float actual,float wanted) {
    return isfinite(actual) && actual==wanted; // 35 and 50 are exactly representable.
}
// One outstanding SDO transaction. A write ACK has four bytes; a float/uint32
// read has eight. Never accept an ACK or another endpoint as a read value.
struct Sdo {
    bool waiting=false, received=false;
    uint8_t node=0, expected_length=0;
    uint16_t endpoint=0;
    uint32_t value=0;
    void cancel() { waiting=false; received=false; }
    void begin(uint8_t n,uint16_t ep,uint8_t len) {
        node=n; endpoint=ep; expected_length=len; value=0;
        received=false; waiting=true;
    }
    void observe(uint32_t id,uint8_t len,const uint8_t* data) {
        if (!waiting || received || !data || id!=(uint32_t(node)<<5|5u) ||
            len!=expected_length || len<4 ||
            (uint16_t(data[1]) | uint16_t(data[2])<<8)!=endpoint) return;
        // Reserved response bytes must be ignored (ODrive CAN protocol).
        if (len==8) value=uint32_t(data[4]) | uint32_t(data[5])<<8 |
            uint32_t(data[6])<<16 | uint32_t(data[7])<<24;
        received=true;
    }
    template<class IO> bool transact(IO& io,uint8_t n,uint16_t ep,
            bool write,uint32_t bits,uint32_t& result) {
        cancel();
        if (io.cancelled() || !io.drain() || io.cancelled()) return false;
        uint8_t data[8]={uint8_t(write),uint8_t(ep),uint8_t(ep>>8),0,
            uint8_t(bits),uint8_t(bits>>8),uint8_t(bits>>16),uint8_t(bits>>24)};
        begin(n,ep,write?4:8);
        if (!io.send(uint32_t(n)<<5|4u,write?8:4,data)) {cancel();return false;}
        uint32_t started=io.now();
        while (uint32_t(io.now()-started)<150) {
            if (io.cancelled()) {cancel();return false;}
            io.pump();
            if (io.cancelled()) {cancel();return false;}
            if (received) { result=value; waiting=false; return true; }
            io.pause();
        }
        cancel();return false;
    }
    template<class IO> bool readU32(IO& io,uint8_t n,uint16_t ep,uint32_t& value_out) {
        return transact(io,n,ep,false,0,value_out);
    }
    template<class IO> bool readFloat(IO& io,uint8_t n,uint16_t ep,float& value_out) {
        uint32_t bits=0;
        if (!readU32(io,n,ep,bits)) return false;
        memcpy(&value_out,&bits,4);
        return isfinite(value_out);
    }
    template<class IO> bool writeFloat(IO& io,uint8_t n,uint16_t ep,float value_in) {
        if (!isfinite(value_in)) return false;
        uint32_t bits=0,ignored=0;memcpy(&bits,&value_in,4);
        return transact(io,n,ep,true,bits,ignored);
    }
};
struct Result {
    float before_soft=NAN,before_hard=NAN,soft=NAN,hard=NAN;
    bool hard_changed=false;
    const char* failure="unverified";
};
// IO implements read/write float, idleReady, awaitReady. No arming, clearing,
// saving or reboot action is available to this policy.
template<class IO> bool apply(IO& io,Map map,Result& r) {
    r={};
    if (!map.soft || !map.hard || !io.idleReady()) {r.failure="fresh IDLE required";return false;}
    if (!io.read(map.soft,r.before_soft) || !io.read(map.hard,r.before_hard) ||
        !isfinite(r.before_soft) || !isfinite(r.before_hard) || !isfinite(r.before_soft) || !isfinite(r.before_hard) || r.before_soft<=0 || r.before_hard<=0) {r.failure="current limit read failed";return false;}
    // Lower an excessive soft limit before lowering the hard limit. If the
    // existing soft limit is below target, raise it only after hard is set.
    if (r.before_soft>SOFT_AMPS &&
        (!io.idleReady() || !io.write(map.soft,SOFT_AMPS))) {r.failure="soft limit write failed";return false;}
    if (!close(r.before_hard,HARD_AMPS)) {
        if (!io.idleReady() || !io.write(map.hard,HARD_AMPS)) {r.failure="hard limit write failed";return false;}
        r.hard_changed=true;
    }
    // Hard-limit changes can briefly assert INITIALIZING; require the normal
    // fresh error/heartbeat checks to settle without clearing any error.
    if (!io.awaitReady(r.hard_changed)) {r.failure="ODrive not ready after limits";return false;}
    float verified_hard=NAN;
    if (!io.read(map.hard,verified_hard) || !close(verified_hard,HARD_AMPS)) {r.failure="50 A hard limit not verified";return false;}
    if (r.before_soft<SOFT_AMPS &&
        (!io.idleReady() || !io.write(map.soft,SOFT_AMPS))) {r.failure="soft limit write failed";return false;}
    if (!io.read(map.soft,r.soft) || !io.read(map.hard,r.hard) ||
        !close(r.soft,SOFT_AMPS) || !close(r.hard,HARD_AMPS) || !io.idleReady()) {
        r.failure="35 A / 50 A readback not verified";return false;
    }
    r.failure=nullptr;return true;
}
}
