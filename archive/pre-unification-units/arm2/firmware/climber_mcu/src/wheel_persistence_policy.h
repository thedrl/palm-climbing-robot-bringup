#pragma once
#include <stdint.h>
#include <math.h>
namespace wheel_persistence {
// Official 0.6.11/-1 and 0.6.12/-1 S1 descriptors, verified in
// endpoint-evidence.json. Use only after wheel_current::supported succeeds.
inline uint16_t startupFirst(uint8_t revision) {
    return revision==11 ? 219 : revision==12 ? 238 : 0;
}
inline bool validSnapshot(float soft, float hard, const uint8_t flags[5]) {
    if (!isfinite(soft) || !isfinite(hard) || soft!=35.0f || hard!=50.0f) return false;
    for (unsigned i=0;i<5;++i) if (flags[i]!=0) return false;
    return true;
}
// A bool response has four header bytes and one value byte. Reject padded,
// malformed, stale-endpoint, and non-canonical bool responses, never coerce
// an unverified read to false. The existing float/U32 SDO parser is untouched.
struct BoolSdo {
    bool waiting=false, received=false;
    uint8_t node=0, value=255;
    uint16_t endpoint=0;
    void cancel() {waiting=false;received=false;value=255;}
    void observe(uint32_t id,uint8_t len,const uint8_t* data) {
        if (!waiting || received || !data || len!=5 ||
            id!=((uint32_t(node)<<5)|5u) ||
            (uint16_t(data[1])|(uint16_t(data[2])<<8))!=endpoint || data[4]>1) return;
        value=data[4];received=true;
    }
    template<class IO> bool read(IO& io,uint8_t n,uint16_t ep,uint8_t& result) {
        cancel();
        if (io.cancelled() || !io.drain() || io.cancelled()) return false;
        node=n;endpoint=ep;waiting=true;
        const uint8_t data[4]={0,uint8_t(ep),uint8_t(ep>>8),0};
        if (!io.send((uint32_t(n)<<5)|4u,4,data)) {cancel();return false;}
        const uint32_t started=io.now();
        while (uint32_t(io.now()-started)<150) {
            if (io.cancelled()) {cancel();return false;}
            io.pump();
            if (io.cancelled()) {cancel();return false;}
            if (received) {result=value;waiting=false;return true;}
            io.pause();
        }
        cancel();return false;
    }
};
}
