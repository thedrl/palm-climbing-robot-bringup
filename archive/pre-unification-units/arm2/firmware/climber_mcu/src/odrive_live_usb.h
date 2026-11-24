#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace odrive_live {
// Arduino Nano ESP32's bundled TinyUSB TX buffer holds only 64 bytes.
// Every piece is a complete line <=63 bytes, including both newlines.
// Keep one snapshot only; discard old/incomplete data instead of blocking.
class UsbSnapshot {
public:
    explicit UsbSnapshot(const char* prefix = "ODRIVE_PART") : prefix_(prefix) {}
    enum { FRAGMENT_BYTES = 24, RECORD_BYTES = 256, MAX_AGE_MS = 200 };
    void clear() { length_ = 0; part_ = 0; }
    bool pending() const { return length_ != 0; }
    void offer(uint32_t now, int unit, uint32_t sequence, const char* text, int length) {
        clear();
        if (!text || unit < 0 || unit > 3 || length <= 0 || length >= RECORD_BYTES) return;
        while (length && (*text == '\n' || *text == '\r')) { ++text; --length; }
        while (length && (text[length-1] == '\n' || text[length-1] == '\r')) --length;
        if (!length) return;
        memcpy(record_, text, length); record_[length] = 0;
        length_ = length; unit_ = unit; sequence_ = sequence; started_ = now;
    }
    template<class IO> void pump(uint32_t now, IO& io) {
        if (!pending()) return;
        const uint32_t age = uint32_t(now - started_);
        if (age >= MAX_AGE_MS) { clear(); return; }
        const int offset = part_ * FRAGMENT_BYTES;
        const int remaining = length_ - offset;
        const int count = remaining < FRAGMENT_BYTES ? remaining : FRAGMENT_BYTES;
        const int total = (length_ + FRAGMENT_BYTES - 1) / FRAGMENT_BYTES;
        char piece[64];
        const int n = snprintf(piece, sizeof(piece), "\n%s,1,%d,%lu,%d,%d,%lu,%.*s\n",
            prefix_, unit_, (unsigned long)sequence_, part_, total, (unsigned long)age, count, record_ + offset);
        if (n <= 0 || n >= int(sizeof(piece))) { clear(); return; }
        // One bounded write per pump call; never wait for USB room or retry here.
        if (io.writeIfRoom(piece, n)) {
            ++part_;
            if (part_ == total) clear();
        }
    }
private:
    const char* prefix_;
    char record_[RECORD_BYTES] = {};
    int length_ = 0, part_ = 0, unit_ = 0;
    uint32_t sequence_ = 0, started_ = 0;
};
}
