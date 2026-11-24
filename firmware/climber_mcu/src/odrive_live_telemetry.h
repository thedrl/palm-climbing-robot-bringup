#pragma once

// ODrive CANSimple read-only telemetry. No allocation, delay, synchronous CAN
// request, CAN initialization/reset, state change, or setpoint is allowed here.
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "odrive_live_usb.h"

namespace odrive_live {
constexpr uint32_t PERIOD_MS = 200;
constexpr uint32_t REQUEST_MS = 100; // Alternate VI and Iq: each gets 5 Hz.
constexpr uint32_t FRESH_MS = 600;
constexpr uint32_t HEARTBEAT_MS = 1000;
enum Link { UNINITIALIZED = 0, RUNNING = 1, STOPPED = 2, BUS_OFF = 3, RECOVERING = 4, UNKNOWN = 5 };
struct BusStatus { Link link; bool tx_free; bool error_free; };
struct Sample {
    float a = NAN, b = NAN;
    uint32_t at = 0;
    bool seen = false;
    int32_t age(uint32_t now) const { return seen ? int32_t(now - at) : -1; }
    bool fresh(uint32_t now) const { return seen && uint32_t(now - at) <= FRESH_MS; }
};

class Telemetry {
public:
    bool enabled = true; // GUI listens only; no periodic USB commands needed.
    bool ready = false; // Existing explicit CAN initialization must finish first.
    uint32_t sequence = 0;
    Sample vi, iq, encoder;
    uint32_t hb_at = 0, axis_error = 0;
    uint8_t axis_state = 0;
    bool hb_seen = false;

    void reset() {
        ready = false;
        usb_.clear();
        vi = Sample{}; iq = Sample{}; encoder = Sample{};
        hb_seen = false; hb_at = 0; axis_state = 0; axis_error = 0;
    }
    bool online(uint32_t now) const {
        return hb_seen && uint32_t(now - hb_at) < HEARTBEAT_MS;
    }
    void receive(uint32_t id, uint8_t len, const uint8_t* data, uint32_t now, uint8_t node) {
        if (len != 8 || !data || (id & ~uint32_t(0x7ff)) || (id >> 5) != node) return;
        const uint8_t command = id & 0x1f;
        if (command == 0x01) {
            memcpy(&axis_error, data, 4);
            axis_state = data[4]; hb_at = now; hb_seen = true;
            // ODrive may retain old Iq values in IDLE. Never label those live.
            if (axis_state != 8) iq = Sample{};
            return;
        }
        Sample* target = command == 0x17 ? &vi : command == 0x14 ? &iq : command == 0x09 ? &encoder : nullptr;
        if (!target) return;
        if (command == 0x14 && (!online(now) || axis_state != 8)) return;
        memcpy(&target->a, data, 4); memcpy(&target->b, data + 4, 4);
        target->seen = isfinite(target->a) && isfinite(target->b);
        target->at = now;
    }

    // IO must provide status(), inputPending(), sendRead(node, command), and
    // writeIfRoom(buffer, length). All implementations must be nonblocking.
    template<class IO>
    void tick(uint32_t now, int unit, uint8_t node, IO& io) {
        if (!enabled || io.inputPending()) { usb_.clear(); return; }
        const bool emit = uint32_t(now - last_emit_) >= PERIOD_MS;
        const bool poll = uint32_t(now - last_request_) >= REQUEST_MS;
        if (!emit && !poll) { usb_.pump(now, io); return; }
        const BusStatus bus = io.status();
        if (poll) {
            last_request_ = now; // Never accumulate overdue work or retry in a loop.
            const uint8_t command = next_iq_ ? 0x14 : 0x17;
            next_iq_ = !next_iq_;
            if (ready && bus.link == RUNNING && bus.tx_free && bus.error_free && online(now)
                && (command != 0x14 || axis_state == 8)) {
                io.sendRead(node, command); // Only read requests, with RTR set.
            }
        }
        if (!emit) { usb_.pump(now, io); return; }
        last_emit_ = now;
        const bool live = ready && bus.link == RUNNING && online(now);
        const bool bus_valid = live && vi.fresh(now);
        const bool iq_valid = live && axis_state == 8 && iq.fresh(now);
        const bool encoder_valid = live && encoder.fresh(now);
        char line[256];
        const int length = snprintf(line, sizeof(line),
            "\nODRIVE_LIVE,1,%d,%lu,%d,%d,%lu,%ld,%ld,%ld,%ld,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            unit, (unsigned long)++sequence, int(bus.link), live ? int(axis_state) : -1,
            (unsigned long)axis_error, hb_seen ? long(uint32_t(now - hb_at)) : -1L,
            long(vi.age(now)), long(iq.age(now)), long(encoder.age(now)),
            bus_valid ? vi.a : NAN, bus_valid ? vi.b : NAN,
            iq_valid ? iq.a : NAN, iq_valid ? iq.b : NAN,
            encoder_valid ? encoder.b * 60.0f : NAN);
        if (length > 0 && length < int(sizeof(line))) usb_.offer(now, unit, sequence, line, length);
        usb_.pump(now, io);
    }
private:
    UsbSnapshot usb_;
    uint32_t last_emit_ = 0, last_request_ = 0;
    bool next_iq_ = false;
};
} // namespace odrive_live
