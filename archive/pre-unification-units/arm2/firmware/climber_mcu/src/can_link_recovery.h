#pragma once
#include <stdint.h>
namespace can_link {
enum Driver { OFF, RUNNING, STOPPED, BUS_OFF, RECOVERING, UNKNOWN };
enum Phase { INACTIVE, BACKOFF, CONNECTING, ZERO, IDLE, CONFIRM_IDLE, READY };
enum Event { LINK_LOST, ATTEMPT_FAILED, CONNECTED, RELEASE_REQUIRED };
struct Status {
    Driver driver = OFF;
    uint32_t tx_errors = 0, rx_errors = 0;
    bool tx_free = true, heartbeat_fresh = false;
    uint8_t axis_state = 0;
    uint32_t heartbeat_sequence = 0;
    bool busUsable() const { return driver == RUNNING && tx_errors < 128 && rx_errors < 128; }
};
// No stored motion command, fault clearing, motor arming or blocking waits.
// IO may only restart/purge CAN, send discovery beacons, zero speed and IDLE.
class Recovery {
public:
    static constexpr uint32_t ATTEMPT_MS = 2500, RETRY_MS = 5000, SEND_MS = 50;
    Phase phase = INACTIVE;
    uint32_t attempts = 0, faults = 0, generation = 0;
    Status last_fault;
    bool needs_stop = false;
    bool ready() const { return phase == READY; }
    bool motionAllowed() const { return ready() && !needs_stop; }
    void request(uint32_t now) {
        if (phase == READY) return;
        phase = BACKOFF; attempted_ = false; attempted_at_ = now;
    }
    void disable() { phase = INACTIVE; needs_stop = ever_ready_; ++generation; }
    bool acknowledgeStop() {
        if (!ready()) return false;
        needs_stop = false;
        return true;
    }
    template<class IO> void tick(uint32_t now, const Status& s, IO& io) {
        if (phase == INACTIVE) return;
        if (phase == READY) {
            if (s.busUsable() && s.heartbeat_fresh) return;
            last_fault = s; ++faults; ++generation; needs_stop = true;
            io.captureFault(s);
            io.quiesce(); // Abort old TX frames before a cable/power return.
            phase = BACKOFF; attempted_ = false;
            io.report(LINK_LOST);
            return;
        }
        if (phase == BACKOFF) {
            if (attempted_ && uint32_t(now - attempted_at_) < RETRY_MS) return;
            attempted_ = true; attempted_at_ = now; ++attempts;
            if (!io.restartLink()) { io.report(ATTEMPT_FAILED); return; }
            phase = CONNECTING; last_send_ = now - SEND_MS;
            return;
        }
        if (!s.busUsable() || uint32_t(now - attempted_at_) >= ATTEMPT_MS
            || (phase != CONNECTING && !s.heartbeat_fresh)) {
            // Keep the first runtime failure; startup attempts are also observable.
            if (!ever_ready_) last_fault = s;
            io.quiesce(); phase = BACKOFF;
            io.report(ATTEMPT_FAILED); return;
        }
        if (phase == CONFIRM_IDLE) {
            if (s.heartbeat_fresh && s.axis_state == 1
                && s.heartbeat_sequence != idle_requested_after_) {
                phase = READY; ever_ready_ = true;
                io.report(needs_stop ? RELEASE_REQUIRED : CONNECTED);
            }
            return;
        }
        if (phase == CONNECTING && s.heartbeat_fresh) {
            phase = ZERO; last_send_ = now - SEND_MS;
        }
        if (!s.tx_free || uint32_t(now - last_send_) < SEND_MS) return;
        last_send_ = now;
        if (phase == CONNECTING) { io.sendBeacon(); return; }
        if (phase == ZERO) { if (io.sendZero()) phase = IDLE; return; }
        if (phase == IDLE && io.sendIdle()) {
            idle_requested_after_ = s.heartbeat_sequence;
            phase = CONFIRM_IDLE;
        }
    }
private:
    bool ever_ready_ = false, attempted_ = false;
    uint32_t attempted_at_ = 0, last_send_ = 0, idle_requested_after_ = 0;
};
}
