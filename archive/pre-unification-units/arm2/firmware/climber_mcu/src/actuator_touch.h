#pragma once
#include "actuator_bench_guard.h"
#include <string.h>

// Fixed bench envelope for unit 2. These are test bounds, not motor ratings or
// calibrated force/travel limits. No reverse command is used in this routine.
namespace actuator_touch {
static constexpr uint16_t TARGET_MM = 236;
static constexpr uint16_t MAX_START_MM = 336;
static constexpr int MAX_POWER = 40;
static constexpr uint32_t RANGE_AGE_MS = 120;
static constexpr uint32_t TELEMETRY_AGE_MS = 80;
static constexpr uint32_t COMMAND_AGE_MS = 120;
static constexpr uint32_t TIMEOUT_MS = 8000;
static constexpr int MAX_ENCODER_TRAVEL = 800;
static constexpr uint32_t REVERSE_RELEASE_MS = 1200;

enum class RangeState { pending, ready, error };
struct Range {
    RangeState state = RangeState::pending;
    uint16_t mm = 0;
    uint8_t status = 255;
    uint32_t at_ms = 0;
};

inline actuator_bench::Config envelope() {
    actuator_bench::Config c;
    // Operator-requested 15 V trial; the identified motor is nominally 12 V.
    // This sampled trip is not a continuous-current or contact-force rating.
    c.trip_a = 2.0; c.min_v = 14.0; c.max_v = 15.5;
    return c;
}

inline const char* rangeFault(const Range& r, uint32_t now) {
    if (r.state == RangeState::error) return "tof_transport_error";
    if (r.state != RangeState::ready) return nullptr;
    if (r.status != 0 || r.mm < 30 || r.mm > 1000) return "invalid_tof";
    if (now - r.at_ms >= RANGE_AGE_MS) return "stale_tof";
    return nullptr;
}

inline const char* sampleFault(const actuator_bench::Sample& s, uint32_t now, bool idle) {
    const char* fault = actuator_bench::healthy(s, envelope());
    if (fault) return fault;
    if (now - s.at_ms >= TELEMETRY_AGE_MS) return "stale_telemetry";
    if (s.forward_limit) return "limit_pressed";
    if (s.pwm1 < 0) return "reverse_pwm_unexpected";
    if (idle && (s.pwm1 != 0 || abs((int)s.current1) > 25)) return "not_idle";
    return nullptr;
}

struct Result {
    const char* reason = "not_started";
    bool stopped = false;
    bool threshold_seen = false;
    bool moved_commanded = false;
    const char* settle_fault = nullptr;
    bool output_verified = false;
    uint16_t last_mm = 0;
    uint32_t last_range_at_ms = 0;
    int64_t encoder_delta = 0;
};

// IO keeps the hardware adapter small; tests execute this exact control loop
// with deterministic sensor, serial, limit, watchdog, and current failures.
template <class IO> Result run(IO& io) {
    Result result;
    actuator_bench::Sample sample;
    Range last_range;
    int32_t start_encoder = 0;
    bool have_sample = false, have_range = false, sensor_started = false;
    bool sample_captured = false;
    uint32_t last_sample_ms = io.now(), last_range_ms = io.now();
    uint32_t last_command = io.now(), began = io.now();
    int applied = 0;
    bool reverse_released = !io.reverseLimit();
    uint32_t motion_started = 0;

    auto cancelled = [&]() -> const char* {
        if (io.cancelled()) return "operator_or_usb_stop";
        if (io.limit()) return "limit_pressed";
        // Forward-only departure may start at REV, but must leave that switch.
        if (!io.reverseLimit()) reverse_released = true;
        else if (reverse_released) return "reverse_limit_reasserted";
        else if (result.moved_commanded && io.now() - motion_started >= REVERSE_RELEASE_MS)
            return "reverse_limit_not_released";
        return nullptr;
    };
    auto read_cycle = [&](bool idle) -> const char* {
        if (const char* f = cancelled()) return f;
        uint32_t at = io.now();
        Range r = io.range();
        if (const char* f = cancelled()) return f;
        if (io.now() - at > 40) return "slow_tof_io";
        if (const char* f = rangeFault(r, io.now())) return f;
        if (r.state == RangeState::ready) {
            if (have_range && r.at_ms == last_range.at_ms) return "repeated_tof_sample";
            if (have_range && abs((int)r.mm - last_range.mm) > 20) return "tof_jump";
            have_range = true; last_range = r; last_range_ms = r.at_ms;
            result.last_mm = r.mm;
            result.last_range_at_ms = r.at_ms;
            // A raw close reading stops immediately. Do not wait for averaging
            // or drive again if a later noisy reading rises above the target.
            if (r.mm <= TARGET_MM) return "tof_threshold_seen";
        }
        if (io.now() - last_range_ms >= RANGE_AGE_MS) return "stale_tof";
        if (!have_sample || io.now() - last_sample_ms >= 50) {
            sample = io.sample(); last_sample_ms = sample.at_ms;
            sample_captured = true;
            if (have_sample && sample.encoder_ok) result.encoder_delta = (int64_t)sample.encoder - start_encoder;
            if (const char* f = cancelled()) return f;
            if (const char* f = sampleFault(sample, io.now(), idle)) return f;
            if (!have_sample) start_encoder = sample.encoder;
            if (idle && llabs((int64_t)sample.encoder - start_encoder) > 1) return "baseline_encoder_motion";
            have_sample = true;
        }
        if (have_sample && io.now() - sample.at_ms >= TELEMETRY_AGE_MS) return "stale_telemetry";
        return nullptr;
    };

    const char* reason = cancelled();
    // Always acknowledge a direct-drive zero before configuration or sensing.
    if (!io.stop()) { result.reason = "stop_unacknowledged_cut_power"; return result; }
    result.stopped = true;
    if (!reason && !io.watchdog()) reason = "watchdog_not_acknowledged";
    if (!reason) {
        sensor_started = true; // Cleanup even a partially failed sensor start.
        if (!io.startSensor()) reason = "tof_start_failed";
    }
    began = last_range_ms = last_sample_ms = io.now();
    unsigned count = 0;
    uint16_t baseline_min = 65535, baseline_max = 0;
    uint32_t counted_ms = 0;
    while (!reason && count < 5) {
        if (io.now() - began >= 1500) { reason = "baseline_timeout"; break; }
        reason = read_cycle(true);
        if (have_range && (!count || last_range.at_ms != counted_ms)) {
            counted_ms = last_range.at_ms; ++count;
            if (last_range.mm < baseline_min) baseline_min = last_range.mm;
            if (last_range.mm > baseline_max) baseline_max = last_range.mm;
        }
        if (!reason && baseline_max - baseline_min > 8 && count) reason = "unstable_tof_baseline";
        if (!reason && have_range && last_range.mm > MAX_START_MM) reason = "start_more_than_100mm_from_target";
        io.pause();
    }
    if (!reason && !have_sample) reason = "missing_baseline_telemetry";
    uint16_t progress_mm = last_range.mm;
    int64_t progress_ticks = 0;
    uint32_t encoder_progress_ms = io.now(), range_progress_ms = io.now();
    uint32_t last_log = io.now();
    began = last_command = io.now();

    while (!reason) {
        if (io.now() - began >= TIMEOUT_MS) { reason = "approach_timeout"; break; }
        if (result.moved_commanded && io.now() - last_command >= COMMAND_AGE_MS) {
            reason = "command_age"; break;
        }
        reason = read_cycle(false);
        if (reason) break;
        result.encoder_delta = (int64_t)sample.encoder - start_encoder;
        if (result.encoder_delta < -2) { reason = "wrong_encoder_direction"; break; }
        if (result.encoder_delta >= MAX_ENCODER_TRAVEL) { reason = "encoder_travel_bound"; break; }
        if (last_range.mm > baseline_max + 10) { reason = "distance_increasing"; break; }
        if (result.encoder_delta >= progress_ticks + 2) {
            progress_ticks = result.encoder_delta; encoder_progress_ms = io.now();
        }
        if ((int)last_range.mm <= (int)progress_mm - 5) {
            progress_mm = last_range.mm; range_progress_ms = io.now();
        }
        if (io.now() - encoder_progress_ms >= 1200) { reason = "encoder_stall"; break; }
        if (io.now() - range_progress_ms >= 1500) { reason = "tof_no_approach_progress"; break; }
        if (io.now() - began >= TIMEOUT_MS) { reason = "approach_timeout"; break; }
        if (result.moved_commanded && io.now() - last_command >= COMMAND_AGE_MS) {
            reason = "command_age"; break;
        }
        if (!result.moved_commanded || io.now() - last_command >= 50) {
            if (const char* f = cancelled()) { reason = f; break; }
            if (io.now() - last_range.at_ms >= RANGE_AGE_MS) { reason = "stale_tof"; break; }
            applied = applied + 2 > MAX_POWER ? MAX_POWER : applied + 2;
            last_command = io.now();
            if (!result.moved_commanded) motion_started = last_command;
            result.moved_commanded = true; result.stopped = false;
            if (!io.drive(applied)) { reason = "drive_not_acknowledged"; break; }
            if (io.now() - last_command > 40) { reason = "slow_drive_acknowledgement"; break; }
        }
        if (io.now() - last_log >= 100) {
            io.log("approach", last_range, sample, applied); last_log = io.now();
        }
        io.pause();
    }

    result.reason = reason;
    result.threshold_seen = reason && !strcmp(reason, "tof_threshold_seen");
    // Stop before printing, sensor cleanup, or further telemetry.
    result.stopped = io.stop();
    // Preserve the triggering sample, but only print it after sending zero.
    if (sample_captured) io.log("at_stop", last_range, sample, applied);
    if (!result.stopped) {
        result.reason = "stop_unacknowledged_cut_power";
        if (sensor_started) io.endSensor();
        return result; // No RoboClaw reads after a failed stop.
    }
    io.event(result.reason);
    uint32_t settle_began = io.now();
    int32_t first_settled = 0;
    bool have_settled = false;
    while (sensor_started && io.now() - settle_began < 500) {
        if (io.cancelled()) { result.settle_fault = "settle_cancelled"; break; }
        auto s = io.sample();
        if (have_sample && s.encoder_ok) result.encoder_delta = (int64_t)s.encoder - start_encoder;
        const char* f = actuator_bench::healthy(s, envelope());
        if (!f && (s.pwm1 || abs((int)s.current1) > 25)) f = "output_not_zero_after_stop";
        if (!f && io.now() - s.at_ms >= TELEMETRY_AGE_MS) f = "stale_settle_telemetry";
        if (!f && io.now() - settle_began >= 250) {
            if (have_settled && llabs((int64_t)s.encoder - first_settled) > 1) f = "encoder_not_settled";
            if (!have_settled) first_settled = s.encoder;
            have_settled = true;
        }
        if (f) {
            result.settle_fault = f; result.stopped = io.stop();
            io.log("settle_fault", last_range, s, 0);
            if (!result.stopped) result.reason = "stop_unacknowledged_cut_power";
            break;
        }
        Range r = io.range();
        if (const char* f = rangeFault(r, io.now())) { result.settle_fault = f; break; }
        if (r.state == RangeState::ready) {
            last_range = r; result.last_mm = r.mm; result.last_range_at_ms = r.at_ms;
        }
        result.encoder_delta = (int64_t)s.encoder - start_encoder;
        io.log("stopped", last_range, s, 0);
        for (unsigned n = 0; n < 5; ++n) io.pause();
    }
    result.output_verified = have_settled && !result.settle_fault && result.stopped;
    if (sensor_started) io.endSensor();
    return result;
}
}
