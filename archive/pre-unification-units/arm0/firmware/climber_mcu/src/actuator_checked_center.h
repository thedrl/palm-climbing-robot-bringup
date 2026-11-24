#pragma once
#include "actuator_bench_guard.h"

namespace actuator_checked_center {
constexpr int MAX_COMMAND_POWER = 50;
constexpr uint32_t STOP_VERIFY_TIMEOUT_MS = 1600;
struct Config {
    int power = MAX_COMMAND_POWER, homing_power = 40, ramp_step = 8, forward_sign = 1;
    int32_t min_stroke = 1350, max_stroke = 1900, tolerance = 40;
};
struct Result {
    const char* reason = "not_started";
    const char* settle_fault = nullptr;
    bool complete = false, motion_commanded = false, stop_ack = false, output_verified = false;
    bool final_encoder_valid = false;
    int32_t fwd = 0, rev = 0, target = 0, final_encoder = 0, tolerance_ticks = 0;
};
inline actuator_bench::Config envelope() {
    actuator_bench::Config c;
    c.trip_a = 2.0; c.min_v = 12.0; c.max_v = 21.5;
    return c;
}

template <class IO> class Run {
    IO& io;
    Config c;
    Result result;
    actuator_bench::Sample sample;
    const char* fault = nullptr;

    const char* read() {
        sample = io.sample();
        if (sample.encoder_ok && sample.read_ms <= 60 && io.now() - sample.at_ms < 80) {
            result.final_encoder = sample.encoder;
            result.final_encoder_valid = true;
        }
        if (const char* f = actuator_bench::healthy(sample, envelope())) return f;
        if (sample.read_ms > 60 || io.now() - sample.at_ms >= 80) return "slow_or_stale_telemetry";
        return nullptr;
    }

    bool stopAndCheck(const char* phase, bool fault_snapshot = false) {
        result.output_verified = false;
        result.stop_ack = io.stop();
        if (!result.stop_ack) { fault = "stop_unacknowledged_cut_power"; return false; }
        if (fault_snapshot) io.log("fault_sample", sample, 0);
        const uint32_t began = io.now();
        // No telemetry during this interval: reads may feed the watchdog.
        while (io.now() - began < 250) {
            if (io.cancelled()) { fault = "operator_or_usb_stop"; return false; }
            io.pause();
        }
        unsigned count = 0;
        uint32_t first_at = 0, previous_at = 0;
        int32_t first_encoder = 0;
        bool have_previous = false;
        const char* unsettled = "stop_verification_timeout";
        // Allow a 1 s controller idle delay plus a complete quiet window.
        // Current, status and PWM faults still abort on the first fresh sample.
        while (io.now() - began < STOP_VERIFY_TIMEOUT_MS) {
            if (io.cancelled()) { fault = "operator_or_usb_stop"; return false; }
            const char* f = read();
            if (!f && have_previous && sample.at_ms == previous_at) f = "repeated_telemetry";
            io.log(phase, sample, 0);
            if (f) { fault = f; return false; }
            if (sample.pwm1) { fault = "output_not_idle"; return false; }
            have_previous = true;
            previous_at = sample.at_ms;
            // With zero PWM and healthy telemetry, allow residual current/coast
            // to settle, but require a new continuous quiet window before success.
            if (abs((int)sample.current1) > 25) {
                count = 0;
                unsettled = "output_not_idle";
            } else {
                if (count && llabs((int64_t)sample.encoder - first_encoder) > 1) {
                    count = 0;
                    unsettled = "encoder_not_settled";
                }
                if (!count) { first_encoder = sample.encoder; first_at = sample.at_ms; }
                ++count;
            }
            if (io.now() - began >= STOP_VERIFY_TIMEOUT_MS) break;
            if (count >= 5 && sample.at_ms - first_at >= 250) {
                result.output_verified = true;
                return true;
            }
            const uint32_t at = io.now();
            while (io.now() - at < 50) {
                if (io.cancelled()) { fault = "operator_or_usb_stop"; return false; }
                io.pause();
            }
        }
        fault = unsettled;
        return false;
    }

    bool leg(int direction, bool centering) {
        const char* phase = centering ? "center" : direction > 0 ? "fwd" : "rev";
        const int32_t start = sample.encoder;
        int32_t progress_encoder = start;
        const uint32_t began = io.now();
        uint32_t last_progress = began, last_command = began, ramp_at = began, first_drive = began;
        uint32_t previous_sample_at = sample.at_ms;
        bool drove = false;
        bool origin_released = !(direction > 0 ? io.rev() : io.fwd());
        int applied = 0;
        while (true) {
            if (io.cancelled()) { fault = "operator_or_usb_stop"; return false; }
            if (io.fwd() && io.rev()) { fault = "conflicting_limits"; return false; }
            if (direction > 0 ? io.fwd() : io.rev()) {
                if (centering) { fault = "limit_while_centering"; return false; }
                if (!stopAndCheck("limit_stopped")) return false;
                const uint32_t at = io.now();
                while (io.now() - at < 30) {
                    if (io.cancelled()) { fault = "operator_or_usb_stop"; return false; }
                    if (!(direction > 0 ? io.fwd() : io.rev())) { fault = "transient_limit_no_retry"; return false; }
                    io.pause();
                }
                return true;
            }
            const bool origin = direction > 0 ? io.rev() : io.fwd();
            if (!origin) origin_released = true;
            if (origin && origin_released) { fault = "origin_limit_reasserted"; return false; }
            if (origin && drove && io.now() - first_drive >= 1200) { fault = "origin_limit_not_released"; return false; }
            if (io.now() - began >= 8000) { fault = "phase_timeout"; return false; }
            if (drove && io.now() - last_command >= 120) { fault = "command_age"; return false; }
            if (const char* f = read()) { fault = f; return false; }
            if (io.cancelled()) { fault = "operator_or_usb_stop"; return false; }
            if (sample.at_ms == previous_sample_at) { fault = "repeated_telemetry"; return false; }
            previous_sample_at = sample.at_ms;
            if (io.now() - began >= 8000 || (drove && io.now() - last_command >= 120)) {
                fault = "phase_or_command_deadline"; return false;
            }
            if (direction > 0 ? io.fwd() : io.rev()) continue;
            const int64_t progress = ((int64_t)sample.encoder - start) * direction * c.forward_sign;
            if (progress < -2 || (sample.pwm1 && (sample.pwm1 > 0 ? 1 : -1) != direction)) {
                fault = "wrong_direction"; return false;
            }
            if (progress > c.max_stroke + c.tolerance) { fault = "travel_outside_stroke_bound"; return false; }
            const int64_t error = ((int64_t)result.target - sample.encoder) * c.forward_sign;
            if (centering && error <= c.tolerance) {
                if (!stopAndCheck("center_stopped")) return false;
                if (llabs((int64_t)sample.encoder - result.target) > c.tolerance) {
                    fault = "center_outside_tolerance_no_retry"; return false;
                }
                return true;
            }
            if (((int64_t)sample.encoder - progress_encoder) * direction * c.forward_sign >= 2) {
                progress_encoder = sample.encoder; last_progress = io.now();
            }
            if (io.now() - last_progress >= 1200) { fault = "encoder_stall_no_boost"; return false; }
            int target_power = centering ? c.power : c.homing_power;
            if (!drove || io.now() - ramp_at >= 60) {
                applied = applied < target_power ? (applied + c.ramp_step < target_power ? applied + c.ramp_step : target_power)
                                                : target_power;
                ramp_at = io.now();
            }
            if (!drove) { first_drive = io.now(); last_progress = first_drive; }
            // Mark even an unacknowledged attempt as commanded; it may have arrived.
            result.motion_commanded = true;
            if (!io.drive(direction * applied)) { fault = "drive_unacknowledged_or_limit"; return false; }
            drove = true; last_command = io.now();
            io.log(phase, sample, direction * applied);
            while (io.now() - last_command < 40) {
                if (io.cancelled() || (direction > 0 ? io.fwd() : io.rev())) break;
                io.pause();
            }
        }
    }

public:
    Run(IO& hardware, const Config& config) : io(hardware), c(config) {}
    Result run() {
        result.stop_ack = io.stop();
        if (!result.stop_ack) { result.reason = "stop_unacknowledged_cut_power"; return result; }
        if (io.cancelled()) fault = "operator_or_usb_stop";
        if (!fault && !io.watchdog()) fault = "watchdog_not_acknowledged";
        if (!fault && stopAndCheck("baseline") && leg(1, false)) {
            result.fwd = sample.encoder;
            if (leg(-1, false)) {
                result.rev = sample.encoder;
                const int64_t stroke = ((int64_t)result.fwd - result.rev) * c.forward_sign;
                result.target = (int32_t)((int64_t)result.fwd + ((int64_t)result.rev - result.fwd) / 2);
                io.range(result.fwd, result.rev, result.target);
                if (stroke < c.min_stroke || stroke > c.max_stroke) fault = "invalid_stroke_no_retry";
                else {
                    c.tolerance = (int32_t)(stroke / 10);
                    result.tolerance_ticks = c.tolerance;
                    if (leg(1, true)) {
                        result.reason = "coarse_center_verified"; result.complete = true; return result;
                    }
                }
            }
        }
        const char* original = fault ? fault : "unexpected_exit";
        if (result.stop_ack) {
            fault = nullptr;
            if (!stopAndCheck("fault_stopped", true)) result.settle_fault = fault;
        }
        result.reason = result.stop_ack ? original : "stop_unacknowledged_cut_power";
        return result;
    }
};
}
