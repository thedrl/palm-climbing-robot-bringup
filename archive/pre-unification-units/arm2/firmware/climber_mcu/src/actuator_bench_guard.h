#pragma once

#include <stdint.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>

namespace actuator_bench {
static constexpr uint32_t SAMPLE_PERIOD_MS = 50;
static constexpr uint32_t SAMPLE_BUDGET_MS = 80;
static constexpr uint32_t COMMAND_AGE_MS = 120;
static constexpr uint32_t MOTION_TIMEOUT_MS = 2000;
static constexpr uint32_t STALL_TIMEOUT_MS = 1250;
static constexpr uint32_t ARM_LIFETIME_MS = 30000;
static constexpr double MAX_STEP_MM = 2.5;

struct Config {
    double ticks_per_mm = 0;
    int forward_sign = 0;
    double trip_a = 0;
    double min_v = 0;
    double max_v = 0;
    int power = 0;
};

inline bool validConfig(const Config& c) {
    return isfinite(c.ticks_per_mm) && c.ticks_per_mm >= 1 && c.ticks_per_mm <= 10000 &&
        (c.forward_sign == 1 || c.forward_sign == -1) &&
        isfinite(c.trip_a) && c.trip_a >= 0.1 && c.trip_a <= 7 &&
        isfinite(c.min_v) && isfinite(c.max_v) && c.min_v >= 6 &&
        c.max_v > c.min_v && c.max_v <= 34 && c.power >= 1 && c.power <= 20;
}

inline bool numbers(const char* text, double* values, unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
        while (isspace((unsigned char)*text)) ++text;
        char* end = nullptr;
        errno = 0;
        values[i] = strtod(text, &end);
        if (end == text || errno || !isfinite(values[i]) || (*end && !isspace((unsigned char)*end))) return false;
        text = end;
    }
    while (isspace((unsigned char)*text)) ++text;
    return *text == 0;
}

inline bool parseConfig(const char* text, Config* result) {
    double v[6];
    if (!numbers(text, v, 6) || (v[1] != -1 && v[1] != 1) || v[5] < 1 || v[5] > 20 || floor(v[5]) != v[5]) return false;
    Config c;
    c.ticks_per_mm = v[0]; c.forward_sign = (int)v[1]; c.trip_a = v[2];
    c.min_v = v[3]; c.max_v = v[4]; c.power = (int)v[5];
    if (!validConfig(c)) return false;
    *result = c;
    return true;
}

struct Sample {
    uint32_t at_ms = 0, read_ms = 0;
    int32_t encoder = 0;
    int16_t current1 = 0, current2 = 0, pwm1 = 0, pwm2 = 0;
    uint16_t voltage10 = 0;
    uint32_t status = 0;
    bool encoder_ok = false, current_ok = false, voltage_ok = false, status_ok = false, pwm_ok = false;
    bool forward_limit = false, reverse_limit = false;
};

inline const char* healthy(const Sample& s, const Config& c) {
    if (!s.encoder_ok || !s.current_ok || !s.voltage_ok || !s.status_ok || !s.pwm_ok) return "invalid_telemetry";
    if (s.read_ms > SAMPLE_BUDGET_MS) return "slow_telemetry";
    if (s.status & ~0x03000000UL) return "controller_status";
    if (s.forward_limit && s.reverse_limit) return "conflicting_limits";
    double voltage = s.voltage10 / 10.0;
    if (voltage < c.min_v || voltage > c.max_v) return "voltage_outside_window";
    if (abs((int)s.current1) >= c.trip_a * 100.0) return "current_trip";
    if (s.pwm2 != 0 || abs((int)s.current2) > 25) return "unused_m2_not_idle";
    return nullptr;
}

struct Guard {
    Config config;
    int32_t start = 0, target = 0;
    int direction = 0, encoder_direction = 0;
    int64_t needed = 0, last_progress = 0;
    uint32_t began_ms = 0, last_progress_ms = 0;
    bool active = false;

    const char* begin(const Config& c, double delta_mm, const Sample& s, uint32_t now) {
        active = false;
        if (!validConfig(c)) return "invalid_configuration";
        if (!isfinite(delta_mm) || delta_mm == 0 || fabs(delta_mm) > MAX_STEP_MM) return "step_outside_2_5mm_limit";
        const char* fault = healthy(s, c);
        if (fault) return fault;
        if (now - s.at_ms > SAMPLE_BUDGET_MS) return "stale_telemetry";
        if (s.pwm1 != 0 || abs((int)s.current1) > 25) return "m1_not_idle_or_offset_suspect";
        direction = delta_mm > 0 ? 1 : -1;
        encoder_direction = direction * c.forward_sign;
        if (direction > 0 ? s.forward_limit : s.reverse_limit) return "direction_limit";
        needed = llround(fabs(delta_mm) * c.ticks_per_mm);
        if (needed < 2) return "step_below_encoder_resolution";
        int64_t target64 = (int64_t)s.encoder + encoder_direction * needed;
        if (target64 < INT32_MIN || target64 > INT32_MAX) return "encoder_overflow";
        config = c; start = s.encoder; target = (int32_t)target64;
        began_ms = last_progress_ms = now;
        last_progress = 0; active = true;
        return nullptr;
    }

    // A non-null reason always means stop. Only "target_reached" permits settling verification.
    const char* evaluate(const Sample& s, uint32_t now, bool cancelled, uint32_t command_age) {
        if (!active) return "not_active";
        const char* fault = cancelled ? "operator_or_usb_stop" : healthy(s, config);
        if (!fault && now - s.at_ms > SAMPLE_BUDGET_MS) fault = "stale_telemetry";
        if (!fault && now - began_ms >= MOTION_TIMEOUT_MS) fault = "motion_timeout";
        if (!fault && command_age >= COMMAND_AGE_MS) fault = "command_age";
        if (!fault && (direction > 0 ? s.forward_limit : s.reverse_limit)) fault = "direction_limit";
        int64_t progress = ((int64_t)s.encoder - start) * encoder_direction;
        if (!fault && progress < -1) fault = "wrong_encoder_direction";
        if (!fault && progress > needed + 1) fault = "target_overshoot";
        if (!fault && progress >= needed) fault = "target_reached";
        if (!fault && progress > last_progress) { last_progress = progress; last_progress_ms = now; }
        if (!fault && now - last_progress_ms >= STALL_TIMEOUT_MS) fault = "encoder_stall";
        if (fault) active = false;
        return fault;
    }

    bool settledAtTarget(const Sample& first, const Sample& last) const {
        return !healthy(first, config) && !healthy(last, config) &&
            first.pwm1 == 0 && last.pwm1 == 0 &&
            llabs((int64_t)last.encoder - first.encoder) <= 1 &&
            llabs((int64_t)last.encoder - target) <= 1;
    }
};
}
