#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace wheel_tuning {

constexpr float VEL_GAIN = 0.5f;
constexpr float VEL_INTEGRATOR_GAIN = 2.5f;
constexpr float VEL_INTEGRATOR_LIMIT = 1.2f;
constexpr float VEL_LIMIT_TOLERANCE = 2.5f;
constexpr float VEL_RAMP_RATE = 10.0f;
constexpr float SPINOUT_MECH_THRESH = -80.0f;
constexpr float SPINOUT_ELEC_THRESH = 80.0f;

// Verified ODrive S1 0.6.11 and 0.6.12 endpoint IDs gated by commit hash.
struct Map {
    uint16_t vel_gain = 0;
    uint16_t vel_integrator_gain = 0;
    uint16_t vel_integrator_limit = 0;
    uint16_t vel_limit_tolerance = 0;
    uint16_t vel_ramp_rate = 0;
    uint16_t spinout_mech_thresh = 0;
    uint16_t spinout_elec_thresh = 0;
    uint16_t dc_undervoltage = 0;
    uint16_t dc_overvoltage = 0;
    uint16_t dc_max_pos_current = 0;
    uint16_t dc_max_neg_current = 0;
    uint16_t brake_resistor_enable = 0;
    uint16_t comm_offset_valid = 0;
    uint16_t rs485_status = 0;

    Map() = default;
    constexpr Map(uint16_t vg, uint16_t vig, uint16_t vil, uint16_t vlt, uint16_t vrr,
                  uint16_t smt, uint16_t set, uint16_t duv, uint16_t dov,
                  uint16_t dpos, uint16_t dneg, uint16_t brk, uint16_t cov, uint16_t rs)
        : vel_gain(vg), vel_integrator_gain(vig), vel_integrator_limit(vil),
          vel_limit_tolerance(vlt), vel_ramp_rate(vrr), spinout_mech_thresh(smt),
          spinout_elec_thresh(set), dc_undervoltage(duv), dc_overvoltage(dov),
          dc_max_pos_current(dpos), dc_max_neg_current(dneg),
          brake_resistor_enable(brk), comm_offset_valid(cov), rs485_status(rs) {}
};

inline bool supported(uint8_t protocol, uint8_t hw_major, uint8_t hw_minor,
        uint8_t hw_variant, uint8_t major, uint8_t minor, uint8_t revision,
        uint8_t unreleased, uint32_t commit, Map& out) {
    out = {};
    if (protocol != 2 || hw_major != 5 || hw_minor != 2 || hw_variant != 0 ||
        major != 0 || minor != 6 || unreleased) {
        return false;
    }
    if (revision == 11 && (commit == 0xc063d552u || commit == 0xb3c4cf45u)) {
        out = {371, 372, 373, 375, 376, 390, 391, 161, 162, 163, 164, 195, 458, 601};
    } else if (revision == 12 && (commit == 0xd92a33feu || commit == 0x5de70d7bu)) {
        out = {392, 393, 394, 397, 398, 412, 413, 166, 167, 168, 169, 212, 480, 629};
    }
    return out.vel_gain != 0;
}

struct ParamSpec {
    const char* name;
    uint16_t Map::*field;
    float min_val;
    float max_val;
};

inline bool lookupParam(const char* name, const Map& map,
                        uint16_t& endpoint_out, float& min_out, float& max_out) {
    if (!name || !*name) return false;
    static constexpr ParamSpec SPECS[] = {
        {"vel_gain", &Map::vel_gain, 0.05f, 2.0f},
        {"vel_integrator_gain", &Map::vel_integrator_gain, 0.0f, 15.0f},
        {"vel_integrator_limit", &Map::vel_integrator_limit, 0.1f, 3.0f},
        {"vel_limit_tolerance", &Map::vel_limit_tolerance, 1.1f, 4.0f},
        {"vel_ramp_rate", &Map::vel_ramp_rate, 0.5f, 50.0f},
        {"spinout_elec", &Map::spinout_elec_thresh, 10.0f, 300.0f},
        {"spinout_mech", &Map::spinout_mech_thresh, -300.0f, -10.0f},
        {"dc_undervoltage", &Map::dc_undervoltage, 30.0f, 44.0f},
        {"dc_overvoltage", &Map::dc_overvoltage, 51.0f, 57.0f},
        {"dc_max_pos_current", &Map::dc_max_pos_current, 5.0f, 60.0f},
        {"dc_max_neg_current", &Map::dc_max_neg_current, -15.0f, -0.01f},
    };
    for (const auto& spec : SPECS) {
        if (strcmp(name, spec.name) == 0) {
            const uint16_t ep = map.*(spec.field);
            if (ep == 0) return false;
            endpoint_out = ep;
            min_out = spec.min_val;
            max_out = spec.max_val;
            return true;
        }
    }
    return false;
}

inline bool validateValue(float value, float min_val, float max_val) {
    return isfinite(value) && value >= min_val && value <= max_val;
}

inline bool closeEnough(float actual, float target) {
    return isfinite(actual) && isfinite(target) && fabsf(actual - target) <= 1e-3f;
}

// Single-byte CANSimple SDO read observer for uint8 endpoints (e.g. rs485 status).
struct ByteSdo {
    bool waiting = false, received = false;
    uint8_t node = 0, value = 255;
    uint16_t endpoint = 0;

    void cancel() { waiting = false; received = false; value = 255; }

    void observe(uint32_t id, uint8_t len, const uint8_t* data) {
        if (!waiting || received || !data || len != 5 ||
            id != ((uint32_t(node) << 5) | 5u) ||
            (uint16_t(data[1]) | (uint16_t(data[2]) << 8)) != endpoint) {
            return;
        }
        value = data[4];
        received = true;
    }

    template<class IO>
    bool read(IO& io, uint8_t n, uint16_t ep, uint8_t& result) {
        cancel();
        if (io.cancelled() || !io.drain() || io.cancelled()) return false;
        node = n;
        endpoint = ep;
        waiting = true;
        const uint8_t data[4] = {0, uint8_t(ep), uint8_t(ep >> 8), 0};
        if (!io.send((uint32_t(n) << 5) | 4u, 4, data)) {
            cancel();
            return false;
        }
        const uint32_t started = io.now();
        while (uint32_t(io.now() - started) < 150) {
            if (io.cancelled()) { cancel(); return false; }
            io.pump();
            if (io.cancelled()) { cancel(); return false; }
            if (received) {
                result = value;
                waiting = false;
                return true;
            }
            io.pause();
        }
        cancel();
        return false;
    }
};

}  // namespace wheel_tuning
