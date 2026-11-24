#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>

#include "wheel_current_limits.h"
#include "wheel_persistence_policy.h"
#include "wheel_tuning_policy.h"

namespace ODriveAxisState {
constexpr uint8_t AXIS_STATE_IDLE = 1;
constexpr uint8_t AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7;
constexpr uint8_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
}

struct Get_Error_msg_t {
    uint32_t Active_Errors = 0;
    uint32_t Disarm_Reason = 0;
};

struct Get_Version_msg_t {
    uint8_t Protocol_Version = 2;
    uint8_t Hw_Version_Major = 5;
    uint8_t Hw_Version_Minor = 2;
    uint8_t Hw_Version_Variant = 0;
    uint8_t Fw_Version_Major = 0;
    uint8_t Fw_Version_Minor = 6;
    uint8_t Fw_Version_Revision = 11;
    uint8_t Fw_Version_Unreleased = 0;
};

struct FakeSerial {
    int avail = 0;
    bool open = true;
    int available() const { return avail; }
    explicit operator bool() const { return open; }
    void print(const char*) {}
    void println(const char*) {}
    void printf(const char*, ...) {}
} Serial;

struct Beat {
    uint8_t Axis_State = ODriveAxisState::AXIS_STATE_IDLE;
    uint32_t Axis_Error = 0;
    uint8_t Procedure_Result = 0;
};

struct Data {
    uint32_t heartbeat_sequence = 1;
    Beat last_heartbeat;
} odrive_data[1];

constexpr int NUM_LOCAL_ODRIVES = 1;
constexpr uint8_t LOCAL_ODRIVE_NODE_ID = 0;

struct CanRecoveryStub {
    uint32_t generation = 1;
} can_recovery;

enum { TWAI_STATE_RUNNING = 1 };
enum { ESP_OK = 0 };
struct twai_status_info_t {
    int state = TWAI_STATE_RUNNING;
    uint32_t msgs_to_rx = 0;
    uint32_t msgs_to_tx = 0;
};

int can_intf = 0;
bool can_initialized = true;

static uint32_t tick = 0;
uint32_t millis() { return tick; }
void delay(unsigned n) { tick += n; }

struct Env {
    bool online = true;
    bool motion_allowed = true;
    bool version_ok = true;
    bool errors_ok = true;
    bool send_ok = true;
    bool respond_sdo = true;
    bool heartbeat = true;
    bool cal_enter_state7 = true;
    bool cal_never_finishes = false;
    bool cal_inject_error = false;
    bool cal_inject_serial_cancel = false;
    uint32_t cal_duration_ms = 50;
    uint32_t cal_error_bits = 0x00008000;
    uint8_t cal_proc_result = 0;
    uint8_t cal_post_offset_valid = 1;
    uint32_t active_errors = 0;
    uint32_t disarm_reason = 0;
    uint32_t commit = 0xb3c4cf45u;
    uint16_t mismatch_write_ep = 0;
    uint32_t cal_started_ms = 0;
    bool cal_in_progress = false;
    unsigned set_vel_zero_calls = 0;
    unsigned set_state_idle_calls = 0;
    unsigned set_state_cal_calls = 0;
    unsigned save_calls = 0;
    Get_Version_msg_t version;
    std::map<uint16_t, float> floats;
    std::map<uint16_t, uint8_t> bytes;
    std::vector<uint8_t> pending_sdo;
} e;

wheel_current::Sdo wheel_current_sdo;
wheel_persistence::BoolSdo wheel_persistence_bool_sdo;
wheel_tuning::ByteSdo wheel_tuning_byte_sdo;
bool wheel_limits_verified[NUM_LOCAL_ODRIVES] = {false};
uint32_t wheel_limits_verified_gen[NUM_LOCAL_ODRIVES] = {0};

int twai_get_status_info(twai_status_info_t* out) {
    out->state = TWAI_STATE_RUNNING;
    out->msgs_to_rx = 0;
    out->msgs_to_tx = 0;
    return ESP_OK;
}
void pumpEvents(int&) {}

bool odriveOnline(int) { return e.online; }
bool wheelMotionAllowed() { return e.motion_allowed; }
bool wheelStartCancelled(int index) {
    return Serial.available() > 0 || !Serial || !odriveOnline(index) || !wheelMotionAllowed();
}
void printOdriveErrorBits(uint32_t) {}

static void replySdo() {
    if (e.pending_sdo.empty() || !e.respond_sdo) return;
    const uint8_t opcode = e.pending_sdo[0];
    const uint16_t ep = uint16_t(e.pending_sdo[1]) | (uint16_t(e.pending_sdo[2]) << 8);
    e.pending_sdo.clear();
    if (opcode == 1) {
        float v = 0.0f;
        uint32_t bits = 0;
        // Read value from pending write? Already stored in sendMsg below.
        (void)v;
        (void)bits;
        uint8_t resp[4] = {1, uint8_t(ep), uint8_t(ep >> 8), 0};
        wheel_current_sdo.observe((uint32_t(LOCAL_ODRIVE_NODE_ID) << 5) | 5u, 4, resp);
        return;
    }
    if (ep == 13) {
        uint8_t resp[8] = {
            0, uint8_t(ep), uint8_t(ep >> 8), 0,
            uint8_t(e.commit), uint8_t(e.commit >> 8), uint8_t(e.commit >> 16), uint8_t(e.commit >> 24)
        };
        wheel_current_sdo.observe((uint32_t(LOCAL_ODRIVE_NODE_ID) << 5) | 5u, 8, resp);
        return;
    }
    if (e.bytes.count(ep)) {
        uint8_t resp[5] = {0, uint8_t(ep), uint8_t(ep >> 8), 0, e.bytes[ep]};
        const uint32_t id = (uint32_t(LOCAL_ODRIVE_NODE_ID) << 5) | 5u;
        wheel_persistence_bool_sdo.observe(id, 5, resp);
        wheel_tuning_byte_sdo.observe(id, 5, resp);
        return;
    }
    if (e.floats.count(ep)) {
        float val = e.floats[ep];
        uint32_t bits = 0;
        memcpy(&bits, &val, 4);
        uint8_t resp[8] = {
            0, uint8_t(ep), uint8_t(ep >> 8), 0,
            uint8_t(bits), uint8_t(bits >> 8), uint8_t(bits >> 16), uint8_t(bits >> 24)
        };
        wheel_current_sdo.observe((uint32_t(LOCAL_ODRIVE_NODE_ID) << 5) | 5u, 8, resp);
    }
}

void pumpCanOnce() {
    tick += 1;
    if (e.cal_in_progress) {
        const uint32_t elapsed = tick - e.cal_started_ms;
        if (e.cal_inject_serial_cancel && elapsed >= 10) {
            Serial.avail = 1;
        } else if (e.cal_inject_error && elapsed >= 15) {
            odrive_data[0].last_heartbeat.Axis_Error = e.cal_error_bits;
            ++odrive_data[0].heartbeat_sequence;
        } else if (!e.cal_never_finishes) {
            if (e.cal_enter_state7 && elapsed >= 10 && elapsed < e.cal_duration_ms) {
                odrive_data[0].last_heartbeat.Axis_State = ODriveAxisState::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                odrive_data[0].last_heartbeat.Procedure_Result = 1;
                ++odrive_data[0].heartbeat_sequence;
            } else if (elapsed >= e.cal_duration_ms) {
                odrive_data[0].last_heartbeat.Axis_State = ODriveAxisState::AXIS_STATE_IDLE;
                odrive_data[0].last_heartbeat.Procedure_Result = e.cal_proc_result;
                if (e.version.Fw_Version_Revision == 11) {
                    e.bytes[458] = e.cal_post_offset_valid;
                } else {
                    e.bytes[480] = e.cal_post_offset_valid;
                }
                e.cal_in_progress = false;
                ++odrive_data[0].heartbeat_sequence;
            }
        }
    } else if (e.heartbeat) {
        ++odrive_data[0].heartbeat_sequence;
    }
    replySdo();
}

void pumpCanFor(uint32_t duration_ms) {
    const uint32_t start = millis();
    while (millis() - start < duration_ms) {
        pumpCanOnce();
        delay(2);
    }
}

bool sendMsg(int&, uint32_t id, uint8_t len, const uint8_t* data) {
    if (!e.send_ok || !data || len < 4) return false;
    if (id != ((uint32_t(LOCAL_ODRIVE_NODE_ID) << 5) | 4u)) return false;
    e.pending_sdo.assign(data, data + len);
    if (data[0] == 1 && len == 8) {
        const uint16_t ep = uint16_t(data[1]) | (uint16_t(data[2]) << 8);
        uint32_t bits = uint32_t(data[4]) | (uint32_t(data[5]) << 8) |
                        (uint32_t(data[6]) << 16) | (uint32_t(data[7]) << 24);
        float v = 0.0f;
        memcpy(&v, &bits, 4);
        if (ep == e.mismatch_write_ep) {
            e.floats[ep] = v + 1.0f;
        } else {
            e.floats[ep] = v;
        }
    }
    return true;
}

struct ODriveCAN {
    enum ResetAction { SaveConfiguration = 1 };
    bool getVersion(Get_Version_msg_t& msg, int) { msg = e.version; return e.version_ok; }
    bool getError(Get_Error_msg_t& msg, int) {
        msg.Active_Errors = e.active_errors;
        msg.Disarm_Reason = e.disarm_reason;
        return e.errors_ok;
    }
    bool setVelocity(float v, float) {
        if (v == 0.0f) ++e.set_vel_zero_calls;
        return e.send_ok;
    }
    bool setState(uint8_t s) {
        if (s == ODriveAxisState::AXIS_STATE_IDLE) {
            ++e.set_state_idle_calls;
            e.cal_in_progress = false;
            odrive_data[0].last_heartbeat.Axis_State = ODriveAxisState::AXIS_STATE_IDLE;
            ++odrive_data[0].heartbeat_sequence;
        } else if (s == ODriveAxisState::AXIS_STATE_ENCODER_OFFSET_CALIBRATION) {
            ++e.set_state_cal_calls;
            e.cal_in_progress = true;
            e.cal_started_ms = tick;
        }
        return e.send_ok;
    }
    bool reset(ResetAction action) {
        assert(action == SaveConfiguration);
        ++e.save_calls;
        return e.send_ok;
    }
} drive;

ODriveCAN* odrives[1] = {&drive};

#include "wheel_current_limits_runtime.h"
#include "wheel_persistence_runtime.h"
#include "wheel_tuning_runtime.h"

static unsigned checks = 0;
#define CHECK(x) do { ++checks; assert(x); } while (0)

static void seedEndpoints(uint8_t rev) {
    e.floats.clear();
    e.bytes.clear();
    if (rev == 11) {
        e.version.Fw_Version_Revision = 11;
        e.commit = 0xb3c4cf45u;
        e.floats[305] = 35.0f; e.floats[306] = 38.0f;
        e.floats[371] = 0.1667f; e.floats[372] = 0.3333f; e.floats[373] = INFINITY;
        e.floats[375] = 1.4f; e.floats[376] = 30.0f;
        e.floats[390] = -10.0f; e.floats[391] = 10.0f;
        e.floats[161] = 38.0f; e.floats[162] = 54.0f;
        e.floats[163] = 25.0f; e.floats[164] = -2.0f;
        e.bytes[195] = 0; e.bytes[458] = 1; e.bytes[601] = 0;
        for (uint16_t ep = 219; ep < 224; ++ep) e.bytes[ep] = 0;
    } else {
        e.version.Fw_Version_Revision = 12;
        e.commit = 0xd92a33feu;
        e.floats[325] = 35.0f; e.floats[326] = 38.0f;
        e.floats[392] = 0.1667f; e.floats[393] = 0.3333f; e.floats[394] = INFINITY;
        e.floats[397] = 1.4f; e.floats[398] = 30.0f;
        e.floats[412] = -10.0f; e.floats[413] = 10.0f;
        e.floats[166] = 38.0f; e.floats[167] = 54.0f;
        e.floats[168] = 25.0f; e.floats[169] = -2.0f;
        e.bytes[212] = 0; e.bytes[480] = 1; e.bytes[629] = 0;
        for (uint16_t ep = 238; ep < 243; ++ep) e.bytes[ep] = 0;
    }
}

static void resetEnv(uint8_t rev = 11) {
    e = Env{};
    tick = 0;
    Serial.avail = 0;
    Serial.open = true;
    can_recovery.generation = 1;
    odrive_data[0] = Data{};
    wheel_limits_verified[0] = false;
    wheel_limits_verified_gen[0] = 0;
    wheel_current_sdo.cancel();
    wheel_persistence_bool_sdo.cancel();
    wheel_tuning_byte_sdo.cancel();
    seedEndpoints(rev);
}

static void testPolicy() {
    wheel_tuning::Map m;
    CHECK(wheel_tuning::supported(2, 5, 2, 0, 0, 6, 11, 0, 0xc063d552u, m));
    CHECK(m.vel_gain == 371 && m.vel_integrator_gain == 372 && m.vel_integrator_limit == 373);
    CHECK(m.vel_limit_tolerance == 375 && m.vel_ramp_rate == 376);
    CHECK(m.spinout_mech_thresh == 390 && m.spinout_elec_thresh == 391);
    CHECK(m.dc_undervoltage == 161 && m.dc_overvoltage == 162);
    CHECK(m.dc_max_pos_current == 163 && m.dc_max_neg_current == 164);
    CHECK(m.brake_resistor_enable == 195 && m.comm_offset_valid == 458 && m.rs485_status == 601);

    CHECK(wheel_tuning::supported(2, 5, 2, 0, 0, 6, 11, 0, 0xb3c4cf45u, m));
    CHECK(m.vel_gain == 371);

    CHECK(wheel_tuning::supported(2, 5, 2, 0, 0, 6, 12, 0, 0xd92a33feu, m));
    CHECK(m.vel_gain == 392 && m.vel_integrator_gain == 393 && m.vel_integrator_limit == 394);
    CHECK(m.vel_limit_tolerance == 397 && m.vel_ramp_rate == 398);
    CHECK(m.spinout_mech_thresh == 412 && m.spinout_elec_thresh == 413);
    CHECK(m.dc_undervoltage == 166 && m.dc_overvoltage == 167);
    CHECK(m.dc_max_pos_current == 168 && m.dc_max_neg_current == 169);
    CHECK(m.brake_resistor_enable == 212 && m.comm_offset_valid == 480 && m.rs485_status == 629);

    wheel_tuning::Map bad;
    CHECK(wheel_tuning::supported(2, 5, 2, 0, 0, 6, 12, 0, 0x5de70d7bu, m));
    CHECK(!wheel_tuning::supported(2, 5, 2, 0, 0, 6, 11, 1, 0xb3c4cf45u, bad));
    CHECK(!wheel_tuning::supported(2, 5, 2, 0, 0, 6, 13, 0, 0xb3c4cf45u, bad));
    CHECK(!wheel_tuning::supported(2, 5, 2, 0, 0, 6, 11, 0, 0x12345678u, bad));
    CHECK(!wheel_tuning::supported(2, 4, 2, 0, 0, 6, 11, 0, 0xb3c4cf45u, bad));

    uint16_t ep = 0;
    float lo = 0.0f, hi = 0.0f;
    CHECK(wheel_tuning::lookupParam("vel_gain", m, ep, lo, hi) && ep == 392);
    CHECK(wheel_tuning::validateValue(0.05f, lo, hi));
    CHECK(wheel_tuning::validateValue(2.0f, lo, hi));
    CHECK(!wheel_tuning::validateValue(0.049f, lo, hi));
    CHECK(!wheel_tuning::validateValue(2.01f, lo, hi));
    CHECK(!wheel_tuning::validateValue(NAN, lo, hi));
    CHECK(!wheel_tuning::validateValue(INFINITY, lo, hi));
    CHECK(!wheel_tuning::validateValue(-INFINITY, lo, hi));

    CHECK(wheel_tuning::lookupParam("dc_max_neg_current", m, ep, lo, hi) && ep == 169);
    CHECK(wheel_tuning::validateValue(-15.0f, lo, hi));
    CHECK(wheel_tuning::validateValue(-0.01f, lo, hi));
    CHECK(!wheel_tuning::validateValue(0.0f, lo, hi));
    CHECK(!wheel_tuning::validateValue(-15.1f, lo, hi));

    CHECK(!wheel_tuning::lookupParam(nullptr, m, ep, lo, hi));
    CHECK(!wheel_tuning::lookupParam("", m, ep, lo, hi));
    CHECK(!wheel_tuning::lookupParam("bogus_param", m, ep, lo, hi));

    wheel_tuning::ByteSdo bs;
    bs.waiting = true;
    bs.node = 0;
    bs.endpoint = 601;
    uint8_t msg[8] = {0, 0x59, 0x02, 0, 0x42, 0, 0, 0};  // 601 = 0x0259
    bs.observe(37, 5, msg); CHECK(!bs.received);         // wrong node
    bs.observe(5, 4, msg); CHECK(!bs.received);          // wrong len
    bs.observe(5, 8, msg); CHECK(!bs.received);          // wrong len
    msg[1] = 0x58; bs.observe(5, 5, msg); CHECK(!bs.received);  // wrong endpoint
    msg[1] = 0x59; bs.observe(5, 5, msg); CHECK(bs.received && bs.value == 0x42);
    bs.cancel(); bs.observe(5, 5, msg); CHECK(!bs.received);
}

static void testPrintAndApply() {
    resetEnv(11);
    CHECK(printWheelTuning(0));
    CHECK(applyWheelTuningPreset(0));
    CHECK(wheel_limits_verified[0]);
    CHECK(wheel_limits_verified_gen[0] == can_recovery.generation);
    CHECK(wheel_tuning::closeEnough(e.floats[305], 35.0f));
    CHECK(wheel_tuning::closeEnough(e.floats[306], 50.0f));
    CHECK(wheel_tuning::closeEnough(e.floats[371], wheel_tuning::VEL_GAIN));
    CHECK(wheel_tuning::closeEnough(e.floats[372], wheel_tuning::VEL_INTEGRATOR_GAIN));
    CHECK(wheel_tuning::closeEnough(e.floats[373], wheel_tuning::VEL_INTEGRATOR_LIMIT));
    CHECK(wheel_tuning::closeEnough(e.floats[375], wheel_tuning::VEL_LIMIT_TOLERANCE));
    CHECK(wheel_tuning::closeEnough(e.floats[376], wheel_tuning::VEL_RAMP_RATE));
    CHECK(wheel_tuning::closeEnough(e.floats[390], wheel_tuning::SPINOUT_MECH_THRESH));
    CHECK(wheel_tuning::closeEnough(e.floats[391], wheel_tuning::SPINOUT_ELEC_THRESH));
    // Saving right after applyWheelTuningPreset should now pass the 35/50 A gate.
    CHECK(saveWheelCurrentLimits(0) && e.save_calls == 1);

    resetEnv(12);
    CHECK(printWheelTuning(0));
    CHECK(applyWheelTuningPreset(0));
    CHECK(wheel_tuning::closeEnough(e.floats[325], 35.0f));
    CHECK(wheel_tuning::closeEnough(e.floats[326], 50.0f));
    CHECK(wheel_tuning::closeEnough(e.floats[392], wheel_tuning::VEL_GAIN));
    CHECK(wheel_tuning::closeEnough(e.floats[413], wheel_tuning::SPINOUT_ELEC_THRESH));

    // Refuses when faulted or not IDLE or mismatch.
    resetEnv(11);
    e.disarm_reason = 0x8000;
    CHECK(!applyWheelTuningPreset(0));
    CHECK(!wheel_limits_verified[0]);

    resetEnv(11);
    odrive_data[0].last_heartbeat.Axis_State = ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
    CHECK(!applyWheelTuningPreset(0));

    resetEnv(11);
    e.mismatch_write_ep = 371;
    CHECK(!applyWheelTuningPreset(0));

    resetEnv(11);
    e.commit = 0xdeadbeefu;
    CHECK(!printWheelTuning(0));
    CHECK(!applyWheelTuningPreset(0));
}

static void testSetParam() {
    resetEnv(11);
    CHECK(setWheelTuningParam(0, "vel_gain", 0.65f));
    CHECK(wheel_tuning::closeEnough(e.floats[371], 0.65f));
    CHECK(setWheelTuningParam(0, "dc_undervoltage", 36.0f));
    CHECK(wheel_tuning::closeEnough(e.floats[161], 36.0f));
    CHECK(setWheelTuningParam(0, "dc_max_neg_current", -3.0f));
    CHECK(wheel_tuning::closeEnough(e.floats[164], -3.0f));

    CHECK(!setWheelTuningParam(0, "unknown_key", 1.0f));
    CHECK(!setWheelTuningParam(0, "vel_gain", 9.9f));
    CHECK(!setWheelTuningParam(0, "vel_gain", NAN));

    e.mismatch_write_ep = 371;
    CHECK(!setWheelTuningParam(0, "vel_gain", 0.5f));

    resetEnv(11);
    e.active_errors = 1;
    CHECK(!setWheelTuningParam(0, "vel_gain", 0.5f));
}

static void testCalibration() {
    // Happy path
    resetEnv(11);
    wheel_limits_verified[0] = true;
    CHECK(calibrateWheelEncoderOffset(0));
    CHECK(!wheel_limits_verified[0]);
    CHECK(e.set_state_cal_calls == 1);
    CHECK(e.save_calls == 0);

    // Fast completion >1000 ms without observing state 7
    resetEnv(12);
    e.cal_enter_state7 = false;
    e.cal_duration_ms = 1100;
    CHECK(calibrateWheelEncoderOffset(0));

    // Refuses when motion not allowed / faulted / rs485 != 0
    resetEnv(11);
    e.motion_allowed = false;
    CHECK(!calibrateWheelEncoderOffset(0));
    CHECK(e.set_state_cal_calls == 0);

    resetEnv(11);
    e.disarm_reason = 0x8000;
    CHECK(!calibrateWheelEncoderOffset(0));
    CHECK(e.set_state_cal_calls == 0);

    resetEnv(11);
    e.bytes[601] = 2;
    CHECK(!calibrateWheelEncoderOffset(0));
    CHECK(e.set_state_cal_calls == 0);

    // Abort on USB input during calibration
    resetEnv(11);
    e.cal_duration_ms = 500;
    e.cal_inject_serial_cancel = true;
    CHECK(!calibrateWheelEncoderOffset(0));
    CHECK(e.set_state_idle_calls >= 1);
    CHECK(e.set_vel_zero_calls >= 2);

    // Error heartbeat during calibration
    resetEnv(11);
    e.cal_duration_ms = 500;
    e.cal_inject_error = true;
    CHECK(!calibrateWheelEncoderOffset(0));
    CHECK(e.set_state_idle_calls >= 1);

    // Timeout
    resetEnv(11);
    e.cal_never_finishes = true;
    CHECK(!calibrateWheelEncoderOffset(0));
    CHECK(e.set_state_idle_calls >= 1);
    CHECK(tick >= 20000);

    // Post-check failures (procedure_result != 0 or offset_valid != 1)
    resetEnv(11);
    e.cal_proc_result = 3;
    CHECK(!calibrateWheelEncoderOffset(0));

    resetEnv(11);
    e.cal_post_offset_valid = 0;
    CHECK(!calibrateWheelEncoderOffset(0));
}

int main() {
    testPolicy();
    testPrintAndApply();
    testSetParam();
    testCalibration();
    printf("wheel tuning: %u checks passed\n", checks);
    return 0;
}
