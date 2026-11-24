#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "RoboClaw.h"
#include "ODriveCAN.h"

#define IS_ESP32_TWAI
#include "driver/twai.h"
#include "odrive_can_transport.h"
#include "can_link_recovery.h"
#include "wheel_current_limits.h"
#include "wheel_persistence_policy.h"
#include "wheel_tuning_policy.h"
#include "odrive_live_telemetry.h"
#include "actuator_live_telemetry.h"
#include "actuator_current_limit.h"
#include "actuator_voltage_limit.h"
#include "wheel_fault_status.h"

// ---------- Hardware configuration ----------
#define CAN_BAUDRATE 1000000

#ifndef UNIT_ID
#define UNIT_ID 0
#endif

#ifndef LOCAL_ODRIVE_NODE_ID
#define LOCAL_ODRIVE_NODE_ID UNIT_ID
#endif

#define NUM_LOCAL_ODRIVES 1

#if UNIT_ID == 1
#define PIN_CAN_TX      D9
#define PIN_CAN_RX      D10
#else
#define PIN_CAN_TX      D8
#define PIN_CAN_RX      D9
#endif

#define PIN_ACT_LIMIT_IN  D5   // retracted / reverse limit
#define PIN_ACT_LIMIT_OUT D3   // extended / forward limit
#define PIN_ROBO_RX       D7
#define PIN_ROBO_TX       D6
#define ROBOCLAW_ADDR     0x80
#define ROBO_BAUDRATE     38400

#define PIN_SDA           A4
#define PIN_SCL           A5
#define TCAADDR           0x70
#define NUM_TOF_SENSORS   1
#define TOF_MUX_CHANNEL   2

// ---------- Bring-up behavior ----------
static constexpr float ACTUATOR_M_PER_REV = 0.008f;
static constexpr float ACTUATOR_PPR = 103.8f;
static constexpr float ACTUATOR_STARTUP_SPEED_MPS = 0.10f;  // 10 cm/s
static constexpr bool ACTUATOR_UNIT1_EXPERIMENTAL_HOMING = (UNIT_ID == 1);
static constexpr uint32_t ACTUATOR_LIMIT_TIMEOUT_MS = 8000;
static constexpr uint32_t ACTUATOR_LIMIT_DEBOUNCE_MS = ACTUATOR_UNIT1_EXPERIMENTAL_HOMING ? 80 : 30;
static constexpr uint32_t ACTUATOR_LIMIT_CONFIRM_MS = 150;
static constexpr uint8_t ACTUATOR_HOMING_ATTEMPTS = 3;
#ifndef ACTUATOR_MIN_ENCODER_RANGE_TICKS_CONFIG
#define ACTUATOR_MIN_ENCODER_RANGE_TICKS_CONFIG 1350
#endif
static constexpr int32_t ACTUATOR_MIN_ENCODER_RANGE_TICKS = ACTUATOR_MIN_ENCODER_RANGE_TICKS_CONFIG;
static constexpr int32_t ACTUATOR_MAX_ENCODER_RANGE_TICKS = 1900;
static constexpr int32_t ACTUATOR_CENTER_TOLERANCE_TICKS = 40;
static constexpr uint8_t ACTUATOR_STARTUP_POWER = 40;  // RoboClaw open-loop command, 0..127
static constexpr uint8_t ACTUATOR_LIMIT_CREEP_POWER = 24;
static constexpr uint8_t ACTUATOR_JOG_POWER = 20;
static constexpr uint32_t ACTUATOR_PULSE_MS = 250;
static constexpr uint8_t ACTUATOR_COMMAND_ATTEMPTS = 3;
static constexpr uint32_t ACTUATOR_COMMAND_RETRY_DELAY_MS = 60;
static constexpr uint32_t ACTUATOR_COMMAND_REFRESH_MS = 60;
static constexpr uint32_t ACTUATOR_RAMP_INTERVAL_MS = 60;
static constexpr uint8_t ACTUATOR_RAMP_STEP = 8;
static constexpr uint32_t ACTUATOR_PREFLIGHT_DELAY_MS = 150;
static constexpr uint8_t ACTUATOR_ENCODER_READ_ATTEMPTS = 3;
static constexpr uint32_t ACTUATOR_ENCODER_READ_RETRY_DELAY_MS = 5;
static constexpr uint8_t ACTUATOR_ENCODER_CENTER_POWER = 40;
static constexpr uint8_t ACTUATOR_ENCODER_CENTER_MED_POWER = 26;
static constexpr uint8_t ACTUATOR_ENCODER_CENTER_SLOW_POWER = 22;
static constexpr int32_t ACTUATOR_ENCODER_CENTER_MED_TICKS = 500;
static constexpr int32_t ACTUATOR_ENCODER_CENTER_SLOW_TICKS = 180;
static constexpr uint32_t ACTUATOR_CENTER_PROGRESS_MS = 250;
static constexpr int32_t ACTUATOR_CENTER_STALL_TICKS = 10;
static constexpr uint32_t ACTUATOR_CENTER_STALL_MS = 700;
static constexpr uint8_t ACTUATOR_CENTER_BREAKAWAY_POWER = 40;
static constexpr uint8_t ACTUATOR_CENTER_FALLBACK_POWER = 15;
static constexpr float ACTUATOR_CENTER_FALLBACK_FRACTION = 0.20f;
static constexpr uint32_t ACTUATOR_CENTER_FALLBACK_MIN_MS = 150;
static constexpr uint32_t ACTUATOR_CENTER_FALLBACK_MAX_MS = 350;
static constexpr uint32_t ACTUATOR_REV_RELEASE_TIMEOUT_MS = 800;
static constexpr uint32_t ROBOCLAW_SERIAL_RESET_DELAY_MS = 150;
static constexpr uint8_t ROBOCLAW_SERIAL_TIMEOUT_TENTHS = 2;  // 200 ms deadman timeout

static constexpr float ODRIVE_CURRENT_LIMIT_AMPS = 35.0f;
static constexpr float ODRIVE_DEFAULT_VEL_LIMIT_TURNS_PER_SEC = 10.0f;
static constexpr float ODRIVE_TEST_RPM = 5.0f;
static constexpr uint32_t ODRIVE_TEST_MS = 2000;
static constexpr uint32_t ODRIVE_MONITOR_MS = 5000;
static constexpr uint32_t ODRIVE_MONITOR_INTERVAL_MS = 500;
static constexpr float ODRIVE_TURN_TEST_RPM = 10.0f;
static constexpr float STARTUP_WHEEL_TEST_RPM = 600.0f;
static constexpr float STARTUP_WHEEL_TEST_SECONDS = 3.0f;
static constexpr uint32_t STARTUP_WHEEL_STOP_PAUSE_MS = 500;
static constexpr uint32_t ODRIVE_STOP_TIMEOUT_MS = 3000;
static constexpr uint32_t ODRIVE_STOP_SETTLE_MS = 200;
static constexpr float ODRIVE_STOP_SPEED_TURNS_PER_SEC = 0.05f;
static constexpr char WHEEL_STOP_REVISION[] = "2026-09-14-wheel-stop-fix1";

ESP32TWAIIntf can_intf;
wheel_current::Sdo wheel_current_sdo;
wheel_persistence::BoolSdo wheel_persistence_bool_sdo;
wheel_tuning::ByteSdo wheel_tuning_byte_sdo;
bool wheel_limits_verified[NUM_LOCAL_ODRIVES] = {false};
uint32_t wheel_limits_verified_gen[NUM_LOCAL_ODRIVES] = {0};
ODriveCAN local_odrive(wrap_can_intf(can_intf), LOCAL_ODRIVE_NODE_ID);
ODriveCAN* odrives[NUM_LOCAL_ODRIVES] = {&local_odrive};

struct ODriveUserData {
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    uint32_t last_heartbeat_ms = 0;
    uint32_t heartbeat_sequence = 0;

    Get_Encoder_Estimates_msg_t last_feedback;
    bool received_feedback = false;
};

ODriveUserData odrive_data[NUM_LOCAL_ODRIVES];

HardwareSerial RoboSerial(1);
RoboClaw roboclaw(&RoboSerial, 10000);
uint8_t roboclaw_addr = ROBOCLAW_ADDR;
uint32_t roboclaw_baudrate = ROBO_BAUDRATE;
int roboclaw_rx_pin = PIN_ROBO_RX;
int roboclaw_tx_pin = PIN_ROBO_TX;
uint8_t actuator_motor_channel = 1;
bool roboclaw_safety_timeout_ok = false;
Adafruit_VL53L0X tof_sensors[NUM_TOF_SENSORS];
bool tof_sensor_ok[NUM_TOF_SENSORS] = {false};
bool tof_enabled = false;
bool tof_initialized = false;
uint32_t last_tof_print_ms = 0;
bool tof_missing_reported = false;
bool can_initialized = false;
bool can_watch_enabled = false;
uint32_t can_watch_count = 0;
static constexpr uint32_t CAN_WATCH_MAX_FRAMES = 100;
static constexpr uint32_t CAN_AUTOBAUD_BEACON_MS = 1500;
static constexpr uint32_t CAN_AUTOBAUD_BEACON_INTERVAL_MS = 50;

// The dashboard is passive on USB. Poll read-only CAN telemetry only while
// the recovered CAN link is usable and has no queued transmission.
odrive_live::Telemetry live_telemetry;
actuator_live::Telemetry actuator_telemetry;
actuator_current::Guard actuator_current_guard;
actuator_voltage::Guard actuator_voltage_guard;
WheelFaultStatus wheel_fault;
struct LiveTelemetryIO {
    bool inputPending() const { return Serial.available() > 0; }
    odrive_live::BusStatus status() const {
        using namespace odrive_live;
        if (!can_initialized) return {UNINITIALIZED, false, false};
        twai_status_info_t s = {};
        if (twai_get_status_info(&s) != ESP_OK) return {UNKNOWN, false, false};
        Link link = UNKNOWN;
        switch (s.state) {
            case TWAI_STATE_RUNNING: link = RUNNING; break;
            case TWAI_STATE_STOPPED: link = STOPPED; break;
            case TWAI_STATE_BUS_OFF: link = BUS_OFF; break;
            case TWAI_STATE_RECOVERING: link = RECOVERING; break;
        }
        return {link, s.msgs_to_tx == 0, s.tx_error_counter < 128 && s.rx_error_counter < 128};
    }
    void sendRead(uint8_t node, uint8_t command) const {
        if (command != 0x17 && command != 0x14) return;
        sendMsg(can_intf, (uint32_t(node) << 5) | command, 8, nullptr);
    }
    bool writeIfRoom(const char* line, int length) const {
        if (length <= 0 || length > 64 || Serial.availableForWrite() < length) return false;
        return Serial.write(reinterpret_cast<const uint8_t*>(line), length) == size_t(length);
    }
} live_telemetry_io;

String input_line;

void actuatorStop();

bool waitWithStopCheck(uint32_t duration_ms, bool require_wheel = false);
bool stopRequestedFromSerial();
bool stopLocalWheels();

float rpmToTurnsPerSecond(float rpm) {
    return rpm / 60.0f;
}

int32_t metersPerSecondToTicksPerSecond(float speed_mps) {
    return (int32_t)lroundf((ACTUATOR_PPR / ACTUATOR_M_PER_REV) * speed_mps);
}

bool setupCan() {
    const gpio_num_t can_tx_gpio = (gpio_num_t)digitalPinToGPIONumber(PIN_CAN_TX);
    const gpio_num_t can_rx_gpio = (gpio_num_t)digitalPinToGPIONumber(PIN_CAN_RX);
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        can_tx_gpio,
        can_rx_gpio,
        TWAI_MODE_NORMAL
    );

    g_config.rx_queue_len = 32; // Buffer short foreground diagnostics at 100 Hz.
    twai_timing_config_t t_config;
    switch (CAN_BAUDRATE) {
        case 1000000: t_config = TWAI_TIMING_CONFIG_1MBITS(); break;
        case 800000:  t_config = TWAI_TIMING_CONFIG_800KBITS(); break;
        case 500000:  t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
        case 250000:  t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 125000:  t_config = TWAI_TIMING_CONFIG_125KBITS(); break;
        case 100000:  t_config = TWAI_TIMING_CONFIG_100KBITS(); break;
        case 50000:   t_config = TWAI_TIMING_CONFIG_50KBITS(); break;
        case 25000:   t_config = TWAI_TIMING_CONFIG_25KBITS(); break;
        default:      t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    return true;
}

void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
    ODriveUserData* data = static_cast<ODriveUserData*>(user_data);
    data->last_heartbeat = msg;
    data->received_heartbeat = true;
    data->last_heartbeat_ms = millis();
    ++data->heartbeat_sequence;
}

void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    ODriveUserData* data = static_cast<ODriveUserData*>(user_data);
    data->last_feedback = msg;
    data->received_feedback = true;
}

void onCanMessage(const CanMsg& msg) {
    wheel_current_sdo.observe(msg.id, msg.len, msg.buffer);
    wheel_persistence_bool_sdo.observe(msg.id, msg.len, msg.buffer);
    wheel_tuning_byte_sdo.observe(msg.id, msg.len, msg.buffer);
    wheel_fault.observe(msg.id, msg.len, msg.buffer, millis(), LOCAL_ODRIVE_NODE_ID, live_telemetry);
    live_telemetry.receive(msg.id, msg.len, msg.buffer, millis(), LOCAL_ODRIVE_NODE_ID);
    if (can_watch_enabled && can_watch_count < CAN_WATCH_MAX_FRAMES) {
        Serial.printf("CAN rx id=0x%03lx len=%u data=", (unsigned long)msg.id, msg.len);
        for (uint8_t i = 0; i < msg.len && i < 8; ++i) {
            if (msg.buffer[i] < 16) {
                Serial.print("0");
            }
            Serial.print(msg.buffer[i], HEX);
            if (i + 1 < msg.len && i < 7) {
                Serial.print(" ");
            }
        }
        Serial.println();
        can_watch_count++;
    }

    for (auto odrive: odrives) {
        onReceive(msg, *odrive);
    }
}

#include "can_link_runtime.h"
#include "actuator_live_runtime.h"

void pumpCanOnce() {
    if (can_initialized) {
        pumpEvents(can_intf);
    }
    serviceCanRecovery();
    wheel_fault.emit(live_telemetry_io);
    live_telemetry.tick(millis(), UNIT_ID, LOCAL_ODRIVE_NODE_ID, live_telemetry_io);
    actuator_telemetry.tick(millis(), UNIT_ID, actuator_motor_channel, actuator_live_io);
}

void pumpCanFor(uint32_t duration_ms) {
    const uint32_t start_ms = millis();
    while (millis() - start_ms < duration_ms) {
        pumpCanOnce();
        delay(2);
    }
}

const char* twaiStateName(twai_state_t state) {
    switch (state) {
        case TWAI_STATE_STOPPED: return "STOPPED";
        case TWAI_STATE_RUNNING: return "RUNNING";
        case TWAI_STATE_BUS_OFF: return "BUS_OFF";
        case TWAI_STATE_RECOVERING: return "RECOVERING";
        default: return "UNKNOWN";
    }
}

void printCanStatus() {
    printCanRecoveryStatus();
    if (!can_initialized) {
        Serial.println("CAN driver offline. See recovery status above.");
        return;
    }

    twai_status_info_t status = {};
    esp_err_t result = twai_get_status_info(&status);
    if (result != ESP_OK) {
        Serial.printf("CAN status unavailable: esp_err=%d\n", (int)result);
        return;
    }

    Serial.println("\n--- CAN controller status ---");
    Serial.printf("state=%s tx_queue=%lu rx_queue=%lu\n",
        twaiStateName(status.state),
        (unsigned long)status.msgs_to_tx,
        (unsigned long)status.msgs_to_rx);
    Serial.printf("tx_error_counter=%lu rx_error_counter=%lu\n",
        (unsigned long)status.tx_error_counter,
        (unsigned long)status.rx_error_counter);
    Serial.printf("tx_failed_count=%lu bus_error_count=%lu arb_lost_count=%lu\n",
        (unsigned long)status.tx_failed_count,
        (unsigned long)status.bus_error_count,
        (unsigned long)status.arb_lost_count);
    Serial.printf("rx_missed_count=%lu rx_overrun_count=%lu\n",
        (unsigned long)status.rx_missed_count,
        (unsigned long)status.rx_overrun_count);
    Serial.println("-----------------------------\n");
}

void resetCanController() {
    can_recovery.disable();
    const bool stopped = quiesceCanLink();
    Serial.println(stopped ? "CAN controller reset. Auto-retry paused; run can init."
                           : "CAN reset incomplete; driver teardown failed. Wheel blocked.");
}

void sendCanAutobaudBeacon(uint32_t duration_ms = CAN_AUTOBAUD_BEACON_MS) {
    const uint8_t payload[1] = {0};
    const uint32_t start_ms = millis();
    uint32_t accepted_count = 0;
    uint32_t failed_count = 0;

    Serial.printf("Sending CAN auto-baud beacon for %.1f seconds...\n", (float)duration_ms / 1000.0f);
    while (millis() - start_ms < duration_ms) {
        if (stopRequestedFromSerial()) { stopLocalWheels(); break; }
        if (!can_recovery.ready()) break;
        if (sendMsg(can_intf, 0x7df, sizeof(payload), payload)) {
            accepted_count++;
        } else {
            failed_count++;
        }
        pumpCanOnce();
        delay(CAN_AUTOBAUD_BEACON_INTERVAL_MS);
    }
    Serial.printf("CAN beacon transmit requests accepted=%lu failed=%lu\n",
        (unsigned long)accepted_count,
        (unsigned long)failed_count);
    printCanStatus();
}

bool ensureCanInitialized() {
    if (can_recovery.phase == can_link::INACTIVE) can_recovery.request(millis());
    pumpCanOnce();
    if (can_recovery.ready()) return true;
    Serial.println("CAN reconnecting; command rejected. See can status.");
    return false;
}

void watchCanFrames(uint32_t duration_ms = 5000) {
    // Permit raw observation while reconnecting, including a wrong node ID.
    if (can_recovery.phase == can_link::INACTIVE) can_recovery.request(millis());
    pumpCanOnce();

    Serial.printf("Watching raw CAN frames for %.1f seconds. Type stop to exit early.\n", (float)duration_ms / 1000.0f);
    Serial.println("ODrive heartbeats should appear about every 0.1 seconds if the CAN bus is wired/configured.");

    can_watch_count = 0;
    can_watch_enabled = true;
    const uint32_t start_ms = millis();
    while (millis() - start_ms < duration_ms) {
        pumpCanOnce();
        if (stopRequestedFromSerial()) {
            Serial.println("CAN watch stopped.");
            break;
        }
        delay(2);
    }
    can_watch_enabled = false;

    if (can_watch_count == 0) {
        Serial.println("No CAN frames received.");
    } else if (can_watch_count >= CAN_WATCH_MAX_FRAMES) {
        Serial.println("CAN watch frame limit reached.");
    }
    Serial.printf("CAN watch complete. Frames printed: %lu\n", (unsigned long)can_watch_count);
}

bool odriveOnline(int index) {
    const auto s = readCanLinkStatus();
    return can_recovery.ready() && s.busUsable() &&
           odrive_data[index].received_heartbeat &&
           (millis() - odrive_data[index].last_heartbeat_ms < 1000);
}

const char* odriveStateName(uint8_t state) {
    switch (state) {
        case AXIS_STATE_IDLE: return "IDLE";
        case AXIS_STATE_FULL_CALIBRATION_SEQUENCE: return "FULL_CALIBRATION";
        case AXIS_STATE_MOTOR_CALIBRATION: return "MOTOR_CALIBRATION";
        case AXIS_STATE_ENCODER_OFFSET_CALIBRATION: return "ENCODER_OFFSET_CALIBRATION";
        case AXIS_STATE_CLOSED_LOOP_CONTROL: return "CLOSED_LOOP_CONTROL";
        case AXIS_STATE_HOMING: return "HOMING";
        default: return "OTHER";
    }
}

void printOdriveErrorBits(uint32_t errors) {
    if (errors == 0) {
        Serial.print("none");
        return;
    }

    bool first = true;
    auto print_bit = [&](uint32_t bit, const char* name) {
        if ((errors & bit) == 0) {
            return;
        }
        if (!first) {
            Serial.print(",");
        }
        Serial.print(name);
        first = false;
    };

    print_bit(ODRIVE_ERROR_INITIALIZING, "INITIALIZING");
    print_bit(ODRIVE_ERROR_SYSTEM_LEVEL, "SYSTEM_LEVEL");
    print_bit(ODRIVE_ERROR_TIMING_ERROR, "TIMING_ERROR");
    print_bit(ODRIVE_ERROR_MISSING_ESTIMATE, "MISSING_ESTIMATE");
    print_bit(ODRIVE_ERROR_BAD_CONFIG, "BAD_CONFIG");
    print_bit(ODRIVE_ERROR_DRV_FAULT, "DRV_FAULT");
    print_bit(ODRIVE_ERROR_MISSING_INPUT, "MISSING_INPUT");
    print_bit(ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE, "DC_BUS_OVER_VOLTAGE");
    print_bit(ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE, "DC_BUS_UNDER_VOLTAGE");
    print_bit(ODRIVE_ERROR_DC_BUS_OVER_CURRENT, "DC_BUS_OVER_CURRENT");
    print_bit(ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT, "DC_BUS_OVER_REGEN_CURRENT");
    print_bit(ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION, "CURRENT_LIMIT_VIOLATION");
    print_bit(ODRIVE_ERROR_MOTOR_OVER_TEMP, "MOTOR_OVER_TEMP");
    print_bit(ODRIVE_ERROR_INVERTER_OVER_TEMP, "INVERTER_OVER_TEMP");
    print_bit(ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION, "VELOCITY_LIMIT_VIOLATION");
    print_bit(ODRIVE_ERROR_POSITION_LIMIT_VIOLATION, "POSITION_LIMIT_VIOLATION");
    print_bit(ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED, "WATCHDOG_TIMER_EXPIRED");
    print_bit(ODRIVE_ERROR_ESTOP_REQUESTED, "ESTOP_REQUESTED");
    print_bit(ODRIVE_ERROR_SPINOUT_DETECTED, "SPINOUT_DETECTED");
    print_bit(ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED, "BRAKE_RESISTOR_DISARMED");
    print_bit(ODRIVE_ERROR_THERMISTOR_DISCONNECTED, "THERMISTOR_DISCONNECTED");
    print_bit(ODRIVE_ERROR_CALIBRATION_ERROR, "CALIBRATION_ERROR");
}

void printOdriveDiagnostics(int index = 0) {
    if (index < 0 || index >= NUM_LOCAL_ODRIVES) {
        Serial.println("Invalid wheel index for this unit. Use wheel or w0.");
        return;
    }
    if (!ensureCanInitialized()) {
        return;
    }

    pumpCanFor(500);
    Serial.println("\n--- ODrive diagnostics ---");
    Serial.printf("unit id: %d local ODrive node id: %d\n", UNIT_ID, LOCAL_ODRIVE_NODE_ID);
    Serial.printf("heartbeat: %s\n", odriveOnline(index) ? "online" : "not seen recently");
    if (odrive_data[index].received_heartbeat) {
        Serial.printf(
            "state: %u %s, axis_error=0x%08lx, procedure_result=%u, trajectory_done=%u\n",
            odrive_data[index].last_heartbeat.Axis_State,
            odriveStateName(odrive_data[index].last_heartbeat.Axis_State),
            (unsigned long)odrive_data[index].last_heartbeat.Axis_Error,
            odrive_data[index].last_heartbeat.Procedure_Result,
            odrive_data[index].last_heartbeat.Trajectory_Done_Flag
        );
    }

    Get_Version_msg_t version;
    if (odrives[index]->getVersion(version, 100)) {
        Serial.printf(
            "version: protocol %u hw %u.%u.%u fw %u.%u.%u%s\n",
            version.Protocol_Version,
            version.Hw_Version_Major,
            version.Hw_Version_Minor,
            version.Hw_Version_Variant,
            version.Fw_Version_Major,
            version.Fw_Version_Minor,
            version.Fw_Version_Revision,
            version.Fw_Version_Unreleased ? " unreleased" : ""
        );
    } else {
        Serial.println("version: no response");
    }

    Get_Error_msg_t errors;
    if (odrives[index]->getError(errors, 100)) {
        Serial.printf("active_errors: 0x%08lx ", (unsigned long)errors.Active_Errors);
        printOdriveErrorBits(errors.Active_Errors);
        Serial.println();
        Serial.printf("disarm_reason: 0x%08lx ", (unsigned long)errors.Disarm_Reason);
        printOdriveErrorBits(errors.Disarm_Reason);
        Serial.println();
    } else {
        Serial.println("errors: no response");
    }

    Get_Bus_Voltage_Current_msg_t bus;
    if (odrives[index]->getBusVI(bus, 100)) {
        Serial.printf("bus: %.2f V %.3f A\n", bus.Bus_Voltage, bus.Bus_Current);
    } else {
        Serial.println("bus: no response");
    }

    Get_Encoder_Estimates_msg_t feedback;
    if (odrives[index]->getFeedback(feedback, 100)) {
        Serial.printf("encoder: pos=%.4f turns vel=%.4f turns/s %.2f RPM\n",
            feedback.Pos_Estimate,
            feedback.Vel_Estimate,
            feedback.Vel_Estimate * 60.0f);
    } else if (odrive_data[index].received_feedback) {
        Serial.printf("encoder cached: pos=%.4f turns vel=%.4f turns/s %.2f RPM\n",
            odrive_data[index].last_feedback.Pos_Estimate,
            odrive_data[index].last_feedback.Vel_Estimate,
            odrive_data[index].last_feedback.Vel_Estimate * 60.0f);
    } else {
        Serial.println("encoder: no response");
    }

    Get_Iq_msg_t iq;
    if (odrives[index]->getCurrents(iq, 100)) {
        if (odriveOnline(index) && odrive_data[index].last_heartbeat.Axis_State == 8 &&
            odrive_data[index].last_heartbeat.Axis_Error == 0)
            Serial.printf("phase current: setpoint=%.3f A measured=%.3f A\n", iq.Iq_Setpoint, iq.Iq_Measured);
        else
            Serial.printf("phase current retained (not live outside healthy CLOSED_LOOP): target=%.3f A measured=%.3f A\n", iq.Iq_Setpoint, iq.Iq_Measured);
    } else {
        Serial.println("phase current: no response");
    }

    Get_Temperature_msg_t temp;
    if (odrives[index]->getTemperature(temp, 100)) {
        Serial.printf("temperature: fet=%.1f C motor=%.1f C\n", temp.FET_Temperature, temp.Motor_Temperature);
    } else {
        Serial.println("temperature: no response");
    }

    Get_Powers_msg_t power;
    if (odrives[index]->getPower(power, 100)) {
        Serial.printf("power: electrical=%.2f W mechanical=%.2f W\n", power.Electrical_Power, power.Mechanical_Power);
    } else {
        Serial.println("power: no response");
    }
    if (wheel_fault.valid) {
        Serial.printf("Last wheel fault (historical): %lu ms ago, state=%u, error=0x%08lx\n",
            (unsigned long)(millis()-wheel_fault.at), wheel_fault.state, (unsigned long)wheel_fault.error);
        Serial.printf("Samples before that fault: bus=%.2f V age=%ld ms; Iq=%.3f A age=%ld ms; RPM=%.2f age=%ld ms\n",
            wheel_fault.vi.seen?wheel_fault.vi.a:NAN, (long)wheel_fault.vi.age(wheel_fault.at),
            wheel_fault.iq.seen?wheel_fault.iq.b:NAN, (long)wheel_fault.iq.age(wheel_fault.at),
            wheel_fault.encoder.seen?wheel_fault.encoder.b*60.0f:NAN, (long)wheel_fault.encoder.age(wheel_fault.at));
    }
    Serial.println("--------------------------\n");
}

bool configureOdriveVelocityMode(int index, float required_turns_per_sec) {
    if (!wheelMotionAllowed()) return false;
    float limit = max(ODRIVE_DEFAULT_VEL_LIMIT_TURNS_PER_SEC, fabsf(required_turns_per_sec) * 1.5f);
    return odrives[index]->setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP) &&
           odrives[index]->setLimits(limit, ODRIVE_CURRENT_LIMIT_AMPS);
}

bool waitForWheelStopped(int index) {
    const uint32_t start_ms = millis();
    uint32_t quiet_since = start_ms;
    bool quiet = false;
    while (millis() - start_ms < ODRIVE_STOP_TIMEOUT_MS) {
        pumpCanOnce();
        if (Serial.available() > 0 || !Serial || !odriveOnline(index) ||
            odrive_data[index].last_heartbeat.Axis_Error != 0) {
            odrives[index]->setVelocity(0.0f, 0.0f);
            Serial.printf("w%d: stop verification aborted; no restart.\n", index);
            return false;
        }
        // Keep the existing limits while braking. Never reuse cached feedback.
        if (!odrives[index]->setVelocity(0.0f, 0.0f)) {
            Serial.printf("w%d: zero-speed transmission failed.\n", index);
            return false;
        }
        Get_Encoder_Estimates_msg_t feedback;
        const bool fresh = odrives[index]->getFeedback(feedback, 50);
        if (Serial.available() > 0 || !Serial || !odriveOnline(index) ||
            odrive_data[index].last_heartbeat.Axis_Error != 0) {
            return false;
        }
        if (fresh && isfinite(feedback.Vel_Estimate) &&
            fabsf(feedback.Vel_Estimate) <= ODRIVE_STOP_SPEED_TURNS_PER_SEC) {
            if (!quiet) {
                quiet = true;
                quiet_since = millis();
            }
            if (millis() - quiet_since >= ODRIVE_STOP_SETTLE_MS &&
                millis() - start_ms < ODRIVE_STOP_TIMEOUT_MS) {
                return true;
            }
        } else {
            quiet = false;
        }
        delay(10);
    }
    Serial.printf("w%d: stop not verified within %lu ms; no restart.\n",
                  index, (unsigned long)ODRIVE_STOP_TIMEOUT_MS);
    return false;
}

// Leave incoming commands queued so the main handler can stop BOTH motors.
bool wheelStartCancelled(int index) {
    if (Serial.available() > 0 || !Serial || !odriveOnline(index) || !wheelMotionAllowed()) {
        odrives[index]->setVelocity(0.0f, 0.0f);
        Serial.printf("w%d: start cancelled; no speed command sent.\n", index);
        return true;
    }
    return false;
}

bool wheelErrorsClearForStart(int index) {
    Get_Error_msg_t errors;
    const bool fresh = odrives[index]->getError(errors, 100);
    if (wheelStartCancelled(index)) return false;
    if (!fresh) {
        Serial.printf("w%d: no fresh error response; start refused.\n", index);
        return false;
    }
    if (errors.Active_Errors || errors.Disarm_Reason) {
        Serial.printf("w%d: start refused; active=0x%08lx disarm=0x%08lx\n", index,
            (unsigned long)errors.Active_Errors, (unsigned long)errors.Disarm_Reason);
        printOdriveErrorBits(errors.Active_Errors | errors.Disarm_Reason);
        Serial.println("; inspect w0 diag. Clear Errors is an explicit action.");
        return false;
    }
    return true;
}

#include "wheel_current_limits_runtime.h"
#include "wheel_persistence_runtime.h"
#include "wheel_tuning_runtime.h"

bool enterOdriveClosedLoop(int index, float required_turns_per_sec = 0.0f) {
    if (!wheelMotionAllowed()) {
        Serial.println("Wheel blocked: reconnect, then Stop and send a new move.");
        return false;
    }
    if (!odriveOnline(index)) {
        Serial.printf("w%d: no heartbeat\n", index);
        return false;
    }
    // Replace retained speed, but preserve controller faults for diagnosis.
    if (!odrives[index]->setVelocity(0.0f, 0.0f)) return false;
    if (wheelStartCancelled(index) || !wheelErrorsClearForStart(index)) return false;
    if (!waitForWheelStopped(index)) return false;
    if (!ensureWheelCurrentLimits(index)) return false;
    if (wheelStartCancelled(index) || !configureOdriveVelocityMode(index, required_turns_per_sec)) return false;
    if (wheelStartCancelled(index)) return false;

    const uint32_t before = odrive_data[index].heartbeat_sequence;
    if (!odrives[index]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)) return false;
    const uint32_t start_ms = millis();
    while (millis() - start_ms < 3000) {
        pumpCanOnce();
        if (wheelStartCancelled(index)) return false;
        if (odrive_data[index].heartbeat_sequence != before) {
            const auto& heartbeat = odrive_data[index].last_heartbeat;
            if (heartbeat.Axis_Error != 0) {
                odrives[index]->setVelocity(0.0f, 0.0f);
                Serial.printf("w%d: fault while arming: 0x%08lx; no retry.\n", index,
                    (unsigned long)heartbeat.Axis_Error);
                printOdriveErrorBits(heartbeat.Axis_Error); Serial.println();
                return false;
            }
            if (heartbeat.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) return true;
        }
        delay(2);
    }
    odrives[index]->setVelocity(0.0f, 0.0f);
    Serial.printf("w%d: no fresh closed-loop confirmation; start refused.\n", index);
    return false;
}

bool setWheelRpm(int index, float rpm) {
    if (index < 0 || index >= NUM_LOCAL_ODRIVES) {
        Serial.println("Invalid wheel index for this unit. Use wheel, w, or w0.");
        return false;
    }
    if (!ensureCanInitialized()) {
        return false;
    }
    if (!isfinite(rpm)) {
        odrives[index]->setVelocity(0.0f, 0.0f);
        return false;
    }
    // A stop must not re-arm the motor or change its mode/current/speed limits.
    if (rpm == 0.0f) {
        const bool queued = odrives[index]->setVelocity(0.0f, 0.0f);
        if (queued) can_recovery.acknowledgeStop();
        return queued;
    }
    float turns_per_sec = rpmToTurnsPerSecond(rpm);
    const bool already_closed_loop =
        wheelMotionAllowed() &&
        odriveOnline(index) &&
        wheel_limits_verified[index] &&
        wheel_limits_verified_gen[index] == can_recovery.generation &&
        odrive_data[index].last_heartbeat.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL &&
        odrive_data[index].last_heartbeat.Axis_Error == 0;
    if (!already_closed_loop) {
        if (!enterOdriveClosedLoop(index, turns_per_sec)) {
            return false;
        }
    } else {
        if (!configureOdriveVelocityMode(index, turns_per_sec)) {
            return false;
        }
    }
    // Stop may have arrived during the preceding CAN exchanges.
    if (wheelStartCancelled(index) || odrive_data[index].last_heartbeat.Axis_Error != 0 ||
        odrive_data[index].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrives[index]->setVelocity(0.0f, 0.0f);
        return false;
    }
    if (!odrives[index]->setVelocity(turns_per_sec, 0.0f)) {
        odrives[index]->setVelocity(0.0f, 0.0f);
        return false;
    }
    Serial.printf("w%d target queued: %.2f RPM. Check actual RPM.\n", index, rpm);
    return true;
}

bool setLocalWheelRpm(float rpm) {
    return setWheelRpm(0, rpm);
}

bool setAllLocalWheelsRpm(float rpm) {
    if (!ensureCanInitialized()) {
        return false;
    }
    for (int i = 0; i < NUM_LOCAL_ODRIVES; ++i) {
        if (!setWheelRpm(i, rpm)) {
            return false;
        }
    }
    return true;
}

bool stopLocalWheels() {
    if (!can_initialized) return false;
    bool queued = true;
    for (int i = 0; i < NUM_LOCAL_ODRIVES; ++i) {
        // Zero only: never arm, lower limits, or acknowledge the recovery latch.
        if (!odrives[i]->setVelocity(0.0f, 0.0f)) queued = false;
    }
    return queued;
}

void operatorWheelStop() {
    const bool queued = stopLocalWheels();
    if (queued && can_recovery.acknowledgeStop())
        Serial.println("Wheel zero queued. Physical stop not verified.");
    else
        Serial.println("Wheel stop NOT confirmed; CAN reconnecting/disabled.");
}

void clearLocalWheelErrors() {
    // Only an explicit operator action reaches this function. Never re-arm or
    // replay a speed while recovering; keep motion blocked until every check passes.
    can_recovery.needs_stop = true;
    wheel_limits_verified[0] = false;
    if (!ensureCanInitialized()) return;
    const uint32_t recovery_generation = can_recovery.generation;
    const uint32_t began = millis();
    static constexpr uint32_t ACTION_TIMEOUT_MS = 1500;

    auto fail = [](const char* reason) {
        can_recovery.needs_stop = true;
        stopLocalWheels();
        Serial.print("Wheel recovery NOT confirmed: ");
        Serial.println(reason);
    };
    auto stillValid = [&]() -> bool {
        if (Serial.available() > 0 || !Serial) {
            fail("new USB command/disconnect; queued commands preserved.");
            return false;
        }
        if (can_recovery.generation != recovery_generation || !odriveOnline(0)) {
            fail("CAN changed or heartbeat lost; retry explicitly after reconnect.");
            return false;
        }
        if (uint32_t(millis() - began) >= ACTION_TIMEOUT_MS) {
            fail("timed out waiting for a fresh controller response.");
            return false;
        }
        return true;
    };
    auto waitForIdle = [&](uint32_t after_sequence, bool require_no_error) -> bool {
        while (true) {
            pumpCanOnce();
            if (!stillValid()) return false;
            const auto& data = odrive_data[0];
            if (data.heartbeat_sequence != after_sequence &&
                data.last_heartbeat.Axis_State == ODriveAxisState::AXIS_STATE_IDLE) {
                if (require_no_error && data.last_heartbeat.Axis_Error != 0) {
                    Serial.printf("Wheel heartbeat still reports error=0x%08lx.\n",
                        (unsigned long)data.last_heartbeat.Axis_Error);
                    fail("fault remains after Clear Errors; inspect w0 diag.");
                    return false;
                }
                return true;
            }
            delay(2);
        }
    };

    if (!stillValid()) return;
    if (!stopLocalWheels()) { fail("zero-speed queue failed."); return; }
    const uint32_t before_idle = odrive_data[0].heartbeat_sequence;
    if (!odrives[0]->setState(ODriveAxisState::AXIS_STATE_IDLE)) {
        fail("IDLE request queue failed."); return;
    }
    if (!waitForIdle(before_idle, false) || !stillValid()) return;

    const uint32_t before_clear = odrive_data[0].heartbeat_sequence;
    if (!odrives[0]->clearErrors()) { fail("Clear Errors queue failed."); return; }
    if (!waitForIdle(before_clear, true) || !stillValid()) return;

    Get_Error_msg_t errors;
    const uint32_t elapsed = uint32_t(millis() - began);
    if (elapsed >= ACTION_TIMEOUT_MS) {
        fail("timed out before the error response check."); return;
    }
    const uint32_t remaining = ACTION_TIMEOUT_MS - elapsed;
    const uint16_t request_timeout = uint16_t(remaining < 100 ? remaining : 100);
    const bool fresh = odrives[0]->getError(errors, request_timeout);
    // The synchronous query can pump CAN and change the recovery generation.
    if (!stillValid()) return;
    if (!fresh) { fail("no fresh error response."); return; }
    Serial.printf("Fresh error response: active=0x%08lx disarm=0x%08lx\n",
        (unsigned long)errors.Active_Errors, (unsigned long)errors.Disarm_Reason);
    if (errors.Active_Errors || errors.Disarm_Reason ||
        odrive_data[0].last_heartbeat.Axis_Error != 0 ||
        odrive_data[0].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE) {
        fail("controller still faulted or not IDLE; inspect w0 diag."); return;
    }
    if (!can_recovery.acknowledgeStop()) {
        fail("CAN is no longer ready."); return;
    }
    Serial.println("Wheel recovered in IDLE. Send a new movement command when ready.");
}

void idleLocalWheels() {
    if (!ensureCanInitialized()) return;
    if (!stopLocalWheels()) { Serial.println("Wheel zero queue failed; IDLE not confirmed."); return; }
    const uint32_t before = odrive_data[0].heartbeat_sequence;
    if (!odrives[0]->setState(ODriveAxisState::AXIS_STATE_IDLE)) {
        Serial.println("IDLE request queue failed."); return;
    }
    pumpCanFor(200);
    const bool confirmed = odriveOnline(0) && odrive_data[0].heartbeat_sequence != before &&
        odrive_data[0].last_heartbeat.Axis_State == ODriveAxisState::AXIS_STATE_IDLE;
    Serial.println(confirmed ? "Local ODrive IDLE confirmed by fresh heartbeat." : "Local ODrive IDLE NOT confirmed.");
}

bool runWheelJogTest(float rpm = ODRIVE_TEST_RPM, uint32_t duration_ms = ODRIVE_TEST_MS) {
    Serial.printf("Wheel jog test: %.2f RPM for %.2f seconds.\n", rpm, (float)duration_ms / 1000.0f);
    if (!setLocalWheelRpm(rpm)) {
        stopLocalWheels();
        return false;
    }
    if (!waitWithStopCheck(duration_ms, true)) {
        return false;
    }
    stopLocalWheels();
    if (!waitForWheelStopped(0)) {
        return false;
    }
    Serial.println("Wheel jog test complete. Wheel stopped.");
    return true;
}

void monitorLocalWheel(uint32_t duration_ms = ODRIVE_MONITOR_MS) {
    if (!ensureCanInitialized()) {
        return;
    }

    Serial.printf("Monitoring local wheel for %.1f seconds. Any USB command exits monitoring and is handled normally.\n", (float)duration_ms / 1000.0f);
    Serial.println("t_s,state,actual_rpm,pos_turns,bus_v,bus_a,iq_setpoint_a,iq_measured_a");

    const uint32_t start_ms = millis();
    uint32_t next_print_ms = 0;
    while (millis() - start_ms < duration_ms) {
        pumpCanOnce();
        // Leave the full command (including Stop) for the main command handler.
        // That handler stops both motors and acknowledges CAN recovery correctly.
        if (Serial.available()) {
            Serial.println("Wheel monitor ended; queued command will be handled.");
            return;
        }

        const uint32_t elapsed_ms = millis() - start_ms;
        if (elapsed_ms >= next_print_ms) {
            Get_Encoder_Estimates_msg_t feedback;
            bool feedback_ok = odrives[0]->getFeedback(feedback, 50);

            Get_Bus_Voltage_Current_msg_t bus;
            bool bus_ok = odrives[0]->getBusVI(bus, 50);

            Get_Iq_msg_t iq;
            const bool closed_loop_before = odriveOnline(0) &&
                odrive_data[0].last_heartbeat.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;
            bool iq_ok = closed_loop_before && odrives[0]->getCurrents(iq, 50);
            // ODrive can retain the last current values after disarming.
            iq_ok = iq_ok && odriveOnline(0) &&
                odrive_data[0].last_heartbeat.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL;

            Serial.printf("%.2f,%s,",
                (float)elapsed_ms / 1000.0f,
                odriveOnline(0) ? odriveStateName(odrive_data[0].last_heartbeat.Axis_State) : "NO_HEARTBEAT");
            if (feedback_ok) {
                Serial.printf("%.2f,%.4f,", feedback.Vel_Estimate * 60.0f, feedback.Pos_Estimate);
            } else {
                Serial.print("nan,nan,");
            }
            if (bus_ok) {
                Serial.printf("%.2f,%.3f,", bus.Bus_Voltage, bus.Bus_Current);
            } else {
                Serial.print("nan,nan,");
            }
            if (iq_ok) {
                Serial.printf("%.3f,%.3f\n", iq.Iq_Setpoint, iq.Iq_Measured);
            } else {
                Serial.println("nan,nan");
            }

            next_print_ms += ODRIVE_MONITOR_INTERVAL_MS;
        }
        delay(10);
    }
    Serial.println("Wheel monitor complete.");
}

bool moveWheelRelativeTurns(float turns, float rpm = ODRIVE_TURN_TEST_RPM) {
    if (!ensureCanInitialized()) {
        return false;
    }
    if (!enterOdriveClosedLoop(0)) {
        return false;
    }

    Get_Encoder_Estimates_msg_t start_feedback;
    if (!odrives[0]->getFeedback(start_feedback, 100)) {
        Serial.println("Could not read ODrive encoder before turn test.");
        return false;
    }

    const float speed_turns_per_sec = max(0.02f, fabsf(rpmToTurnsPerSecond(rpm)));
    const float signed_velocity = turns >= 0.0f ? speed_turns_per_sec : -speed_turns_per_sec;
    const float target_delta = fabsf(turns);

    if (!configureOdriveVelocityMode(0, signed_velocity)) return false;
    pumpCanFor(100);

    Serial.printf(
        "Wheel velocity-turn test: start=%.4f delta=%.4f turns at %.2f RPM.\n",
        start_feedback.Pos_Estimate,
        turns,
        fabsf(rpm)
    );
    Serial.println("If the wheel is truly 1:1 with the encoder, the visible wheel should move this many rotations.");

    if (wheelStartCancelled(0) || odrive_data[0].last_heartbeat.Axis_Error != 0 ||
        odrive_data[0].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
        stopLocalWheels(); return false;
    }
    if (!odrives[0]->setVelocity(signed_velocity, 0.0f)) { stopLocalWheels(); return false; }

    const uint32_t timeout_ms = (uint32_t)((target_delta / speed_turns_per_sec + 3.0f) * 1000.0f);
    const uint32_t start_ms = millis();
    uint32_t next_print_ms = 0;
    while (millis() - start_ms < timeout_ms) {
        pumpCanOnce();
        if (stopRequestedFromSerial() || !wheelMotionAllowed() || odrive_data[0].last_heartbeat.Axis_Error != 0 ||
            odrive_data[0].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
            stopLocalWheels();
            Serial.println("Wheel turn test aborted: Stop or CAN connection lost.");
            return false;
        }

        const uint32_t elapsed_ms = millis() - start_ms;
        if (elapsed_ms >= next_print_ms) {
            Get_Encoder_Estimates_msg_t feedback;
            if (odrives[0]->getFeedback(feedback, 50)) {
                const float moved = feedback.Pos_Estimate - start_feedback.Pos_Estimate;
                const float moved_abs = fabsf(moved);
                const float remaining = max(0.0f, target_delta - moved_abs);
                Serial.printf("turn test t=%.1fs moved=%.4f turns remaining=%.4f actual_rpm=%.2f\n",
                    (float)elapsed_ms / 1000.0f,
                    moved,
                    remaining,
                    feedback.Vel_Estimate * 60.0f);
                if (moved_abs >= target_delta) {
                    stopLocalWheels();
                    Serial.println("Wheel turn test complete.");
                    return true;
                }
            }
            next_print_ms += ODRIVE_MONITOR_INTERVAL_MS;
        }
        delay(10);
    }

    stopLocalWheels();
    Serial.println("Wheel turn test timeout.");
    return false;
}

void tcaSelect(uint8_t channel) {
    if (channel > 7) {
        return;
    }
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

bool setupTofSensors() {
    bool any_ok = false;
    for (uint8_t i = 0; i < NUM_TOF_SENSORS; ++i) {
        tcaSelect(TOF_MUX_CHANNEL);
        tof_sensor_ok[i] = tof_sensors[i].begin(0x29, false, &Wire);
        any_ok = any_ok || tof_sensor_ok[i];
        Serial.printf("tof%d: %s\n", i, tof_sensor_ok[i] ? "ok" : "not found");
    }
    return any_ok;
}

bool ensureTofInitialized() {
    if (tof_initialized) {
        bool any_ok = false;
        for (uint8_t i = 0; i < NUM_TOF_SENSORS; ++i) {
            any_ok = any_ok || tof_sensor_ok[i];
        }
        return any_ok;
    }

    tof_initialized = true;
    Serial.println("Initializing ToF/I2C now...");
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setTimeOut(50);
    return setupTofSensors();
}

void readTofOnce(bool verbose_missing = true) {
    if (!ensureTofInitialized()) {
        if (verbose_missing) {
            Serial.println("No ToF sensor found. Check A4 SDA, A5 SCL, 3.3V, GND, and TCA channel.");
        }
        return;
    }

    VL53L0X_RangingMeasurementData_t measure;
    for (uint8_t i = 0; i < NUM_TOF_SENSORS; ++i) {
        if (!tof_sensor_ok[i]) {
            if (verbose_missing) {
                Serial.printf("tof%d: not found\n", i);
            }
            continue;
        }
        tcaSelect(TOF_MUX_CHANNEL);
        tof_sensors[i].rangingTest(&measure, false);
        if (measure.RangeStatus != 4) {
            Serial.printf("tof%d: %.3f m\n", i, (float)measure.RangeMilliMeter / 1000.0f);
        } else {
            Serial.printf("tof%d: out of range\n", i);
        }
    }
}

bool readActuatorEncoderOnce(int32_t* ticks_out, uint8_t* status_out = nullptr) {
    uint8_t status = 0;
    bool valid = false;
    int32_t enc = actuator_motor_channel == 2
        ? (int32_t)roboclaw.ReadEncM2(roboclaw_addr, &status, &valid)
        : (int32_t)roboclaw.ReadEncM1(roboclaw_addr, &status, &valid);

    if (status_out != nullptr) {
        *status_out = status;
    }
    if (valid && ticks_out != nullptr) {
        *ticks_out = enc;
    }
    return valid;
}

bool readActuatorEncoderRobust(int32_t* ticks_out, uint8_t* status_out = nullptr, uint8_t* successes_out = nullptr) {
    uint8_t successes = 0;
    int32_t last_ticks = 0;
    uint8_t last_status = 0;

    for (uint8_t attempt = 0; attempt < ACTUATOR_ENCODER_READ_ATTEMPTS; ++attempt) {
        roboclaw.clear();
        delay(5);

        int32_t ticks = 0;
        uint8_t status = 0;
        if (readActuatorEncoderOnce(&ticks, &status)) {
            ++successes;
            last_ticks = ticks;
            last_status = status;
        } else {
            last_status = status;
        }
        delay(ACTUATOR_ENCODER_READ_RETRY_DELAY_MS);
    }

    if (successes_out != nullptr) {
        *successes_out = successes;
    }
    if (status_out != nullptr) {
        *status_out = last_status;
    }
    if (successes > 0 && ticks_out != nullptr) {
        *ticks_out = last_ticks;
    }
    return successes > 0;
}

int32_t readActuatorTicks(bool* valid_out = nullptr) {
    int32_t enc = 0;
    bool valid = readActuatorEncoderRobust(&enc);
    if (valid_out != nullptr) {
        *valid_out = valid;
    }
    return valid ? enc : 0;
}

void printActuatorEncoderSnapshot() {
    int32_t active_ticks = 0;
    uint8_t active_status = 0;
    uint8_t active_successes = 0;
    bool active_valid = readActuatorEncoderRobust(&active_ticks, &active_status, &active_successes);
    Serial.printf(
        "active encoder M%u: %s%ld status=0x%02x valid_reads=%u/%u\n",
        actuator_motor_channel,
        active_valid ? "" : "invalid ",
        (long)active_ticks,
        active_status,
        active_successes,
        ACTUATOR_ENCODER_READ_ATTEMPTS
    );

    uint8_t enc1_status = 0;
    uint8_t enc2_status = 0;
    uint8_t speed1_status = 0;
    uint8_t speed2_status = 0;
    bool enc1_valid = false;
    bool enc2_valid = false;
    bool speed1_valid = false;
    bool speed2_valid = false;

    int32_t enc1 = (int32_t)roboclaw.ReadEncM1(roboclaw_addr, &enc1_status, &enc1_valid);
    int32_t enc2 = (int32_t)roboclaw.ReadEncM2(roboclaw_addr, &enc2_status, &enc2_valid);
    int32_t speed1 = (int32_t)roboclaw.ReadSpeedM1(roboclaw_addr, &speed1_status, &speed1_valid);
    int32_t speed2 = (int32_t)roboclaw.ReadSpeedM2(roboclaw_addr, &speed2_status, &speed2_valid);

    Serial.printf(
        "encoder M1: %s%ld status=0x%02x | speed M1: %s%ld status=0x%02x\n",
        enc1_valid ? "" : "invalid ",
        (long)enc1,
        enc1_status,
        speed1_valid ? "" : "invalid ",
        (long)speed1,
        speed1_status
    );
    Serial.printf(
        "encoder M2: %s%ld status=0x%02x | speed M2: %s%ld status=0x%02x\n",
        enc2_valid ? "" : "invalid ",
        (long)enc2,
        enc2_status,
        speed2_valid ? "" : "invalid ",
        (long)speed2,
        speed2_status
    );
}

void watchActuatorEncoders(uint32_t duration_ms = 10000) {
    Serial.println("Watching RoboClaw encoder readback for 10 seconds. Type stop to exit early.");
    Serial.println("No actuator motion is commanded by this watch. Move/pulse the actuator separately if needed.");

    const uint32_t start_ms = millis();
    uint32_t last_print_ms = 0;
    while (millis() - start_ms < duration_ms) {
        if (millis() - last_print_ms >= 500) {
            last_print_ms = millis();
            Serial.printf("t=%.1fs ", (float)(millis() - start_ms) / 1000.0f);
            printActuatorEncoderSnapshot();
        }
        if (stopRequestedFromSerial()) {
            Serial.println("Encoder watch stopped by command.");
            return;
        }
        pumpCanOnce();
        delay(10);
    }

    Serial.println("Encoder watch complete.");
}

bool resetActuatorEncoderCounters(bool print_result = true) {
    bool ok = false;
    for (uint8_t attempt = 0; attempt < ACTUATOR_COMMAND_ATTEMPTS; ++attempt) {
        ok = roboclaw.ResetEncoders(roboclaw_addr);
        if (ok) {
            break;
        }
        roboclaw.clear();
        delay(ACTUATOR_COMMAND_RETRY_DELAY_MS);
    }
    if (print_result) {
        Serial.printf("RoboClaw ResetEncoders command: %s\n", ok ? "acknowledged" : "no response");
    }
    delay(100);
    return ok;
}

void resetActuatorEncoders() {
    resetActuatorEncoderCounters(true);
    printActuatorEncoderSnapshot();
}

bool stopRequestedFromSerial() {
    if (!Serial.available()) {
        return false;
    }
    String line = Serial.readStringUntil('\n');
    line.trim();
    line.toLowerCase();
    return line == "s" || line == "stop" || line == "estop";
}

uint8_t clampActuatorPower(int power) {
    return (uint8_t)constrain(abs(power), 0, 127);
}

#include "actuator_current_limit_runtime.h"
#include "actuator_voltage_limit_runtime.h"

bool actuatorDriveChannelOpenLoop(uint8_t channel, int power) {
    uint8_t command = clampActuatorPower(power);
    if (command == 0) {
        bool stopped_forward = channel == 2 ? roboclaw.ForwardM2(roboclaw_addr, 0) : roboclaw.ForwardM1(roboclaw_addr, 0);
        bool stopped_backward = channel == 2 ? roboclaw.BackwardM2(roboclaw_addr, 0) : roboclaw.BackwardM1(roboclaw_addr, 0);
#if UNIT_ID != 1
        if (channel == 2) {
            roboclaw.SpeedM2(roboclaw_addr, 0);
        } else {
            roboclaw.SpeedM1(roboclaw_addr, 0);
        }
#endif
        // Arm 1 stays in open-loop mode at stop instead of engaging velocity PID.
        return stopped_forward || stopped_backward;
    }

    if (channel != 1 && channel != 2) return ensureActuatorCurrentCeiling(channel);
    if (actuator_current_guard.failed()) return false;
    if (!ensureActuatorVoltageCeiling()) return false;
    if (!ensureActuatorCurrentCeiling(channel)) return false;

    if (power > 0) {
        return channel == 2 ? roboclaw.ForwardM2(roboclaw_addr, command) : roboclaw.ForwardM1(roboclaw_addr, command);
    }
    return channel == 2 ? roboclaw.BackwardM2(roboclaw_addr, command) : roboclaw.BackwardM1(roboclaw_addr, command);
}

bool actuatorManualDrivePermitted(int power) {
    if (power == 0 || !actuator_telemetry.manual) return true;
    if (actuator_telemetry.fault) {
        Serial.println("Actuator telemetry fault: press Stop before another move."); return false;
    }
    if (actuator_telemetry.manual_power > 40) {
        if (!ensureActuatorVoltageCeiling()) return false;
        if (const char* why = actuator_telemetry.raisedPowerCheck(actuator_telemetry.manual_power, millis())) {
            Serial.println(why); return false;
        }
    }
    return true;
}

bool actuatorDriveOpenLoop(int power) {
    if (!actuatorManualDrivePermitted(power)) return false;
    const bool ok = actuatorDriveChannelOpenLoop(actuator_motor_channel, power);
    // Operator cancellation is not a failed motor/controller acknowledgement.
    if(power == 0 || ok || !(actuator_current_guard.cancelled() || actuator_voltage_guard.cancelled()))
        actuator_telemetry.commandResult(power, ok, millis());
    return ok;
}

int rampTowardPower(int current_power, int target_power) {
    if (current_power < target_power) {
        return min(current_power + (int)ACTUATOR_RAMP_STEP, target_power);
    }
    if (current_power > target_power) {
        return max(current_power - (int)ACTUATOR_RAMP_STEP, target_power);
    }
    return current_power;
}

int actuatorCenterPowerForError(int32_t error, int max_power) {
    int abs_error = abs(error);
    int magnitude = abs(max_power);
    if (abs_error <= ACTUATOR_ENCODER_CENTER_SLOW_TICKS) {
        magnitude = min(magnitude, (int)ACTUATOR_ENCODER_CENTER_SLOW_POWER);
    } else if (abs_error <= ACTUATOR_ENCODER_CENTER_MED_TICKS) {
        magnitude = min(magnitude, (int)ACTUATOR_ENCODER_CENTER_MED_POWER);
    }
    return max_power < 0 ? -magnitude : magnitude;
}

bool actuatorDriveChannelOpenLoopWithRetry(uint8_t channel, int power) {
    bool command_ok = false;
    for (uint8_t attempt = 0; attempt < ACTUATOR_COMMAND_ATTEMPTS; ++attempt) {
        command_ok = actuatorDriveChannelOpenLoop(channel, power);
        if (command_ok) {
            return true;
        }
        // A latched guard refusal or pending Stop cannot improve by retrying.
        // Zero packets always retain the existing stop retries.
        if(power != 0 && (actuator_current_guard.failed() || actuator_current_guard.cancelled() || actuator_voltage_guard.failed() || actuator_voltage_guard.cancelled()))
            return false;
        roboclaw.clear();
        delay(ACTUATOR_COMMAND_RETRY_DELAY_MS);
    }
    return command_ok;
}

bool actuatorDriveOpenLoopWithRetry(int power) {
    if (!actuatorManualDrivePermitted(power)) return false;
    const bool ok = actuatorDriveChannelOpenLoopWithRetry(actuator_motor_channel, power);
    // Operator cancellation is not a failed motor/controller acknowledgement.
    if(power == 0 || ok || !(actuator_current_guard.cancelled() || actuator_voltage_guard.cancelled()))
        actuator_telemetry.commandResult(power, ok, millis());
    return ok;
}

void beginRoboClawSerial(uint32_t baudrate, int rx_pin, int tx_pin) {
    RoboSerial.end();
    delay(20);
    RoboSerial.begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);
    delay(100);
    roboclaw.clear();
    roboclaw_safety_timeout_ok = false;
    roboclaw_baudrate = baudrate;
    roboclaw_rx_pin = rx_pin;
    roboclaw_tx_pin = tx_pin;
}

void printRoboClawSerialConfig() {
    Serial.printf(
        "RoboClaw serial config: addr=0x%02x baud=%lu RX=D%d(GPIO%d) TX=D%d(GPIO%d) actuator=M%u\n",
        roboclaw_addr,
        (unsigned long)roboclaw_baudrate,
        roboclaw_rx_pin,
        digitalPinToGPIONumber(roboclaw_rx_pin),
        roboclaw_tx_pin,
        digitalPinToGPIONumber(roboclaw_tx_pin),
        actuator_motor_channel
    );
}

bool probeRoboClawAddress(uint8_t address) {
    char version[64] = {0};
    if (roboclaw.ReadVersion(address, version)) {
        return true;
    }

    bool valid = false;
    (void)roboclaw.ReadMainBatteryVoltage(address, &valid);
    return valid;
}

uint16_t roboclawCrcUpdate(uint16_t crc, uint8_t data) {
    crc = crc ^ ((uint16_t)data << 8);
    for (uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x8000) {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc <<= 1;
        }
    }
    return crc;
}

bool roboclawWritePacketAck(uint8_t command, const uint8_t* data, uint8_t data_len) {
    bool ok = false;
    for (uint8_t attempt = 0; attempt < ACTUATOR_COMMAND_ATTEMPTS; ++attempt) {
        roboclaw.clear();
        delay(12);  // Clear any incomplete RoboClaw packet.

        uint16_t crc = 0;
        RoboSerial.write(roboclaw_addr);
        crc = roboclawCrcUpdate(crc, roboclaw_addr);
        RoboSerial.write(command);
        crc = roboclawCrcUpdate(crc, command);
        for (uint8_t i = 0; i < data_len; ++i) {
            RoboSerial.write(data[i]);
            crc = roboclawCrcUpdate(crc, data[i]);
        }
        RoboSerial.write((uint8_t)(crc >> 8));
        RoboSerial.write((uint8_t)crc);
        RoboSerial.flush();

        int ack = roboclaw.read(10000);
        if (ack == 0xFF) {
            ok = true;
            break;
        }
    }
    return ok;
}

bool setRoboClawSerialTimeout(uint8_t timeout_tenths) {
    uint8_t data[] = {timeout_tenths};
    bool ok = roboclawWritePacketAck(14, data, sizeof(data));
    roboclaw_safety_timeout_ok = ok;
    return ok;
}

bool ensureRoboClawSafetyTimeout() {
    if (roboclaw_safety_timeout_ok) {
        return true;
    }
    bool ok = setRoboClawSerialTimeout(ROBOCLAW_SERIAL_TIMEOUT_TENTHS);
    if (!ok) {
        actuatorStop();
        Serial.println("RoboClaw serial safety timeout did not acknowledge. Refusing actuator motion.");
    }
    return ok;
}

void actuatorStop() {
    if (actuator_telemetry.drain_required) {
        delay(12); roboclaw.clear(); // Remove a late telemetry reply before ACK reads.
        actuator_telemetry.drain_required = false;
    }
    const bool first = actuatorDriveChannelOpenLoopWithRetry(1, 0);
    const bool second = actuatorDriveChannelOpenLoopWithRetry(2, 0);
    actuator_telemetry.stopResult(first && second);
}

void resetRoboClawSerialLink(bool verbose = true) {
    if (verbose) {
        Serial.println("Resetting RoboClaw serial link and sending stop commands...");
        printRoboClawSerialConfig();
    }

    actuatorStop();
    roboclaw.clear();
    delay(ROBOCLAW_SERIAL_RESET_DELAY_MS);

    beginRoboClawSerial(roboclaw_baudrate, roboclaw_rx_pin, roboclaw_tx_pin);
    roboclaw.clear();
    delay(ROBOCLAW_SERIAL_RESET_DELAY_MS);

    actuatorStop();
    roboclaw.clear();
    bool timeout_ok = setRoboClawSerialTimeout(ROBOCLAW_SERIAL_TIMEOUT_TENTHS);

    if (verbose) {
        Serial.printf("RoboClaw serial deadman timeout: %s (%u ms).\n",
            timeout_ok ? "enabled" : "not acknowledged",
            (unsigned int)ROBOCLAW_SERIAL_TIMEOUT_TENTHS * 100);
        Serial.println("RoboClaw serial reset complete. Actuator should be stopped.");
    }
}

bool actuatorPreflight() {
    resetRoboClawSerialLink(false);
    delay(ACTUATOR_PREFLIGHT_DELAY_MS);

    bool valid = false;
    (void)roboclaw.ReadMainBatteryVoltage(roboclaw_addr, &valid);
    roboclaw.clear();
    return ensureRoboClawSafetyTimeout();
}

void printRoboClawDiagnostics() {
    Serial.println("\n--- RoboClaw diagnostics ---");
    printRoboClawSerialConfig();

    char version[64] = {0};
    bool version_ok = roboclaw.ReadVersion(roboclaw_addr, version);
    Serial.printf("version: %s%s\n", version_ok ? "" : "no response ", version_ok ? version : "");

    bool valid_main = false;
    uint16_t main_mv10 = roboclaw.ReadMainBatteryVoltage(roboclaw_addr, &valid_main);
    Serial.printf("main battery: %s", valid_main ? "" : "invalid ");
    Serial.printf("%.1f V\n", (float)main_mv10 / 10.0f);

    bool valid_logic = false;
    uint16_t logic_mv10 = roboclaw.ReadLogicBatteryVoltage(roboclaw_addr, &valid_logic);
    Serial.printf("logic battery: %s", valid_logic ? "" : "invalid ");
    Serial.printf("%.1f V\n", (float)logic_mv10 / 10.0f);

    bool valid_error = false;
    uint32_t error = roboclaw.ReadError(roboclaw_addr, &valid_error);
    Serial.printf("error: %s0x%08lx\n", valid_error ? "" : "invalid ", (unsigned long)error);

    uint8_t enc_status = 0;
    bool valid_enc = false;
    int32_t enc = (int32_t)roboclaw.ReadEncM1(roboclaw_addr, &enc_status, &valid_enc);
    Serial.printf("encoder M1: %s%ld status=0x%02x\n", valid_enc ? "" : "invalid ", (long)enc, enc_status);

    uint8_t enc2_status = 0;
    bool valid_enc2 = false;
    int32_t enc2 = (int32_t)roboclaw.ReadEncM2(roboclaw_addr, &enc2_status, &valid_enc2);
    Serial.printf("encoder M2: %s%ld status=0x%02x\n", valid_enc2 ? "" : "invalid ", (long)enc2, enc2_status);

    uint8_t speed_status = 0;
    bool valid_speed = false;
    int32_t speed = (int32_t)roboclaw.ReadSpeedM1(roboclaw_addr, &speed_status, &valid_speed);
    Serial.printf("speed M1: %s%ld status=0x%02x\n", valid_speed ? "" : "invalid ", (long)speed, speed_status);

    uint8_t speed2_status = 0;
    bool valid_speed2 = false;
    int32_t speed2 = (int32_t)roboclaw.ReadSpeedM2(roboclaw_addr, &speed2_status, &valid_speed2);
    Serial.printf("speed M2: %s%ld status=0x%02x\n", valid_speed2 ? "" : "invalid ", (long)speed2, speed2_status);

    int16_t pwm1 = 0;
    int16_t pwm2 = 0;
    if (roboclaw.ReadPWMs(roboclaw_addr, pwm1, pwm2)) {
        Serial.printf("PWM M1=%d M2=%d\n", pwm1, pwm2);
    } else {
        Serial.println("PWM read: no response");
    }

    int16_t current1 = 0;
    int16_t current2 = 0;
    if (roboclaw.ReadCurrents(roboclaw_addr, current1, current2)) {
        Serial.printf("current M1=%.2f A M2=%.2f A\n", (float)current1 / 100.0f, (float)current2 / 100.0f);
    } else {
        Serial.println("current read: no response");
    }

    Serial.println("----------------------------\n");
}

void scanRoboClaw() {
    struct PinPair {
        const char* label;
        int rx_pin;
        int tx_pin;
    };

    static const PinPair pin_pairs[] = {
        {"configured D7 RX / D6 TX", PIN_ROBO_RX, PIN_ROBO_TX},
        {"swapped D6 RX / D7 TX", PIN_ROBO_TX, PIN_ROBO_RX},
        {"D0 RX / D1 TX", D0, D1},
        {"swapped D1 RX / D0 TX", D1, D0},
    };
    static const uint32_t baudrates[] = {ROBO_BAUDRATE, 115200, 57600, 19200, 9600, 2400, 230400, 460800};
    static const uint8_t addresses[] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87};

    Serial.println("\n--- RoboClaw scan ---");
    Serial.println("Scanning common pin pairs, baud rates, and packet-serial addresses.");
    Serial.println("This is read-only. No actuator motion commands are sent.");

    for (const PinPair& pair: pin_pairs) {
        for (uint32_t baud: baudrates) {
            beginRoboClawSerial(baud, pair.rx_pin, pair.tx_pin);
            Serial.printf(
                "Trying %s, baud %lu, RX=D%d(GPIO%d), TX=D%d(GPIO%d)\n",
                pair.label,
                (unsigned long)baud,
                pair.rx_pin,
                digitalPinToGPIONumber(pair.rx_pin),
                pair.tx_pin,
                digitalPinToGPIONumber(pair.tx_pin)
            );

            for (uint8_t address: addresses) {
                if (probeRoboClawAddress(address)) {
                    roboclaw_addr = address;
                    Serial.printf("FOUND RoboClaw at addr=0x%02x using %s, baud %lu.\n",
                        roboclaw_addr,
                        pair.label,
                        (unsigned long)baud);
                    printRoboClawDiagnostics();
                    return;
                }
            }
        }
    }

    beginRoboClawSerial(ROBO_BAUDRATE, PIN_ROBO_RX, PIN_ROBO_TX);
    roboclaw_addr = ROBOCLAW_ADDR;
    Serial.println("No RoboClaw response found.");
    Serial.println("Check RoboClaw packet serial mode/address, ESP32 TX/RX wiring, RoboClaw logic/main power, and common ground.");
    Serial.println("---------------------\n");
}

const char* limitStateText(int pin) {
    return digitalRead(pin) == LOW ? "PRESSED" : "open";
}

bool limitPressedDebounced(int pin, uint32_t stable_ms = ACTUATOR_LIMIT_DEBOUNCE_MS) {
    if (digitalRead(pin) != LOW) {
        return false;
    }

    const uint32_t start_ms = millis();
    while (millis() - start_ms < stable_ms) {
        if (digitalRead(pin) != LOW) {
            return false;
        }
        delay(2);
    }
    return true;
}

void printLimitSnapshot() {
    int rev_raw = digitalRead(PIN_ACT_LIMIT_IN);
    int fwd_raw = digitalRead(PIN_ACT_LIMIT_OUT);
    Serial.printf(
        "limits unit=%d REV/IN pin=%d raw=%d %s | FWD/OUT pin=%d raw=%d %s\n",
        UNIT_ID,
        PIN_ACT_LIMIT_IN,
        rev_raw,
        rev_raw == LOW ? "PRESSED" : "open",
        PIN_ACT_LIMIT_OUT,
        fwd_raw,
        fwd_raw == LOW ? "PRESSED" : "open"
    );
}

void watchLimitSwitches(uint32_t duration_ms = 20000) {
    Serial.println("Watching limit switches for 20 seconds. Press switches by hand. Type stop to exit early.");
    Serial.println("raw=1 means open with INPUT_PULLUP. raw=0 means the switch is connecting the pin to GND.");
    int last_rev = -1;
    int last_fwd = -1;
    uint32_t last_print_ms = 0;
    const uint32_t start_ms = millis();

    while (millis() - start_ms < duration_ms) {
        int rev_raw = digitalRead(PIN_ACT_LIMIT_IN);
        int fwd_raw = digitalRead(PIN_ACT_LIMIT_OUT);
        bool changed = rev_raw != last_rev || fwd_raw != last_fwd;
        bool periodic = millis() - last_print_ms >= 1000;

        if (changed || periodic) {
            printLimitSnapshot();
            last_rev = rev_raw;
            last_fwd = fwd_raw;
            last_print_ms = millis();
        }

        if (stopRequestedFromSerial()) {
            Serial.println("Limit watch stopped.");
            return;
        }

        pumpCanOnce();
        delay(25);
    }

    Serial.println("Limit watch complete.");
}

bool actuatorMoveUntilLimit(const char* label, int limit_pin, int motor_power, uint32_t timeout_ms, uint32_t* elapsed_ms_out = nullptr) {
    Serial.printf("Actuator %s at open-loop power %+d until limit pin %d...\n", label, motor_power, limit_pin);
    const uint32_t start_ms = millis();
    const uint32_t already_pressed_ms = ACTUATOR_UNIT1_EXPERIMENTAL_HOMING ? ACTUATOR_LIMIT_CONFIRM_MS : ACTUATOR_LIMIT_DEBOUNCE_MS;
    if (limitPressedDebounced(limit_pin, already_pressed_ms)) {
        actuatorStop();
        if (elapsed_ms_out != nullptr) {
            *elapsed_ms_out = 0;
        }
        Serial.printf("Actuator %s limit is already pressed.\n", label);
        return true;
    }
    if (!ensureRoboClawSafetyTimeout()) {
        if (elapsed_ms_out != nullptr) {
            *elapsed_ms_out = millis() - start_ms;
        }
        return false;
    }
    int applied_power = rampTowardPower(0, motor_power);
    bool command_ok = actuatorDriveOpenLoopWithRetry(applied_power);
    if (!command_ok) { actuatorStop(); return false; }
    Serial.printf("RoboClaw drive command: %s\n", command_ok ? "sent" : "no response");
    uint32_t last_refresh_ms = millis();
    bool creep_after_touch = false;

    while (true) {
        if (stopRequestedFromSerial()) {
            actuatorStop();
            if (elapsed_ms_out != nullptr) {
                *elapsed_ms_out = millis() - start_ms;
            }
            Serial.println("Actuator move aborted by stop command.");
            return false;
        }
        if (limitPressedDebounced(limit_pin)) {
            if (!ACTUATOR_UNIT1_EXPERIMENTAL_HOMING) {
                actuatorStop();
                if (elapsed_ms_out != nullptr) {
                    *elapsed_ms_out = millis() - start_ms;
                }
                Serial.printf("Actuator %s limit reached.\n", label);
                return true;
            }

            actuatorStop();
            delay(ACTUATOR_LIMIT_CONFIRM_MS);
            if (limitPressedDebounced(limit_pin, ACTUATOR_LIMIT_CONFIRM_MS)) {
                if (elapsed_ms_out != nullptr) {
                    *elapsed_ms_out = millis() - start_ms;
                }
                Serial.printf("Actuator %s limit reached and confirmed.\n", label);
                return true;
            }

            creep_after_touch = true;
            Serial.printf("Actuator %s ignored transient limit trigger; creeping into limit.\n", label);
            applied_power = 0;
            last_refresh_ms = 0;
        }
        pumpCanOnce();
        if (actuator_telemetry.manual && actuator_telemetry.fault) {
            actuatorStop();
            Serial.println("Actuator stop requested: telemetry reply failed. Press Stop to retry.");
            return false;
        }
        if (millis() - last_refresh_ms >= ACTUATOR_COMMAND_REFRESH_MS) {
            last_refresh_ms = millis();
            int target_power = motor_power;
            if (creep_after_touch) {
                target_power = motor_power > 0 ? ACTUATOR_LIMIT_CREEP_POWER : -ACTUATOR_LIMIT_CREEP_POWER;
            }
            applied_power = rampTowardPower(applied_power, target_power);
            if (!actuatorDriveOpenLoop(applied_power)) {
                actuatorStop();
                if (elapsed_ms_out != nullptr) {
                    *elapsed_ms_out = millis() - start_ms;
                }
                Serial.println((actuator_current_guard.cancelled() || actuator_voltage_guard.cancelled())
                    ? "Actuator move cancelled by incoming USB command."
                    : "Actuator move aborted: RoboClaw command refresh failed.");
                return false;
            }
        }
        if (millis() - start_ms > timeout_ms) {
            actuatorStop();
            if (elapsed_ms_out != nullptr) {
                *elapsed_ms_out = millis() - start_ms;
            }
            Serial.printf("Actuator %s timeout. Limit switch works, so check motor direction, RoboClaw power, or RoboClaw serial.\n", label);
            return false;
        }
        delay(5);
    }
}

bool actuatorPulseChannel(const char* label, uint8_t channel, int motor_power, uint32_t duration_ms) {
    Serial.print("Actuator pulse starting ");
    printLimitSnapshot();

    if (motor_power > 0 && limitPressedDebounced(PIN_ACT_LIMIT_OUT)) {
        Serial.println("Refusing forward pulse because FWD/OUT limit is pressed.");
        Serial.print("Actuator pulse ending ");
        printLimitSnapshot();
        return false;
    }
    if (motor_power < 0 && limitPressedDebounced(PIN_ACT_LIMIT_IN)) {
        Serial.println("Refusing reverse pulse because REV/IN limit is pressed.");
        Serial.print("Actuator pulse ending ");
        printLimitSnapshot();
        return false;
    }

    Serial.printf("Actuator pulse %s on RoboClaw M%u at open-loop power %+d for %.2f seconds...\n",
        label,
        channel,
        motor_power,
        (float)duration_ms / 1000.0f);

    if (!ensureRoboClawSafetyTimeout()) {
        return false;
    }
    int applied_power = rampTowardPower(0, motor_power);
    bool command_ok = actuatorDriveChannelOpenLoopWithRetry(channel, applied_power);
    Serial.printf("RoboClaw drive command: %s\n", command_ok ? "sent" : "no response");

    const uint32_t start_ms = millis();
    uint32_t last_refresh_ms = start_ms;
    while (millis() - start_ms < duration_ms) {
        if (millis() - last_refresh_ms >= ACTUATOR_COMMAND_REFRESH_MS) {
            last_refresh_ms = millis();
            applied_power = rampTowardPower(applied_power, motor_power);
            if (!actuatorDriveChannelOpenLoop(channel, applied_power)) {
                Serial.println("Pulse stopped because RoboClaw command refresh failed.");
                break;
            }
        }
        if ((motor_power > 0 && limitPressedDebounced(PIN_ACT_LIMIT_OUT)) ||
            (motor_power < 0 && limitPressedDebounced(PIN_ACT_LIMIT_IN))) {
            Serial.println("Pulse stopped by limit switch.");
            break;
        }
        if (stopRequestedFromSerial()) {
            Serial.println("Pulse stopped by stop command.");
            break;
        }
        pumpCanOnce();
        delay(10);
    }

    actuatorStop();
    Serial.print("Actuator pulse ending ");
    printLimitSnapshot();
    Serial.println("Actuator pulse complete.");
    return command_ok;
}

bool actuatorPulse(const char* label, int motor_power, uint32_t duration_ms) {
    return actuatorPulseChannel(label, actuator_motor_channel, motor_power, duration_ms);
}

bool actuatorTimedCenterFromRev(uint32_t duration_ms) {
    if (duration_ms > ACTUATOR_CENTER_FALLBACK_MAX_MS) {
        Serial.printf(
            "Timed center fallback capped from %.2f to %.2f seconds for safety.\n",
            (float)duration_ms / 1000.0f,
            (float)ACTUATOR_CENTER_FALLBACK_MAX_MS / 1000.0f
        );
        duration_ms = ACTUATOR_CENTER_FALLBACK_MAX_MS;
    }

    Serial.printf(
        "Timed center fallback: moving FWD from REV limit at power +%u for %.2f seconds.\n",
        ACTUATOR_CENTER_FALLBACK_POWER,
        (float)duration_ms / 1000.0f
    );

    if (!ensureRoboClawSafetyTimeout()) {
        return false;
    }
    bool command_ok = actuatorDriveOpenLoopWithRetry(ACTUATOR_CENTER_FALLBACK_POWER);
    Serial.printf("RoboClaw drive command: %s\n", command_ok ? "sent" : "no response");

    bool rev_released = !limitPressedDebounced(PIN_ACT_LIMIT_IN);
    const uint32_t start_ms = millis();
    uint32_t last_refresh_ms = start_ms;
    while (millis() - start_ms < duration_ms) {
        const bool rev_pressed = limitPressedDebounced(PIN_ACT_LIMIT_IN);
        const bool fwd_pressed = limitPressedDebounced(PIN_ACT_LIMIT_OUT);

        if (millis() - last_refresh_ms >= ACTUATOR_COMMAND_REFRESH_MS) {
            last_refresh_ms = millis();
            if (!actuatorDriveOpenLoop(ACTUATOR_CENTER_FALLBACK_POWER)) {
                actuatorStop();
                Serial.println("Timed center fallback aborted: RoboClaw command refresh failed.");
                return false;
            }
        }
        if (fwd_pressed) {
            actuatorStop();
            Serial.println("Timed center fallback stopped by FWD/OUT limit. Center estimate is unsafe.");
            return false;
        }
        if (!rev_pressed) {
            rev_released = true;
        }
        if (!rev_released && millis() - start_ms > ACTUATOR_REV_RELEASE_TIMEOUT_MS) {
            actuatorStop();
            Serial.println("Timed center fallback aborted: REV/IN limit did not release. Check actuator direction or switch labels.");
            return false;
        }
        if (rev_released && rev_pressed) {
            actuatorStop();
            Serial.println("Timed center fallback aborted: REV/IN limit re-pressed while centering.");
            return false;
        }
        if (stopRequestedFromSerial()) {
            actuatorStop();
            Serial.println("Timed center fallback aborted by stop command.");
            return false;
        }

        pumpCanOnce();
        delay(10);
    }

    actuatorStop();
    Serial.println("Timed center fallback complete.");
    return command_ok;
}

bool actuatorMoveToTicks(int32_t target_ticks, int32_t fwd_limit_ticks, int32_t rev_limit_ticks, int motor_power, uint32_t timeout_ms) {
    Serial.printf("Actuator moving to center target %ld ticks...\n", (long)target_ticks);
    if (!ensureRoboClawSafetyTimeout()) {
        return false;
    }
    const bool fwd_power_increases_ticks = fwd_limit_ticks > rev_limit_ticks;
    Serial.printf(
        "Actuator encoder direction: FWD power %s encoder ticks.\n",
        fwd_power_increases_ticks ? "increases" : "decreases"
    );
    const uint32_t start_ms = millis();
    uint32_t last_progress_ms = 0;
    uint32_t last_command_ms = 0;
    int applied_power = 0;
    int32_t previous_error = 0;
    bool have_previous_error = false;
    int32_t last_motion_ticks = 0;
    uint32_t last_motion_ms = 0;
    bool have_motion_reference = false;
    bool stall_boost_active = false;

    while (true) {
        bool valid = false;
        int32_t current = readActuatorTicks(&valid);
        if (!valid) {
            actuatorStop();
            Serial.println("Actuator encoder read failed.");
            return false;
        }

        const uint32_t now_ms = millis();
        if (!have_motion_reference || abs(current - last_motion_ticks) >= ACTUATOR_CENTER_STALL_TICKS) {
            last_motion_ticks = current;
            last_motion_ms = now_ms;
            have_motion_reference = true;
            stall_boost_active = false;
        }

        int32_t error = target_ticks - current;
        if (abs(error) <= ACTUATOR_CENTER_TOLERANCE_TICKS) {
            actuatorStop();
            Serial.printf("Actuator centered at %ld ticks, error=%ld ticks.\n", (long)current, (long)error);
            return true;
        }
        if (have_previous_error && ((previous_error > 0 && error < 0) || (previous_error < 0 && error > 0))) {
            actuatorStop();
            Serial.printf("Actuator crossed center target. Stopped at %ld ticks, error=%ld ticks.\n", (long)current, (long)error);
            return true;
        }
        previous_error = error;
        have_previous_error = true;

        int desired_power = 0;
        if (error > 0) {
            desired_power = fwd_power_increases_ticks ? abs(motor_power) : -abs(motor_power);
        } else {
            desired_power = fwd_power_increases_ticks ? -abs(motor_power) : abs(motor_power);
        }
        desired_power = actuatorCenterPowerForError(error, desired_power);
        if (have_motion_reference && now_ms - last_motion_ms >= ACTUATOR_CENTER_STALL_MS) {
            desired_power = desired_power >= 0
                ? ACTUATOR_CENTER_BREAKAWAY_POWER
                : -ACTUATOR_CENTER_BREAKAWAY_POWER;
            if (!stall_boost_active) {
                Serial.printf(
                    "center stall detected near %ld ticks; boosting power to %+d until motion resumes.\n",
                    (long)current,
                    desired_power
                );
                stall_boost_active = true;
            }
        }
        if (millis() - last_progress_ms >= ACTUATOR_CENTER_PROGRESS_MS) {
            last_progress_ms = millis();
            Serial.printf("center progress: current=%ld target=%ld error=%ld power=%+d target_power=%+d\n",
                (long)current,
                (long)target_ticks,
                (long)error,
                applied_power,
                desired_power);
        }
        if (limitPressedDebounced(PIN_ACT_LIMIT_OUT) && desired_power > 0) {
            actuatorStop();
            Serial.printf("Blocked by forward limit while centering at %ld ticks, error=%ld ticks.\n", (long)current, (long)error);
            return false;
        }
        if (limitPressedDebounced(PIN_ACT_LIMIT_IN) && desired_power < 0) {
            actuatorStop();
            Serial.printf("Blocked by reverse limit while centering at %ld ticks, error=%ld ticks.\n", (long)current, (long)error);
            return false;
        }

        if (millis() - last_command_ms >= ACTUATOR_COMMAND_REFRESH_MS) {
            last_command_ms = millis();
            applied_power = rampTowardPower(applied_power, desired_power);
            if (!actuatorDriveOpenLoop(applied_power)) {
                actuatorStop();
                Serial.println("Actuator center aborted: RoboClaw command refresh failed.");
                return false;
            }
        }
        pumpCanOnce();
        if (stopRequestedFromSerial()) {
            actuatorStop();
            Serial.println("Actuator center aborted by stop command.");
            return false;
        }
        if (millis() - start_ms > timeout_ms) {
            actuatorStop();
            Serial.println("Actuator center timeout.");
            return false;
        }
        delay(10);
    }
}

bool measureActuatorRange(int32_t* fwd_ticks_out, int32_t* rev_ticks_out, int32_t* center_ticks_out) {
    uint32_t fwd_elapsed_ms = 0;
    if (!actuatorMoveUntilLimit("FWD/extend", PIN_ACT_LIMIT_OUT, ACTUATOR_STARTUP_POWER, ACTUATOR_LIMIT_TIMEOUT_MS, &fwd_elapsed_ms)) {
        return false;
    }
    Serial.printf("Actuator FWD/extend travel time: %.2f seconds.\n", (float)fwd_elapsed_ms / 1000.0f);

    if (ACTUATOR_UNIT1_EXPERIMENTAL_HOMING) {
        Serial.println("Zeroing actuator encoder at FWD/extend limit.");
        if (!resetActuatorEncoderCounters(false)) {
            Serial.println("Actuator startup refused: RoboClaw did not acknowledge encoder reset at FWD limit.");
            return false;
        }
    }
    delay(250);

    bool valid_fwd = false;
    int32_t fwd_ticks = readActuatorTicks(&valid_fwd);
    if (!valid_fwd) {
        Serial.println("Actuator startup refused: encoder readback unavailable at FWD limit.");
        return false;
    }
    if (ACTUATOR_UNIT1_EXPERIMENTAL_HOMING) {
        Serial.printf("Actuator FWD/extend encoder zero: %ld ticks.\n", (long)fwd_ticks);
    }

    uint32_t rev_elapsed_ms = 0;
    if (!actuatorMoveUntilLimit("REV/retract", PIN_ACT_LIMIT_IN, -ACTUATOR_STARTUP_POWER, ACTUATOR_LIMIT_TIMEOUT_MS, &rev_elapsed_ms)) {
        return false;
    }
    Serial.printf("Actuator REV/retract travel time: %.2f seconds.\n", (float)rev_elapsed_ms / 1000.0f);
    delay(250);

    bool valid_rev = false;
    int32_t rev_ticks = readActuatorTicks(&valid_rev);
    if (!valid_rev) {
        Serial.println("Actuator startup refused: encoder readback unavailable at REV limit.");
        return false;
    }

    *fwd_ticks_out = fwd_ticks;
    *rev_ticks_out = rev_ticks;
    *center_ticks_out = fwd_ticks + ((rev_ticks - fwd_ticks) / 2);
    return true;
}

bool rangeLooksValid(int32_t fwd_ticks, int32_t rev_ticks, int32_t center_ticks) {
    int32_t range_ticks = abs(fwd_ticks - rev_ticks);
    Serial.printf("Actuator range: REV=%ld FWD=%ld center=%ld stroke=%ld ticks\n",
        (long)rev_ticks,
        (long)fwd_ticks,
        (long)center_ticks,
        (long)range_ticks);
    if (range_ticks < ACTUATOR_MIN_ENCODER_RANGE_TICKS || range_ticks > ACTUATOR_MAX_ENCODER_RANGE_TICKS) {
        Serial.printf(
            "Actuator range rejected: stroke must be between %ld and %ld ticks. Possible false limit-switch trigger.\n",
            (long)ACTUATOR_MIN_ENCODER_RANGE_TICKS,
            (long)ACTUATOR_MAX_ENCODER_RANGE_TICKS
        );
        return false;
    }
    return true;
}

bool runActuatorStartup() {
    Serial.printf("Actuator startup using RoboClaw open-loop power %u.\n", ACTUATOR_STARTUP_POWER);
    Serial.println("Note: this is a bring-up command, not calibrated 10 cm/s motion yet.");
    if (!actuatorPreflight()) {
        Serial.println("Actuator startup aborted: RoboClaw serial deadman timeout is not confirmed.");
        return false;
    }

    int32_t fwd_ticks = 0;
    int32_t rev_ticks = 0;
    int32_t center_ticks = 0;
    for (uint8_t attempt = 1; attempt <= ACTUATOR_HOMING_ATTEMPTS; ++attempt) {
        Serial.printf("Actuator homing attempt %u/%u\n", attempt, ACTUATOR_HOMING_ATTEMPTS);
        if (!measureActuatorRange(&fwd_ticks, &rev_ticks, &center_ticks)) {
            return false;
        }
        if (rangeLooksValid(fwd_ticks, rev_ticks, center_ticks)) {
            return actuatorMoveToTicks(center_ticks, fwd_ticks, rev_ticks, ACTUATOR_ENCODER_CENTER_POWER, ACTUATOR_LIMIT_TIMEOUT_MS);
        }
        actuatorStop();
        delay(250);
    }

    Serial.println("Actuator startup failed: could not get a valid full-stroke measurement.");
    return false;
}

bool waitWithStopCheck(uint32_t duration_ms, bool require_wheel) {
    const uint32_t start_ms = millis();
    while (millis() - start_ms < duration_ms) {
        pumpCanOnce();
        if (stopRequestedFromSerial() || (require_wheel && (!wheelMotionAllowed() || odrive_data[0].last_heartbeat.Axis_Error != 0 ||
            odrive_data[0].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL))) {
            stopLocalWheels();
            Serial.println("Wheel test aborted: Stop or CAN connection lost.");
            return false;
        }
        delay(10);
    }
    return true;
}

bool runWheelStartupTest() {
    Serial.println("Starting local ODrive wheel startup test.");
    Serial.printf("Wheel stop revision: %s\n", WHEEL_STOP_REVISION);

    Serial.printf("Forward %.1f RPM for %.1f seconds\n", STARTUP_WHEEL_TEST_RPM, STARTUP_WHEEL_TEST_SECONDS);
    if (!setAllLocalWheelsRpm(STARTUP_WHEEL_TEST_RPM)) {
        stopLocalWheels();
        return false;
    }
    if (!waitWithStopCheck((uint32_t)(STARTUP_WHEEL_TEST_SECONDS * 1000.0f), true)) {
        return false;
    }

    stopLocalWheels();
    if (!waitForWheelStopped(0)) {
        return false;
    }
    Serial.println("Wheel stopped between directions.");
    if (!waitWithStopCheck(STARTUP_WHEEL_STOP_PAUSE_MS, true)) {
        return false;
    }

    Serial.printf("Reverse %.1f RPM for %.1f seconds\n", STARTUP_WHEEL_TEST_RPM, STARTUP_WHEEL_TEST_SECONDS);
    if (!setAllLocalWheelsRpm(-STARTUP_WHEEL_TEST_RPM)) {
        stopLocalWheels();
        return false;
    }
    if (!waitWithStopCheck((uint32_t)(STARTUP_WHEEL_TEST_SECONDS * 1000.0f), true)) {
        return false;
    }

    stopLocalWheels();
    if (!waitForWheelStopped(0)) {
        return false;
    }
    Serial.println("Local ODrive wheel startup test complete. Wheel stopped.");
    return true;
}

void runStartupSequence() {
    Serial.println("STARTUP SEQUENCE BEGIN");
    stopLocalWheels();
    actuatorStop();

    if (!runActuatorStartup()) {
        stopLocalWheels();
        actuatorStop();
        Serial.println("STARTUP FAILED during actuator sequence.");
        return;
    }

    if (!runWheelStartupTest()) {
        stopLocalWheels();
        actuatorStop();
        Serial.println("STARTUP FAILED during wheel test.");
        return;
    }

    tof_enabled = false;
    Serial.println("ToF left off during USB bring-up. Type 'tof init' or 'tof read' to test it.");
    Serial.println("STARTUP SEQUENCE COMPLETE");
}

void printStatus() {
    pumpCanOnce();
    Serial.println("\n--- status ---");
    Serial.printf("unit id: %d\n", UNIT_ID);
    Serial.printf("local ODrive CAN node id: %d\n", LOCAL_ODRIVE_NODE_ID);
#if UNIT_ID == 1
    Serial.println("Actuator stop revision: 2026-09-19-u1-open-loop-stop1");
#endif
    Serial.printf("CAN initialized: %s\n", can_initialized ? "yes" : "no");
    Serial.printf("limit REV/IN pin %d:  %s\n", PIN_ACT_LIMIT_IN, limitStateText(PIN_ACT_LIMIT_IN));
    Serial.printf("limit FWD/OUT pin %d: %s\n", PIN_ACT_LIMIT_OUT, limitStateText(PIN_ACT_LIMIT_OUT));
    bool valid = false;
    int32_t enc = readActuatorTicks(&valid);
    Serial.printf("actuator encoder: %s", valid ? "" : "invalid ");
    Serial.println((long)enc);

    for (int i = 0; i < NUM_LOCAL_ODRIVES; ++i) {
        Serial.printf("local wheel online=%s", odriveOnline(i) ? "yes" : "no");
        if (odrive_data[i].received_heartbeat) {
            Serial.printf(" state=%lu error=%lu", (unsigned long)odrive_data[i].last_heartbeat.Axis_State,
                (unsigned long)odrive_data[i].last_heartbeat.Axis_Error);
        }
        if (odrive_data[i].received_feedback) {
            Serial.printf(" pos=%.3f turns vel=%.3f turns/s",
                odrive_data[i].last_feedback.Pos_Estimate,
                odrive_data[i].last_feedback.Vel_Estimate);
        }
        Serial.println();
    }
    Serial.printf("ToF initialized: %s\n", tof_initialized ? "yes" : "no");
    Serial.printf("ToF enabled: %s\n", tof_enabled ? "yes" : "no");
    if (tof_enabled) {
        readTofOnce();
    }
    Serial.println("--------------\n");
}

void printHelp() {
    Serial.println("\nCommands:");
    Serial.println("  telemetry on|off   USB live ODrive data every 200 ms; no motor setpoints");
    Serial.println("  unit               print this ESP32 unit id and local ODrive node id");
    Serial.println("  firmware           print installed revision; no device commands");
    Serial.println("  start              local actuator FWD limit, REV limit, center, local wheel RPM test");
    Serial.println("  stop               stop local wheel and actuator");
    Serial.println("  status             print local limits, encoder, ODrive, and ToF status");
    Serial.println("  limits             watch limit switch inputs for 20 seconds");
    Serial.println("  can init           initialize CAN and listen for local ODrive");
    Serial.println("  can status         print ESP32 CAN controller error counters");
    Serial.println("  can reset          reset ESP32 CAN after BUS_OFF or wiring changes");
    Serial.println("  can beacon         send CAN frames to wake ODrive auto-baud");
    Serial.println("  can watch          print any raw CAN frames seen for 5 seconds");
    Serial.println("  tof init           initialize ToF/I2C");
    Serial.println("  tof on|off|read    enable, disable, or read ToF sensor");
    Serial.println("  wheel current      read verified ODrive soft/hard current settings");
    Serial.println("  wheel current apply  set/verify 35 A soft / 50 A hard in IDLE; no motion");
    Serial.println("  wheel tune         read ODrive tuning, bus limits, and encoder status");
    Serial.println("  wheel tune apply   apply climbing velocity/spinout preset + 35/50 A in RAM");
    Serial.println("  wheel tune set P V set one tuning/bus parameter P to value V in RAM");
    Serial.println("  wheel tune save    save verified 35/50 A & tuning to ODrive flash; resets ODrive");
    Serial.println("  wheel cal          run guarded encoder offset calibration (state 7) in RAM");
    Serial.println("  wheel diag         print ODrive version, errors, voltage, encoder, current, temp");
    Serial.println("  wheel clear/recover  explicit zero/IDLE, clear, verify; never starts wheel");
    Serial.println("  wheel idle         stop wheel and put local ODrive in IDLE");
    Serial.println("  wheel monitor      print actual RPM/current for 5 seconds");
    Serial.println("  wheel turn 1       move exactly +1 encoder turn slowly");
    Serial.println("  wheel turn -1      move exactly -1 encoder turn slowly");
    Serial.println("  wheel test         gentle +5 RPM wheel jog for 2 seconds");
    Serial.println("  wheel test 10rpm   gentle wheel jog at requested RPM for 2 seconds");
    Serial.println("  wheel 10rpm        local wheel at +10 RPM");
    Serial.println("  w -5               local wheel at -5 RPM");
    Serial.println("  w0 10rpm           compatibility alias for local wheel");
    Serial.println("  all 10rpm          local alias; supervisor broadcasts this to all four units");
    Serial.println("  wstop              stop local wheel");
    Serial.println("  act diag           print RoboClaw voltage, error, encoder, speed, current");
    Serial.println("  act enc            print RoboClaw encoder counts and speeds");
    Serial.println("  act enc watch      watch RoboClaw encoder readback for 10 seconds");
    Serial.println("  act enc reset      reset RoboClaw encoder counters");
    Serial.println("  act reset          reset RoboClaw serial link and send stop commands");
    Serial.println("  act safety         set RoboClaw serial deadman timeout");
    Serial.println("  act scan           scan common RoboClaw serial pins, bauds, and addresses");
    Serial.println("  act use m1|m2      choose RoboClaw motor channel for actuator commands");
    Serial.println("  act pulse fwd      actuator forward pulse for 1 second at low power");
    Serial.println("  act pulse rev      actuator reverse pulse for 1 second at low power");
    Serial.println("  act pulse m1 fwd   pulse RoboClaw M1 forward for 1 second");
    Serial.println("  act pulse m2 fwd   pulse RoboClaw M2 forward for 1 second");
    Serial.println("  act fwd            actuator forward/extend until stopped/limit");
    Serial.println("  act rev            actuator reverse/retract until stopped/limit");
    Serial.println("  act voltage        read voltage limits/configuration; no changes");
    Serial.println("  act voltage apply  verify/apply 25.2 V max with zero PWM; no movement");
    Serial.println("  act voltage restore-min N  restore recorded minimum by one raw count; no movement");
    Serial.println("  act limits         read both raw current-limit fields; no changes");
    Serial.println("  act current        stop, set/verify 6.75 A ceiling; no move");
    Serial.println("  act current save   save verified current/configuration; RoboClaw resets; prior Stop required");
    Serial.println("  w0 current save    save verified 35/50 A configuration; ODrive resets; healthy IDLE required");
    Serial.println("  act power N        manual drive ceiling 1..60; change only while stopped");
    Serial.println("  act stop           stop actuator");
    Serial.println("  act center         run FWD/REV limits, then center actuator");
    Serial.println("  help               print this help\n");
}

float parseRpmToken(String token) {
    token.trim();
    token.toLowerCase();
    token.replace("rpm", "");
    return token.toFloat();
}

#include "actuator_persistence_runtime.h"

void handleWheelTuneSet(String args) {
    args.trim();
    const int sp = args.indexOf(' ');
    if (sp <= 0) {
        Serial.println("Usage: wheel tune set <param> <value>");
        return;
    }
    String param = args.substring(0, sp);
    String val_str = args.substring(sp + 1);
    param.trim();
    val_str.trim();
    if (param.length() == 0 || val_str.length() == 0) {
        Serial.println("Usage: wheel tune set <param> <value>");
        return;
    }
    char* end = nullptr;
    errno = 0;
    const float value = strtof(val_str.c_str(), &end);
    if (errno == ERANGE || end == val_str.c_str() || *end != '\0' || !isfinite(value)) {
        Serial.println("Usage: wheel tune set <param> <value> (finite numeric value required)");
        return;
    }
    if (ensureCanInitialized()) {
        setWheelTuningParam(0, param.c_str(), value);
    }
}

void handleCommand(String command) {
    command.trim();
    command.toLowerCase();
    ActuatorCommandScope actuator_scope(command);
    if (command.length() == 0) {
        return;
    }

    if (command == "telemetry on" || command == "telemetry off") {
        live_telemetry.enabled = (command == "telemetry on");
        Serial.printf("Live ODrive telemetry %s (200 ms).\n", live_telemetry.enabled ? "on" : "off");
        return;
    }
    if (command == "help" || command == "?") {
        printHelp();
        return;
    }
    if (command == "id") {
        Serial.printf("CLIMBER_UNIT_ID=%d LOCAL_ODRIVE_NODE_ID=%d\n", UNIT_ID, LOCAL_ODRIVE_NODE_ID);
        return;
    }
    if (command == "firmware") {
        Serial.printf("Firmware: V19 CAN Wheel Tuning & Ramp; unit=%d; revision=2026-11-19-v19-can-wheel-tuning\n", UNIT_ID);
        return;
    }
    if (command == "unit") {
        Serial.printf("unit id: %d, local ODrive CAN node id: %d\n", UNIT_ID, LOCAL_ODRIVE_NODE_ID);
        return;
    }
    if (command == "status") {
        printStatus();
        return;
    }
    if (command == "can init") {
        if (can_recovery.phase == can_link::INACTIVE) can_recovery.request(millis());
        const uint32_t began = millis();
        do {
            pumpCanOnce();
            if (stopRequestedFromSerial()) { stopLocalWheels(); break; }
            if (can_recovery.ready()) break;
            delay(2);
        } while (millis() - began < can_link::Recovery::ATTEMPT_MS);
        printCanStatus();
        return;
    }
    if (command == "can status") {
        printCanStatus();
        return;
    }
    if (command == "can reset") {
        resetCanController();
        return;
    }
    if (command == "can beacon") {
        if (ensureCanInitialized()) {
            sendCanAutobaudBeacon();
        }
        return;
    }
    if (command == "can watch") {
        watchCanFrames();
        return;
    }
    if (command == "limits") {
        watchLimitSwitches();
        return;
    }
    if (command == "start") {
        runStartupSequence();
        return;
    }
    if (command == "stop" || command == "s" || command == "estop") {
        operatorWheelStop();
        actuatorStop();
        Serial.println("Actuator stop requested.");
        return;
    }
    if (command == "wstop") {
        operatorWheelStop();
        return;
    }
    if (command == "w0 current" || command == "wheel current" || command == "w current") {
        if (ensureCanInitialized()) printWheelCurrentLimits(0);
        return;
    }
    if (command == "w0 current save" || command == "wheel current save" || command == "w current save") {
        if (ensureCanInitialized()) saveWheelCurrentLimits(0);
        return;
    }
    if (command == "w0 current apply" || command == "wheel current apply" || command == "w current apply") {
        if (ensureCanInitialized()) ensureWheelCurrentLimits(0);
        return;
    }
    if (command == "wheel tune" || command == "w tune" || command == "w0 tune") {
        if (ensureCanInitialized()) printWheelTuning(0);
        return;
    }
    if (command == "wheel tune apply" || command == "w tune apply" || command == "w0 tune apply") {
        if (ensureCanInitialized()) applyWheelTuningPreset(0);
        return;
    }
    if (command == "wheel tune save" || command == "w tune save" || command == "w0 tune save") {
        if (ensureCanInitialized()) saveWheelCurrentLimits(0);
        return;
    }
    if (command == "wheel tune set" || command == "w tune set" || command == "w0 tune set") {
        Serial.println("Usage: wheel tune set <param> <value>");
        return;
    }
    if (command.startsWith("wheel tune set ")) {
        handleWheelTuneSet(command.substring(15));
        return;
    }
    if (command.startsWith("w tune set ")) {
        handleWheelTuneSet(command.substring(11));
        return;
    }
    if (command.startsWith("w0 tune set ")) {
        handleWheelTuneSet(command.substring(12));
        return;
    }
    if (command.startsWith("wheel tune ") || command.startsWith("w tune ") || command.startsWith("w0 tune ")) {
        Serial.println("Usage: wheel tune | wheel tune apply | wheel tune set <param> <value> | wheel tune save");
        return;
    }
    if (command == "wheel cal" || command == "w cal" || command == "w0 cal") {
        if (ensureCanInitialized()) calibrateWheelEncoderOffset(0);
        return;
    }
    if (command.startsWith("wheel cal ") || command.startsWith("w cal ") || command.startsWith("w0 cal ")) {
        Serial.println("Usage: wheel cal");
        return;
    }
    if (command == "wheel diag" || command == "w diag" || command == "w0 diag") {
        printOdriveDiagnostics(0);
        return;
    }
    if (command == "wheel clear" || command == "w clear" || command == "w0 clear" ||
        command == "wheel recover" || command == "w recover" || command == "w0 recover") {
        clearLocalWheelErrors();
        return;
    }
    if (command == "wheel idle" || command == "w idle" || command == "w0 idle") {
        idleLocalWheels();
        return;
    }
    if (command == "wheel monitor" || command == "w monitor" || command == "w0 monitor") {
        monitorLocalWheel();
        return;
    }
    if (command == "wheel turn" || command == "w turn" || command == "w0 turn") {
        moveWheelRelativeTurns(1.0f);
        return;
    }
    if (command == "wheel test" || command == "w test" || command == "w0 test") {
        runWheelJogTest();
        return;
    }
    if (command == "tof init") {
        ensureTofInitialized();
        return;
    }
    if (command == "tof on") {
        tof_enabled = ensureTofInitialized();
        tof_missing_reported = false;
        Serial.println(tof_enabled ? "ToF enabled." : "ToF not enabled because no sensor was found.");
        return;
    }
    if (command == "tof off") {
        tof_enabled = false;
        Serial.println("ToF disabled.");
        return;
    }
    if (command == "tof read") {
        readTofOnce(true);
        return;
    }
    if (command == "act voltage") {
        printActuatorVoltageLimits();return;
    }
    if (command == "act voltage apply") {
        if (actuator_current_guard.failed() || actuator_telemetry.fault) {
            Serial.println("Actuator fault: press Stop before applying voltage settings.");return;
        }
        if (ensureActuatorVoltageCeiling(true) && refreshActuatorVoltageStatus()) {
            Serial.printf("Actuator maximum voltage verified: requested=25.2 V readback=%.1f V; minimum/offset preserved. No NVM save or movement.\n",
                actuator_voltage_guard.verifiedMaximum()/10.0f);
        }
        return;
    }
    if (command == "act voltage restore-min" || command.startsWith("act voltage restore-min ")) {
        const char* value = command.c_str() + 23;
        char* end = nullptr; errno = 0;
        long target = strtol(value, &end, 10);
        if (errno || end == value || *end != '\0' || target < 1 || target > 250) {
            Serial.println("Usage: act voltage restore-min N (raw tenths V, current minimum +1, <=250); no movement.");return;
        }
        if (actuator_current_guard.failed() || actuator_telemetry.fault) {
            Serial.println("Actuator fault: press Stop before restoring voltage settings.");return;
        }
        if (restoreActuatorVoltageMinimum(uint16_t(target)) && refreshActuatorVoltageStatus()) {
            Serial.printf("Actuator minimum voltage restored: raw=%u (%.1f V); maximum requested=25.2 V readback=%.1f V; offset/configuration preserved. No NVM save or movement.\n",
                unsigned(actuator_voltage_guard.verifiedMinimum()), actuator_voltage_guard.verifiedMinimum()/10.0f,
                actuator_voltage_guard.verifiedMaximum()/10.0f);
        }
        return;
    }
    if (command == "act limits") {
        printActuatorCurrentLimits();
        return;
    }
    if (command == "act current save") {
        saveActuatorCurrentLimits();
        return;
    }
    if (command == "act current") {
        actuatorStop();
        if (!actuator_telemetry.known_idle) {
            Serial.println("Current limit unchanged: stop not acknowledged."); return;
        }
        if (ensureActuatorCurrentCeiling(actuator_motor_channel)) {
            actuator_scope.touches = false; // Both zero commands acknowledged; resume idle polling.
            Serial.printf("Actuator M%u limit readback: %.2f A; target <=6.75 A.\n",
                actuator_motor_channel, actuator_telemetry.limit.a / 100.0f);
        }
        return;
    }
    if (command == "act power" || command.startsWith("act power ")) {
        if (command.length() > 9) {
            const char* value = command.c_str() + 10;
            char* end = nullptr; errno = 0;
            long target = strtol(value, &end, 10);
            if (errno || end == value || *end != '\0' || target < 1 || target > actuator_live::Telemetry::MAX_MANUAL_POWER) {
                Serial.println("Usage: act power N (1..60); does not move the motor."); return;
            }
            if (target > 40 && actuator_telemetry.known_idle && !actuator_telemetry.manual && !actuator_telemetry.fault) {
                if (actuator_current_guard.failed()) { Serial.println("Actuator fault: press Stop before changing power.");return; }
                if (!ensureActuatorVoltageCeiling(true) || !refreshActuatorVoltageStatus()) return;
            }
            if (const char* why = actuator_telemetry.setManualPower(int(target), millis())) {
                Serial.println(why); return;
            }
        }
        Serial.printf("Manual actuator power=%d/127 (%.1f%%); reboot default=40. No motion.\n",
            actuator_telemetry.manual_power, actuator_telemetry.manual_power * 100.0f / 127.0f);
        return;
    }
    if (command == "act stop") {
        actuatorStop();
        Serial.println(actuator_telemetry.known_idle ? "Actuator zero commands acknowledged." : "Actuator stop NOT confirmed; telemetry paused.");
        return;
    }
    if (command == "act diag") {
        printRoboClawDiagnostics();
        return;
    }
    if (command == "act enc" || command == "act encoder") {
        printActuatorEncoderSnapshot();
        return;
    }
    if (command == "act enc watch" || command == "act encoder watch") {
        watchActuatorEncoders();
        return;
    }
    if (command == "act enc reset" || command == "act encoder reset") {
        resetActuatorEncoders();
        return;
    }
    if (command == "act reset") {
        resetRoboClawSerialLink(true);
        return;
    }
    if (command == "act safety") {
        bool ok = setRoboClawSerialTimeout(ROBOCLAW_SERIAL_TIMEOUT_TENTHS);
        Serial.printf("RoboClaw serial deadman timeout: %s (%u ms).\n",
            ok ? "enabled" : "not acknowledged",
            (unsigned int)ROBOCLAW_SERIAL_TIMEOUT_TENTHS * 100);
        return;
    }
    if (command == "act scan") {
        scanRoboClaw();
        return;
    }
    if (command == "act use m1") {
        actuator_motor_channel = 1;
        Serial.println("Actuator commands now use RoboClaw M1.");
        return;
    }
    if (command == "act use m2") {
        actuator_motor_channel = 2;
        Serial.println("Actuator commands now use RoboClaw M2.");
        return;
    }
    if (command == "act pulse fwd") {
        actuatorPulse("FWD/extend", ACTUATOR_JOG_POWER, ACTUATOR_PULSE_MS);
        return;
    }
    if (command == "act pulse rev") {
        actuatorPulse("REV/retract", -ACTUATOR_JOG_POWER, ACTUATOR_PULSE_MS);
        return;
    }
    if (command == "act pulse m1 fwd") {
        actuatorPulseChannel("M1 FWD/extend test", 1, ACTUATOR_JOG_POWER, ACTUATOR_PULSE_MS);
        return;
    }
    if (command == "act pulse m1 rev") {
        actuatorPulseChannel("M1 REV/retract test", 1, -ACTUATOR_JOG_POWER, ACTUATOR_PULSE_MS);
        return;
    }
    if (command == "act pulse m2 fwd") {
        actuatorPulseChannel("M2 FWD/extend test", 2, ACTUATOR_JOG_POWER, ACTUATOR_PULSE_MS);
        return;
    }
    if (command == "act pulse m2 rev") {
        actuatorPulseChannel("M2 REV/retract test", 2, -ACTUATOR_JOG_POWER, ACTUATOR_PULSE_MS);
        return;
    }
    if (command == "act fwd") {
        actuatorMoveUntilLimit("manual FWD/extend", PIN_ACT_LIMIT_OUT, actuator_telemetry.manual_power, ACTUATOR_LIMIT_TIMEOUT_MS);
        return;
    }
    if (command == "act rev") {
        actuatorMoveUntilLimit("manual REV/retract", PIN_ACT_LIMIT_IN, -actuator_telemetry.manual_power, ACTUATOR_LIMIT_TIMEOUT_MS);
        return;
    }
    if (command == "act center") {
        runActuatorStartup();
        return;
    }

    int space_index = command.indexOf(' ');
    if (space_index > 0) {
        String lhs = command.substring(0, space_index);
        String rhs = command.substring(space_index + 1);
        float rpm = parseRpmToken(rhs);

        if (lhs == "wheel" || lhs == "w") {
            if (rhs.startsWith("monitor")) {
                String seconds_token = rhs.substring(7);
                seconds_token.trim();
                float seconds = seconds_token.length() > 0 ? seconds_token.toFloat() : ((float)ODRIVE_MONITOR_MS / 1000.0f);
                if (seconds <= 0.0f) {
                    seconds = (float)ODRIVE_MONITOR_MS / 1000.0f;
                }
                monitorLocalWheel((uint32_t)(seconds * 1000.0f));
                return;
            }
            if (rhs.startsWith("turn")) {
                String rest = rhs.substring(4);
                rest.trim();
                int rest_space = rest.indexOf(' ');
                String turns_token = rest_space >= 0 ? rest.substring(0, rest_space) : rest;
                String rpm_token = rest_space >= 0 ? rest.substring(rest_space + 1) : "";
                turns_token.trim();
                rpm_token.trim();
                float turns = turns_token.length() > 0 ? turns_token.toFloat() : 1.0f;
                float rpm = rpm_token.length() > 0 ? parseRpmToken(rpm_token) : ODRIVE_TURN_TEST_RPM;
                if (turns == 0.0f) {
                    turns = 1.0f;
                }
                if (rpm == 0.0f) {
                    rpm = ODRIVE_TURN_TEST_RPM;
                }
                moveWheelRelativeTurns(turns, rpm);
                return;
            }
            if (rhs.startsWith("test")) {
                String rpm_token = rhs.substring(4);
                float rpm = rpm_token.length() > 0 ? parseRpmToken(rpm_token) : ODRIVE_TEST_RPM;
                runWheelJogTest(rpm == 0.0f ? ODRIVE_TEST_RPM : rpm);
                return;
            }
            setLocalWheelRpm(rpm);
            return;
        }
        if (lhs == "wall" || lhs == "all") {
            setAllLocalWheelsRpm(rpm);
            Serial.println("Note: on one ESP32, all/wall means this local unit only.");
            return;
        }
        if (lhs.length() == 2 && lhs[0] == 'w' && isDigit(lhs[1])) {
            int index = lhs[1] - '0';
            setWheelRpm(index, rpm);
            return;
        }
    }

    Serial.println("Unknown command. Type help.");
}

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0); // Telemetry must never wait for the USB TX lock.
    for (int i = 0; i < 30 && !Serial; ++i) {
        delay(100);
    }
    delay(300);

    Serial.println("Climber MCU bring-up firmware");
    Serial.println("Live telemetry revision: 2026-11-19-v19-can-wheel-tuning (200 ms, wheel 35 A / trip 50 A)");
    Serial.printf("Unit %d, local ODrive CAN node %d\n", UNIT_ID, LOCAL_ODRIVE_NODE_ID);
    Serial.println("Serial command loop is starting before optional sensors. Type help for commands.");

    pinMode(PIN_ACT_LIMIT_IN, INPUT_PULLUP);
    pinMode(PIN_ACT_LIMIT_OUT, INPUT_PULLUP);

    beginRoboClawSerial(ROBO_BAUDRATE, PIN_ROBO_RX, PIN_ROBO_TX);
    roboclaw.clear();
    actuatorStop();

    for (int i = 0; i < NUM_LOCAL_ODRIVES; ++i) {
        odrives[i]->onStatus(onHeartbeat, &odrive_data[i]);
        odrives[i]->onFeedback(onFeedback, &odrive_data[i]);
    }

    can_recovery.request(millis());
    Serial.println("Ready. CAN connects to IDLE automatically; ToF remains on request.");
    printHelp();
    Serial.print("bringup> ");
}

void loop() {
    pumpCanOnce();

    if (Serial.available()) {
        input_line = Serial.readStringUntil('\n');
        handleCommand(input_line);
        Serial.print("bringup> ");
    }

    if (tof_enabled && millis() - last_tof_print_ms >= 500) {
        last_tof_print_ms = millis();
        bool any_tof_ok = false;
        for (uint8_t i = 0; i < NUM_TOF_SENSORS; ++i) {
            any_tof_ok = any_tof_ok || tof_sensor_ok[i];
        }
        if (any_tof_ok) {
            readTofOnce(false);
        } else if (!tof_missing_reported) {
            tof_missing_reported = true;
            Serial.println("ToF not found; type 'tof off' to disable continuous mode or check sensor wiring.");
        }
    }
}
