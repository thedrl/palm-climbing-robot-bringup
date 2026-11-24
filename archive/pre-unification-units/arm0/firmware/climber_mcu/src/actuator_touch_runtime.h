#pragma once
#include "actuator_touch.h"

struct TouchHardware {
    bool cancelled_latched = false;
    bool sensor_started = false;
    uint32_t now() { return millis(); }
    bool cancelled() {
        if (benchCancelled()) cancelled_latched = true;
        return cancelled_latched;
    }
    bool limit() {
        return digitalRead(PIN_ACT_LIMIT_OUT) == LOW;
    }
    bool reverseLimit() { return digitalRead(PIN_ACT_LIMIT_IN) == LOW; }
    void pause() { delay(10); }
    bool stop() { return benchStop(); }
    bool watchdog() { return setRoboClawSerialTimeout(ROBOCLAW_SERIAL_TIMEOUT_TENTHS); }
    bool drive(int power) {
        if (power < 1 || power > actuator_touch::MAX_POWER || cancelled() || limit()) return false;
        if (!ensureActuatorCurrentCeiling(1)) return false;
        return roboclaw.ForwardM1(roboclaw_addr, (uint8_t)power);
    }
    bool startSensor() {
        if (!ensureTofInitialized() || cancelled() || limit()) return false;
        Wire.setTimeOut(10);
        if (!selectTofPath()) return false;
        sensor_started = true;
        return tof_sensors[0].startRangeContinuous(50) &&
            tof_sensors[0].Status == VL53L0X_ERROR_NONE;
    }
    void endSensor() {
        if (sensor_started && selectTofPath()) tof_sensors[0].stopRangeContinuous();
        // Single-read commands must initialize a fresh single-shot session.
        tof_enabled = false; tof_initialized = false; tof_sensor_ok[0] = false;
        Wire.setTimeOut(50);
    }
    actuator_touch::Range range() {
        actuator_touch::Range r;
        if (!selectTofPath()) { r.state = actuator_touch::RangeState::error; return r; }
        bool ready = tof_sensors[0].isRangeComplete();
        // The library reports "complete" on I2C errors too; check Status first.
        if (tof_sensors[0].Status != VL53L0X_ERROR_NONE) {
            r.state = actuator_touch::RangeState::error; return r;
        }
        if (!ready) return r;
        r.mm = tof_sensors[0].readRangeResult();
        r.status = tof_sensors[0].readRangeStatus();
        r.at_ms = millis();
        r.state = tof_sensors[0].Status == VL53L0X_ERROR_NONE ?
            actuator_touch::RangeState::ready : actuator_touch::RangeState::error;
        return r;
    }
    actuator_bench::Sample sample() {
        actuator_bench::Sample s;
        uint32_t began = millis();
        auto budget = [&]() { return millis() - began <= 60 && !cancelled() && !limit(); };
        uint8_t status = 0;
        s.encoder = (int32_t)roboclaw.ReadEncM1(roboclaw_addr, &status, &s.encoder_ok);
        if (s.encoder_ok && budget()) s.current_ok = roboclaw.ReadCurrents(roboclaw_addr, s.current1, s.current2);
        if (s.current_ok && budget()) s.voltage10 = roboclaw.ReadMainBatteryVoltage(roboclaw_addr, &s.voltage_ok);
        if (s.voltage_ok && budget()) s.status = roboclaw.ReadError(roboclaw_addr, &s.status_ok);
        if (s.status_ok && budget()) s.pwm_ok = roboclaw.ReadPWMs(roboclaw_addr, s.pwm1, s.pwm2);
        s.forward_limit = digitalRead(PIN_ACT_LIMIT_OUT) == LOW;
        s.reverse_limit = digitalRead(PIN_ACT_LIMIT_IN) == LOW;
        s.at_ms = millis(); s.read_ms = s.at_ms - began;
        return s;
    }
    void log(const char* phase, const actuator_touch::Range& r,
             const actuator_bench::Sample& s, int power) {
        Serial.printf("TOUCH_DATA,%d,%s,%lu,%u,%ld,%.2f,%.2f,%.1f,%d,%d,%lu,%lu,%d,%lu,%lu,%d,%d,%d,%d,%d,%d,%d\n",
            UNIT_ID, phase, (unsigned long)millis(), r.mm, (long)s.encoder,
            s.current1 / 100.0f, s.current2 / 100.0f, s.voltage10 / 10.0f,
            s.pwm1, s.pwm2, (unsigned long)s.status, (unsigned long)s.read_ms, power,
            (unsigned long)s.at_ms, (unsigned long)r.at_ms,
            s.encoder_ok, s.current_ok, s.voltage_ok, s.status_ok, s.pwm_ok,
            s.reverse_limit, s.forward_limit);
    }
    void event(const char* reason) { Serial.printf("TOUCH_STOP,%d,%s\n", UNIT_ID, reason); }
};

void printTouchStatus() {
    Serial.printf("Touch revision 2026-09-13-u0-253-settings1. unit=%d enabled=%s target=%umm max_power=%d max_time=%lus max_start=%umm.\n",
        UNIT_ID, UNIT_ID == 0 ? "yes" : "no", actuator_touch::TARGET_MM,
        actuator_touch::MAX_POWER, (unsigned long)(actuator_touch::TIMEOUT_MS / 1000), actuator_touch::MAX_START_MM);
    const auto bounds = actuator_touch::envelope();
    Serial.printf("Probe trips: abs M1 current >=%.1fA; bus outside %.1f..%.1fV. Test bounds, not verified force or mm calibration.\n",
        bounds.trip_a, bounds.min_v, bounds.max_v);
    Serial.println("Fixed encoder travel cutoff: none. ToF is the normal stopping target; physical limits, timeout, sensor/controller faults, encoder direction/stall checks and operator stop remain active.");
    if (UNIT_ID == 0) Serial.println("RoboClaw RX synchronization: enabled before each packet/retry. CRC and controller-fault checks retained.");
    Serial.println("Operator-requested experimental 15V supply; the recorded motor rating is nominally 12V. 2A is a provisional test trip, not a continuous-current or contact-force rating. No current-offset compensation or startup exemption.");
    Serial.printf("Forward-only limits: FWD blocks motion; initial REV is allowed but must release within %lu ms of first drive. REV reassertion stops the test.\n",
        (unsigned long)actuator_touch::REVERSE_RELEASE_MS);
    Serial.println("Stops at first valid ToF <=253mm. Operator-selected contact reference for the measured cylinder setup only, not proof of contact or force. No automatic retry, reversal, compression, or wheel test.");
    Serial.println("'act touch' retains the 2A M1 trip. Explicit 'act touch test' monitors M1 current without that trip during ONE approach only; baseline, stopped-output, unused-M2 and controller-fault checks remain active. No setting is saved.");
    Serial.println("A 2A supply limit does not guarantee a 2A PWM motor-current limit. RoboClaw hardware protection settings are not changed or verified by this command.");
    Serial.printf("Stop verification: stop sent immediately; after ACK, %lums UART quiet, then zero PWM, idle current and stable encoder for >=%lums (at least 5 samples). Verification budget %lums. No retry or reverse correction.\n",
        (unsigned long)actuator_touch::STOP_QUIET_MS, (unsigned long)actuator_touch::STOP_STABLE_MS,
        (unsigned long)actuator_touch::STOP_VERIFY_TIMEOUT_MS);
    Serial.println("'act touch stopcheck' sends zero commands only and checks stopped output; it never commands forward, reverse, or wheel motion.");
}

void runActuatorTouch(actuator_touch::Mode mode = actuator_touch::Mode::protected_approach) {
    bench_armed = false;
    if (UNIT_ID != 0 || actuator_motor_channel != 1) {
        Serial.println("Touch refused: this candidate is for unit 0 / M1 only."); return;
    }
    // Do not arm, initialize, or change the ODrive control mode here.
    if (odriveOnline(0) && odrive_data[0].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE) {
        Serial.println("Touch refused: ODrive is not IDLE. Keep wheel motor power off for this bench test."); return;
    }
    tof_enabled = false;
    bench_log_start_ms = millis();
    printTouchStatus();
    const char* mode_name = mode == actuator_touch::Mode::current_monitor_test ? "current_monitor_test" :
        (mode == actuator_touch::Mode::stop_check_only ? "stop_check_only" : "protected_approach");
    Serial.printf("TOUCH_MODE,unit=%d,mode=%s,m1_approach_trip=%s\n", UNIT_ID, mode_name,
        mode == actuator_touch::Mode::current_monitor_test ? "disabled_for_this_run" : "2A");
    Serial.println("TOUCH_HEADER,unit,phase,t_ms,tof_mm,encoder,m1_a,m2_a,bus_v,pwm_m1,pwm_m2,status,read_ms,command_power,sample_ms,tof_sample_ms,encoder_valid,current_valid,voltage_valid,status_valid,pwm_valid,rev_pressed,fwd_pressed");
    TouchHardware io;
    auto result = actuator_touch::run(io, mode);
    Serial.printf("TOUCH_RESULT,unit=%d,reason=%s,stop_ack=%d,motion_commanded=%d,threshold_seen=%d,last_tof_mm=%u,tof_age_ms=%lu,encoder_delta=%lld,output_verified=%d,settle_fault=%s,mode=%s\n",
        UNIT_ID, result.reason, result.stopped, result.moved_commanded,
        result.threshold_seen, result.last_mm,
        (unsigned long)(result.last_mm ? millis() - result.last_range_at_ms : UINT32_MAX),
        (long long)result.encoder_delta,
        result.output_verified, result.settle_fault ? result.settle_fault : "none", mode_name);
    if (!result.stopped) Serial.println("CUT MOTOR POWER: stopped output was not acknowledged. Do not retry or query the controller.");
    else if (!result.output_verified) Serial.println("CUT MOTOR POWER: zero output/settling was not verified. Review the fault samples before another test.");
    else Serial.println("Touch command ended. No automatic restart. ToF streaming off; 'tof read' takes a new stopped reading.");
}
