#pragma once
#include "actuator_checked_center.h"

struct CheckedCenterHardware {
    bool cancelled_latched = false;
    uint32_t now() { return millis(); }
    void pause() { delay(5); }
    bool cancelled() {
        if (benchCancelled()) cancelled_latched = true;
        return cancelled_latched;
    }
    bool fwd() { return digitalRead(PIN_ACT_LIMIT_OUT) == LOW; }
    bool rev() { return digitalRead(PIN_ACT_LIMIT_IN) == LOW; }
    bool stop() {
        // Match the legacy open-loop stop packets, with every ACK required.
        // BackwardM*(0) is also a zero-power command, not a reversal.
        const bool m1_forward = roboclaw.ForwardM1(roboclaw_addr, 0);
        const bool m1_backward = roboclaw.BackwardM1(roboclaw_addr, 0);
        const bool m2_forward = roboclaw.ForwardM2(roboclaw_addr, 0);
        const bool m2_backward = roboclaw.BackwardM2(roboclaw_addr, 0);
        if (m1_forward && m1_backward && m2_forward && m2_backward) return true;
        benchEvent("fault", "stop_not_acknowledged_cut_motor_power");
        delay(250); // Remain silent so telemetry cannot feed the watchdog.
        return false;
    }
    bool watchdog() { return setRoboClawSerialTimeout(ROBOCLAW_SERIAL_TIMEOUT_TENTHS); }
    bool drive(int power) {
        if (!power || abs(power) > actuator_checked_center::MAX_COMMAND_POWER || cancelled() || (power > 0 ? fwd() : rev())) return false;
        if (!ensureActuatorCurrentCeiling(1)) return false;
        return power > 0 ? roboclaw.ForwardM1(roboclaw_addr, (uint8_t)power)
                         : roboclaw.BackwardM1(roboclaw_addr, (uint8_t)-power);
    }
    actuator_bench::Sample sample() {
        actuator_bench::Sample s;
        const uint32_t began = millis();
        auto budget = [&]() { return millis() - began <= 60 && !cancelled(); };
        uint8_t encoder_status = 0;
        s.encoder = (int32_t)roboclaw.ReadEncM1(roboclaw_addr, &encoder_status, &s.encoder_ok);
        if (s.encoder_ok && budget()) s.current_ok = roboclaw.ReadCurrents(roboclaw_addr, s.current1, s.current2);
        if (s.current_ok && budget()) s.voltage10 = roboclaw.ReadMainBatteryVoltage(roboclaw_addr, &s.voltage_ok);
        if (s.voltage_ok && budget()) s.status = roboclaw.ReadError(roboclaw_addr, &s.status_ok);
        if (s.status_ok && budget()) s.pwm_ok = roboclaw.ReadPWMs(roboclaw_addr, s.pwm1, s.pwm2);
        s.forward_limit = fwd(); s.reverse_limit = rev();
        s.at_ms = millis(); s.read_ms = s.at_ms - began;
        return s;
    }
    void log(const char* phase, const actuator_bench::Sample& s, int power) { logBenchSample(phase, s, power); }
    void range(int32_t fwd_ticks, int32_t rev_ticks, int32_t target) {
        Serial.printf("CENTER_CHECK_RANGE,unit=%d,fwd=%ld,rev=%ld,target=%ld\n", UNIT_ID,
            (long)fwd_ticks, (long)rev_ticks, (long)target);
    }
};

void printCheckedCenterStatus() {
    Serial.printf("Checked center revision 2026-09-14-u0-limit-stop-fix2. unit=%d enabled=%s. No ToF reads or wheel commands.\n",
        UNIT_ID, UNIT_ID == 0 ? "yes" : "no");
    Serial.printf("One FWD limit, REV limit, encoder center pass. power<=%d, timeout=8s/phase, stroke=%ld..%ld ticks.\n",
        actuator_checked_center::MAX_COMMAND_POWER, (long)ACTUATOR_MIN_ENCODER_RANGE_TICKS, (long)ACTUATOR_MAX_ENCODER_RANGE_TICKS);
    Serial.printf("Coarse center: middle 40..60%% of measured stroke. Center power stays at %d after ramp; no 26/22 slowdown. Not a calibrated mm tolerance.\n",
        actuator_checked_center::MAX_COMMAND_POWER);
    Serial.println("Homing power=40 as in start; center power=50. Stop uses M1/M2 forward-zero and reverse-zero, with all acknowledgements required.");
    Serial.println("Unit 0 FWD must increase encoder ticks. Current trip=2A, bus=12..21.5V; experimental supply, not a verified motor rating.");
    Serial.printf("Stopped verification: PWM must be zero; total settling deadline=%lums including 250ms stable encoder and M1 current <=0.25A. No drive during settling.\n",
        (unsigned long)actuator_checked_center::STOP_VERIFY_TIMEOUT_MS);
    Serial.println("Controller faults, invalid/stale telemetry, limits, stall, USB disconnect or operator input stop the test. Watchdog=200ms. No fault exemption, retry, boost or timed fallback.");
}

void runCheckedCenter() {
    bench_armed = false;
    if (UNIT_ID != 0 || actuator_motor_channel != 1) {
        Serial.println("Checked center refused: unit 0/M1 only."); return;
    }
    if (odriveOnline(0) && odrive_data[0].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE) {
        Serial.println("Checked center refused: ODrive is not IDLE. Keep wheel power off."); return;
    }
    tof_enabled = false;
    // No I2C transactions here, including sensor cleanup. It cannot affect this loop.
    printCheckedCenterStatus();
    benchHeader();
    CheckedCenterHardware io;
    actuator_checked_center::Config config;
    config.min_stroke = ACTUATOR_MIN_ENCODER_RANGE_TICKS;
    config.max_stroke = ACTUATOR_MAX_ENCODER_RANGE_TICKS;
    actuator_checked_center::Run<CheckedCenterHardware> check(io, config);
    const auto result = check.run();
    Serial.printf("CENTER_CHECK_RESULT,unit=%d,complete=%d,reason=%s,motion_commanded=%d,stop_ack=%d,output_verified=%d,settle_fault=%s,fwd=%ld,rev=%ld,target=%ld,final=%ld,tolerance_ticks=%ld,final_valid=%d\n",
        UNIT_ID, result.complete, result.reason, result.motion_commanded, result.stop_ack,
        result.output_verified, result.settle_fault ? result.settle_fault : "none",
        (long)result.fwd, (long)result.rev, (long)result.target, (long)result.final_encoder, (long)result.tolerance_ticks,
        result.final_encoder_valid);
    if (!result.stop_ack || !result.output_verified) Serial.println("CUT MOTOR POWER: stopped output was not verified. No automatic retry.");
    else Serial.println("Checked center ended. No automatic retry or wheel test.");
}
