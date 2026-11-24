#pragma once
#include "actuator_bench_guard.h"

static actuator_bench::Config bench_config;
static bool bench_armed = false;
static uint32_t bench_armed_ms = 0;
static uint32_t bench_log_start_ms = 0;

bool benchCancelled() {
    if (!Serial) return true;
    bool cancel = false;
    // Any incoming command aborts this test; never block waiting for a newline.
    while (Serial.available()) { char c = Serial.read(); if (!isspace((unsigned char)c)) cancel = true; }
    return cancel;
}

actuator_bench::Sample readBenchSample() {
    actuator_bench::Sample s;
    uint32_t began = millis();
    uint8_t encoder_status = 0;
    s.encoder = (int32_t)roboclaw.ReadEncM1(roboclaw_addr, &encoder_status, &s.encoder_ok);
    s.current_ok = roboclaw.ReadCurrents(roboclaw_addr, s.current1, s.current2);
    s.voltage10 = roboclaw.ReadMainBatteryVoltage(roboclaw_addr, &s.voltage_ok);
    s.status = roboclaw.ReadError(roboclaw_addr, &s.status_ok);
    s.pwm_ok = roboclaw.ReadPWMs(roboclaw_addr, s.pwm1, s.pwm2);
    s.forward_limit = digitalRead(PIN_ACT_LIMIT_OUT) == LOW;
    s.reverse_limit = digitalRead(PIN_ACT_LIMIT_IN) == LOW;
    s.at_ms = millis(); s.read_ms = s.at_ms - began;
    return s;
}

void benchHeader() {
    bench_log_start_ms = millis();
    Serial.println("BENCH_HEADER,unit,phase,t_ms,encoder,encoder_valid,m1_a,m2_a,current_valid,bus_v,voltage_valid,pwm_m1,pwm_m2,pwm_valid,status,status_valid,fwd_limit,rev_limit,read_ms,command_power");
}

void logBenchSample(const char* phase, const actuator_bench::Sample& s, int power) {
    Serial.printf("BENCH_DATA,%d,%s,%lu,%ld,%d,%.2f,%.2f,%d,%.1f,%d,%d,%d,%d,%lu,%d,%d,%d,%lu,%d\n",
        UNIT_ID, phase, (unsigned long)(s.at_ms - bench_log_start_ms), (long)s.encoder, s.encoder_ok,
        s.current1 / 100.0f, s.current2 / 100.0f, s.current_ok, s.voltage10 / 10.0f, s.voltage_ok,
        s.pwm1, s.pwm2, s.pwm_ok, (unsigned long)s.status, s.status_ok,
        s.forward_limit, s.reverse_limit, (unsigned long)s.read_ms, power);
}

void benchEvent(const char* phase, const char* reason) {
    Serial.printf("BENCH_EVENT,%d,%s,%lu,%s\n", UNIT_ID, phase,
        (unsigned long)(millis() - bench_log_start_ms), reason);
}

bool benchStop() {
    bool m1 = roboclaw.ForwardM1(roboclaw_addr, 0);
    bool m2 = roboclaw.ForwardM2(roboclaw_addr, 0);
    if (m1 && m2) return true;
    benchEvent("fault", "stop_not_acknowledged_cut_motor_power");
    // Telemetry packets may feed the controller's serial watchdog: remain silent here.
    delay(250);
    return false;
}

bool benchWaitSample(uint32_t started) {
    while (millis() - started < actuator_bench::SAMPLE_PERIOD_MS) {
        if (benchCancelled()) return false;
        delay(1);
    }
    return !benchCancelled();
}

void runBenchWatch(unsigned seconds) {
    bench_armed = false;
    tof_enabled = false;
    benchHeader();
    benchEvent("watch", "read_only_no_motion");
    uint32_t started = millis();
    while (millis() - started < seconds * 1000UL && !benchCancelled()) {
        uint32_t cycle = millis();
        auto s = readBenchSample();
        logBenchSample("watch", s, 0);
        if (!s.encoder_ok || !s.current_ok || !s.voltage_ok || !s.status_ok || !s.pwm_ok ||
            s.read_ms > actuator_bench::SAMPLE_BUDGET_MS) {
            benchEvent("fault", "invalid_or_slow_telemetry");
            benchEvent("watch", "complete");
            return;
        }
        if (!benchWaitSample(cycle)) break;
    }
    benchEvent("watch", "complete");
}

void runBenchStep(double delta_mm) {
    bool permitted = bench_armed && millis() - bench_armed_ms < actuator_bench::ARM_LIFETIME_MS;
    bench_armed = false; // One command consumes the authorization, including failed tests.
    tof_enabled = false;
    benchHeader();
    if (!permitted || actuator_motor_channel != 1 || (UNIT_ID != 0 && UNIT_ID != 2)) {
        benchEvent("refused", "fresh_arm_configuration_required_for_unit0_or2_m1"); return;
    }
    if (!isfinite(delta_mm) || delta_mm == 0 || fabs(delta_mm) > actuator_bench::MAX_STEP_MM) {
        benchEvent("refused", "step_must_be_nonzero_and_at_most_2_5mm"); return;
    }
    if (!benchStop() || !setRoboClawSerialTimeout(ROBOCLAW_SERIAL_TIMEOUT_TENTHS)) {
        benchEvent("refused", "stop_or_deadman_not_acknowledged"); return;
    }
    uint32_t hardware_limit = 0;
    if (!roboclaw.ReadM1MaxCurrent(roboclaw_addr, hardware_limit) || hardware_limit == 0 ||
        bench_config.trip_a * 100.0 > hardware_limit) {
        benchEvent("refused", "current_limit_unavailable_or_trip_exceeds_controller_limit"); return;
    }
    // ToF is sampled only while stopped so a blocking range read cannot starve current sampling.
    benchEvent("tof_before", "stationary_sensor_snapshot_not_a_contact_decision");
    readTofOnce();
    actuator_bench::Sample previous, sample;
    int32_t baseline_encoder = 0;
    uint32_t baseline_started = millis();
    bool have_previous = false;
    while (millis() - baseline_started < 1000) {
        if (benchCancelled()) { benchEvent("refused", "operator_or_usb_stop"); return; }
        uint32_t cycle = millis();
        sample = readBenchSample(); logBenchSample("baseline", sample, 0);
        const char* fault = actuator_bench::healthy(sample, bench_config);
        if (!fault && (sample.pwm1 || abs((int)sample.current1) > 25)) fault = "m1_not_idle_or_offset_suspect";
        if (!fault && have_previous && (llabs((int64_t)sample.encoder - previous.encoder) > 1 ||
            llabs((int64_t)sample.encoder - baseline_encoder) > 1)) fault = "baseline_encoder_motion";
        if (fault) { benchEvent("refused", fault); return; }
        if (!have_previous) baseline_encoder = sample.encoder;
        previous = sample; have_previous = true;
        if (!benchWaitSample(cycle)) { benchEvent("refused", "operator_or_usb_stop"); return; }
    }
    actuator_bench::Guard guard;
    const char* reason = guard.begin(bench_config, delta_mm, sample, millis());
    if (reason) { benchEvent("refused", reason); return; }
    Serial.printf("BENCH_TARGET,%d,delta_mm=%.6f,encoder_start=%ld,encoder_target=%ld,ticks_per_mm=%.6f\n",
        UNIT_ID, delta_mm, (long)guard.start, (long)guard.target, bench_config.ticks_per_mm);
    int applied = 0;
    uint32_t last_command = millis();
    uint32_t ramp_at = last_command;
    while (!reason) {
        uint32_t cycle = millis();
        sample = readBenchSample(); logBenchSample("move", sample, applied);
        reason = guard.evaluate(sample, millis(), benchCancelled(), millis() - last_command);
        if (reason) break;
        int magnitude = abs(applied);
        if (magnitude == 0 || millis() - ramp_at >= 100) {
            magnitude = min(magnitude + 2, bench_config.power); ramp_at = millis();
        }
        applied = guard.direction * magnitude;
        if (!actuatorDriveOpenLoop(applied)) { reason = "drive_acknowledgement_failed"; break; }
        last_command = millis();
        if (!benchWaitSample(cycle)) { reason = "operator_or_usb_stop"; break; }
    }
    bool stopped = benchStop();
    benchEvent("stop", reason ? reason : "unknown_stop");
    if (!stopped) return;
    actuator_bench::Sample first_settled, last_settled;
    bool have_settled = false, settled_healthy = true;
    uint32_t stopped_at = millis();
    while (millis() - stopped_at < 700) {
        uint32_t cycle = millis();
        sample = readBenchSample(); logBenchSample("settle", sample, 0);
        const char* fault = actuator_bench::healthy(sample, bench_config);
        if (fault || sample.pwm1) {
            benchEvent("fault", fault ? fault : "pwm_nonzero_after_stop");
            benchStop(); settled_healthy = false; break;
        }
        if (millis() - stopped_at >= 400) {
            if (!have_settled) { first_settled = sample; have_settled = true; }
            last_settled = sample;
        }
        if (!benchWaitSample(cycle)) { settled_healthy = false; break; }
    }
    bool success = reason && !strcmp(reason, "target_reached") && settled_healthy && have_settled &&
        guard.settledAtTarget(first_settled, last_settled);
    if (sample.encoder_ok) Serial.printf("BENCH_DISPLACEMENT,%d,encoder_based_mm=%.6f\n", UNIT_ID,
        ((int64_t)sample.encoder - guard.start) * bench_config.forward_sign / bench_config.ticks_per_mm);
    benchEvent(success ? "complete" : "failed", success ? "target_settled_within_one_tick" : "no_automatic_retry_or_reversal");
    if (settled_healthy && Serial) { benchEvent("tof_after", "stationary_sensor_snapshot"); readTofOnce(); }
}

void handleBenchCommand(String command) {
    if (command == "act bench status") {
        Serial.printf("Bench revision 2026-09-13. single-use arm=%s; max step=2.5 mm; max motion=2 s; max power=20. No automatic contact/force control.\n",
            bench_armed && millis() - bench_armed_ms < actuator_bench::ARM_LIFETIME_MS ? "configured" : "locked");
        return;
    }
    if (command.startsWith("act bench arm ")) {
        bench_armed = false;
        actuator_bench::Config candidate;
        if (!actuator_bench::parseConfig(command.substring(14).c_str(), &candidate)) {
            Serial.println("Bench arm rejected. Supply verified: ticks_per_mm fwd_sign trip_A min_V max_V power(1..20)."); return;
        }
        bench_config = candidate; bench_armed_ms = millis(); bench_armed = true;
        Serial.println("Bench configured for one step within 30 s. User-supplied calibration/ratings are not independently verified. No settings written to RoboClaw.");
        return;
    }
    if (command.startsWith("act bench watch ")) {
        double seconds;
        if (actuator_bench::numbers(command.substring(16).c_str(), &seconds, 1) && seconds >= 1 && seconds <= 30 && floor(seconds) == seconds) {
            runBenchWatch((unsigned)seconds); return;
        }
    }
    if (command.startsWith("act bench step ")) {
        double mm;
        if (actuator_bench::numbers(command.substring(15).c_str(), &mm, 1)) { runBenchStep(mm); return; }
    }
    bench_armed = false;
    Serial.println("Bench command rejected. Use act bench status, watch SECONDS, arm CALIBRATION, or step SIGNED_MM.");
}
