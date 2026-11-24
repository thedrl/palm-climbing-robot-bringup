#pragma once
// Included after the bring-up globals and CAN callbacks; no motor arming here.
can_link::Recovery can_recovery;
struct CanFaultSnapshot {
    bool valid = false;
    uint32_t at = 0, hb_age = 0, axis_error = 0;
    uint8_t axis_state = 0;
    twai_status_info_t counters = {};
    bool counters_valid = false;
    float bus_v = NAN;
    int32_t bus_age = -1;
} can_fault;
const char* can_notice = nullptr;

can_link::Status readCanLinkStatus() {
    can_link::Status result;
    if (!can_initialized) return result;
    twai_status_info_t s = {};
    if (twai_get_status_info(&s) != ESP_OK) { result.driver = can_link::UNKNOWN; return result; }
    switch (s.state) {
        case TWAI_STATE_RUNNING: result.driver = can_link::RUNNING; break;
        case TWAI_STATE_STOPPED: result.driver = can_link::STOPPED; break;
        case TWAI_STATE_BUS_OFF: result.driver = can_link::BUS_OFF; break;
        case TWAI_STATE_RECOVERING: result.driver = can_link::RECOVERING; break;
    }
    result.tx_errors = s.tx_error_counter; result.rx_errors = s.rx_error_counter;
    result.tx_free = s.msgs_to_tx == 0;
    const auto& data = odrive_data[0];
    result.heartbeat_fresh = data.received_heartbeat && uint32_t(millis() - data.last_heartbeat_ms) < 1000;
    result.axis_state = data.last_heartbeat.Axis_State;
    result.heartbeat_sequence = data.heartbeat_sequence;
    return result;
}

bool quiesceCanLink() {
    live_telemetry.reset();
    for (auto& data : odrive_data) {
        data.received_heartbeat = false; data.received_feedback = false;
        data.last_heartbeat_ms = 0;
    }
    if (!can_initialized) return true;
    twai_status_info_t s = {};
    if (twai_get_status_info(&s) != ESP_OK) return false;
    if (s.state == TWAI_STATE_RUNNING && twai_stop() != ESP_OK) return false;
    // STOPPED and BUS_OFF may be uninstalled; never discard a failed result.
    if (twai_driver_uninstall() != ESP_OK) return false;
    can_initialized = false;
    return true;
}

struct CanRecoveryIO {
    void captureFault(const can_link::Status&) const {
        can_fault.valid = true; can_fault.at = millis();
        can_fault.counters_valid = twai_get_status_info(&can_fault.counters) == ESP_OK;
        can_fault.hb_age = millis() - odrive_data[0].last_heartbeat_ms;
        can_fault.axis_state = odrive_data[0].last_heartbeat.Axis_State;
        can_fault.axis_error = odrive_data[0].last_heartbeat.Axis_Error;
        can_fault.bus_v = live_telemetry.vi.seen ? live_telemetry.vi.a : NAN;
        can_fault.bus_age = live_telemetry.vi.age(millis());
    }
    void quiesce() const { quiesceCanLink(); }
    bool restartLink() const {
        if (!quiesceCanLink()) return false;
        if (setupCan()) { can_initialized = true; return true; }
        // A failed start may leave an installed driver if cleanup also failed.
        twai_status_info_t leftover = {};
        can_initialized = twai_get_status_info(&leftover) == ESP_OK;
        return false;
    }
    bool sendBeacon() const {
        const uint8_t payload[1] = {0};
        return sendMsg(can_intf, 0x7df, 1, payload);
    }
    bool sendZero() const { return odrives[0]->setVelocity(0.0f, 0.0f); }
    bool sendIdle() const { return odrives[0]->setState(ODriveAxisState::AXIS_STATE_IDLE); }
    void report(can_link::Event event) const {
        switch (event) {
            case can_link::LINK_LOST: can_notice = "\nCAN lost; wheel blocked. See can status.\n"; break;
            case can_link::ATTEMPT_FAILED: can_notice = "\nCAN not ready; retry later. Wheel blocked.\n"; break;
            case can_link::CONNECTED: can_notice = "\nCAN ready; ODrive IDLE confirmed.\n"; break;
            case can_link::RELEASE_REQUIRED: can_notice = "\nCAN restored IDLE. Press Stop before a new move.\n"; break;
        }
    }
} can_recovery_io;

void serviceCanRecovery() {
    can_recovery.tick(millis(), readCanLinkStatus(), can_recovery_io);
    live_telemetry.ready = can_recovery.ready();
    // One short notice, bounded by the actual 64-byte USB FIFO. Never wait.
    if (can_notice && Serial.available() == 0) {
        const size_t len = strlen(can_notice);
        if (len <= 64 && Serial.availableForWrite() >= int(len) &&
            Serial.write(reinterpret_cast<const uint8_t*>(can_notice), len) == len) can_notice = nullptr;
    }
}

bool wheelMotionAllowed() {
    const auto s = readCanLinkStatus();
    return can_recovery.motionAllowed() && s.busUsable() && s.heartbeat_fresh;
}

void printCanRecoveryStatus() {
    static const char* names[] = {"INACTIVE", "BACKOFF", "CONNECTING", "ZERO", "IDLE", "CONFIRM_IDLE", "READY"};
    Serial.printf("recovery=%s attempts=%lu faults=%lu stop_required=%s\n",
        names[can_recovery.phase], (unsigned long)can_recovery.attempts,
        (unsigned long)can_recovery.faults, can_recovery.needs_stop ? "yes" : "no");
    if (!can_fault.valid) return;
    Serial.printf("last link fault: %lu ms ago; HB age=%lu ms state=%u axis_error=0x%08lx\n",
        (unsigned long)(millis() - can_fault.at), (unsigned long)can_fault.hb_age,
        can_fault.axis_state, (unsigned long)can_fault.axis_error);
    Serial.printf("last bus sample before fault: %.2f V sample_age=%ld ms (cached)\n",
        can_fault.bus_v, (long)can_fault.bus_age);
    if (can_fault.counters_valid) {
        const auto& s = can_fault.counters;
        Serial.printf("fault TWAI state=%d TEC=%lu REC=%lu tx_queue=%lu bus_errors=%lu rx_missed=%lu\n",
            int(s.state), (unsigned long)s.tx_error_counter, (unsigned long)s.rx_error_counter,
            (unsigned long)s.msgs_to_tx, (unsigned long)s.bus_error_count, (unsigned long)s.rx_missed_count);
    }
}
