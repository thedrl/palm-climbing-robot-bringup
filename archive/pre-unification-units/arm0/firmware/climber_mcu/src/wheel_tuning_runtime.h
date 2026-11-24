#pragma once
// Include after wheel_persistence_runtime.h. Declare wheel_tuning::ByteSdo
// wheel_tuning_byte_sdo; and feed wheel_tuning_byte_sdo.observe(...) from
// onCanMessage alongside wheel_current_sdo and wheel_persistence_bool_sdo.

static bool resolveWheelTuningMap(int index, WheelCurrentSdoIO& io,
        Get_Version_msg_t& v, uint32_t& commit, wheel_tuning::Map& map) {
    if (io.cancelled() || !io.drain() || io.cancelled() ||
        !odrives[index]->getVersion(v, 100) || io.cancelled()) {
        Serial.println("Wheel tuning blocked: ODrive version/link not verified.");
        return false;
    }
    if (v.Protocol_Version != 2 || v.Hw_Version_Major != 5 || v.Hw_Version_Minor != 2 ||
        v.Hw_Version_Variant != 0 || v.Fw_Version_Major != 0 || v.Fw_Version_Minor != 6 ||
        (v.Fw_Version_Revision != 11 && v.Fw_Version_Revision != 12) || v.Fw_Version_Unreleased) {
        Serial.println("Wheel tuning blocked: unsupported ODrive firmware/hardware.");
        return false;
    }
    if (!wheel_current_sdo.readU32(io, LOCAL_ODRIVE_NODE_ID, 13, commit) ||
        !wheel_tuning::supported(v.Protocol_Version, v.Hw_Version_Major, v.Hw_Version_Minor,
            v.Hw_Version_Variant, v.Fw_Version_Major, v.Fw_Version_Minor,
            v.Fw_Version_Revision, v.Fw_Version_Unreleased, commit, map)) {
        Serial.printf("Wheel tuning blocked: unsupported/missing firmware commit 0x%08lX.\n",
            (unsigned long)commit);
        return false;
    }
    return true;
}

static bool readTuningFloatAllowInf(WheelCurrentSdoIO& io, uint16_t endpoint, float& out) {
    uint32_t bits = 0;
    if (!wheel_current_sdo.readU32(io, LOCAL_ODRIVE_NODE_ID, endpoint, bits)) {
        return false;
    }
    memcpy(&out, &bits, 4);
    return !isnan(out);
}

static bool writeAndVerifyTuningFloat(int index, WheelCurrentSdoIO& io,
        const char* name, uint16_t endpoint, float target, float& readback_out) {
    if (io.cancelled()) {
        Serial.printf("Wheel tuning cancelled before writing %s.\n", name);
        return false;
    }
    if (!wheel_current_sdo.writeFloat(io, LOCAL_ODRIVE_NODE_ID, endpoint, target)) {
        Serial.printf("Wheel tuning write failed: %s (ep %u).\n", name, unsigned(endpoint));
        return false;
    }
    float actual = NAN;
    if (!wheel_current_sdo.readFloat(io, LOCAL_ODRIVE_NODE_ID, endpoint, actual) ||
        !wheel_tuning::closeEnough(actual, target)) {
        Serial.printf("Wheel tuning readback mismatch: %s requested=%.4f actual=%.4f.\n",
            name, target, actual);
        return false;
    }
    if (io.cancelled() || !odriveOnline(index) ||
        odrive_data[index].last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_IDLE ||
        odrive_data[index].last_heartbeat.Axis_Error != 0) {
        Serial.printf("Wheel tuning blocked after %s: healthy IDLE lost.\n", name);
        return false;
    }
    readback_out = actual;
    return true;
}

bool printWheelTuning(int index) {
    if (index < 0 || index >= NUM_LOCAL_ODRIVES) return false;
    WheelCurrentSdoIO io{index};
    if (io.cancelled()) {
        Serial.println("Wheel tuning unavailable: link/stop not ready.");
        return false;
    }
    Get_Version_msg_t v;
    uint32_t commit = 0;
    wheel_tuning::Map map;
    if (!resolveWheelTuningMap(index, io, v, commit, map)) {
        return false;
    }
    wheel_current::Map cmap;
    if (!wheel_current::supported(v.Protocol_Version, v.Hw_Version_Major, v.Hw_Version_Minor,
            v.Hw_Version_Variant, v.Fw_Version_Major, v.Fw_Version_Minor,
            v.Fw_Version_Revision, v.Fw_Version_Unreleased, commit, cmap)) {
        Serial.println("Wheel tuning unavailable: current limit map unsupported.");
        return false;
    }

    float soft = NAN, hard = NAN;
    float vel_gain = NAN, vel_ig = NAN, vel_il = NAN, vel_tol = NAN, vel_ramp = NAN;
    float spin_elec = NAN, spin_mech = NAN;
    float dc_uv = NAN, dc_ov = NAN, dc_pos = NAN, dc_neg = NAN;
    struct FloatRead {
        uint16_t ep;
        float* dst;
    } reads[] = {
        {cmap.soft, &soft},
        {cmap.hard, &hard},
        {map.vel_gain, &vel_gain},
        {map.vel_integrator_gain, &vel_ig},
        {map.vel_integrator_limit, &vel_il},
        {map.vel_limit_tolerance, &vel_tol},
        {map.vel_ramp_rate, &vel_ramp},
        {map.spinout_elec_thresh, &spin_elec},
        {map.spinout_mech_thresh, &spin_mech},
        {map.dc_undervoltage, &dc_uv},
        {map.dc_overvoltage, &dc_ov},
        {map.dc_max_pos_current, &dc_pos},
        {map.dc_max_neg_current, &dc_neg},
    };
    for (const auto& r : reads) {
        if (!readTuningFloatAllowInf(io, r.ep, *r.dst)) {
            Serial.printf("Wheel tuning unavailable: endpoint %u read failed.\n", unsigned(r.ep));
            return false;
        }
    }

    uint8_t brake_en = 255, offset_valid = 255, rs485_status = 255;
    if (!wheel_persistence_bool_sdo.read(io, LOCAL_ODRIVE_NODE_ID, map.brake_resistor_enable, brake_en) ||
        !wheel_persistence_bool_sdo.read(io, LOCAL_ODRIVE_NODE_ID, map.comm_offset_valid, offset_valid) ||
        !wheel_tuning_byte_sdo.read(io, LOCAL_ODRIVE_NODE_ID, map.rs485_status, rs485_status)) {
        Serial.println("Wheel tuning unavailable: encoder/brake status read failed.");
        return false;
    }

    Serial.println("\n--- ODrive tuning & bus config ---");
    Serial.printf("fw=0.6.%u commit=0x%08lX current=%.2f/%.2f A (target %.0f/%.0f A)\n",
        unsigned(v.Fw_Version_Revision), (unsigned long)commit, soft, hard,
        wheel_current::SOFT_AMPS, wheel_current::HARD_AMPS);
    Serial.printf("vel_gain=%.4f vel_integrator_gain=%.4f vel_integrator_limit=%.4f\n",
        vel_gain, vel_ig, vel_il);
    Serial.printf("vel_limit_tolerance=%.4f vel_ramp_rate=%.4f\n", vel_tol, vel_ramp);
    Serial.printf("spinout_elec=%.2f W spinout_mech=%.2f W\n", spin_elec, spin_mech);
    Serial.printf("dc_undervoltage=%.2f V dc_overvoltage=%.2f V dc_max_pos=%.2f A dc_max_neg=%.2f A brake_resistor=%u\n",
        dc_uv, dc_ov, dc_pos, dc_neg, unsigned(brake_en));
    Serial.printf("comm_offset_valid=%u rs485_status=%u\n",
        unsigned(offset_valid), unsigned(rs485_status));
    Serial.println("----------------------------------\n");
    return true;
}

bool applyWheelTuningPreset(int index) {
    if (index < 0 || index >= NUM_LOCAL_ODRIVES) return false;
    WheelCurrentSdoIO io{index};
    if (!wheelSaveFreshIdle(index, io) || !wheelSaveErrorsClear(index, io)) {
        Serial.println("Wheel tune apply blocked: fresh healthy IDLE with zero errors required.");
        return false;
    }
    if (!ensureWheelCurrentLimits(index)) {
        return false;
    }
    Get_Version_msg_t v;
    uint32_t commit = 0;
    wheel_tuning::Map map;
    if (!resolveWheelTuningMap(index, io, v, commit, map)) {
        return false;
    }

    struct PresetWrite {
        const char* name;
        uint16_t ep;
        float target;
    } writes[] = {
        {"vel_gain", map.vel_gain, wheel_tuning::VEL_GAIN},
        {"vel_integrator_gain", map.vel_integrator_gain, wheel_tuning::VEL_INTEGRATOR_GAIN},
        {"vel_integrator_limit", map.vel_integrator_limit, wheel_tuning::VEL_INTEGRATOR_LIMIT},
        {"vel_limit_tolerance", map.vel_limit_tolerance, wheel_tuning::VEL_LIMIT_TOLERANCE},
        {"vel_ramp_rate", map.vel_ramp_rate, wheel_tuning::VEL_RAMP_RATE},
        {"spinout_mech", map.spinout_mech_thresh, wheel_tuning::SPINOUT_MECH_THRESH},
        {"spinout_elec", map.spinout_elec_thresh, wheel_tuning::SPINOUT_ELEC_THRESH},
    };
    float verified[7] = {};
    for (unsigned i = 0; i < 7; ++i) {
        if (!writeAndVerifyTuningFloat(index, io, writes[i].name, writes[i].ep, writes[i].target, verified[i])) {
            return false;
        }
    }

    Serial.printf(
        "Wheel tuning preset verified (fw=0.6.%u commit=0x%08lX): vel_gain=%.3f vel_ig=%.3f vel_il=%.3f tol=%.2f ramp=%.2f spinout=%.1f/%.1f W.\n",
        unsigned(v.Fw_Version_Revision), (unsigned long)commit,
        verified[0], verified[1], verified[2], verified[3], verified[4], verified[5], verified[6]);
    Serial.println("Applied in RAM only (not saved). Test unloaded, then run 'w0 tune save' to persist.");
    return true;
}

bool setWheelTuningParam(int index, const char* name, float value) {
    if (index < 0 || index >= NUM_LOCAL_ODRIVES) return false;
    WheelCurrentSdoIO io{index};
    if (!wheelSaveFreshIdle(index, io) || !wheelSaveErrorsClear(index, io)) {
        Serial.println("Wheel tune set blocked: fresh healthy IDLE with zero errors required.");
        return false;
    }
    Get_Version_msg_t v;
    uint32_t commit = 0;
    wheel_tuning::Map map;
    if (!resolveWheelTuningMap(index, io, v, commit, map)) {
        return false;
    }
    uint16_t endpoint = 0;
    float min_val = 0.0f, max_val = 0.0f;
    if (!wheel_tuning::lookupParam(name, map, endpoint, min_val, max_val)) {
        Serial.printf("Unknown tuning parameter '%s'. Valid: vel_gain, vel_integrator_gain, vel_integrator_limit, vel_limit_tolerance, vel_ramp_rate, spinout_elec, spinout_mech, dc_undervoltage, dc_overvoltage, dc_max_pos_current, dc_max_neg_current.\n",
            name ? name : "");
        return false;
    }
    if (!wheel_tuning::validateValue(value, min_val, max_val)) {
        Serial.printf("Value %.4f out of range for %s [%.4f .. %.4f].\n",
            value, name, min_val, max_val);
        return false;
    }
    float actual = NAN;
    if (!writeAndVerifyTuningFloat(index, io, name, endpoint, value, actual)) {
        return false;
    }
    Serial.printf("Wheel tuning set in RAM only (not saved): %s=%.4f\n", name, actual);
    return true;
}

bool calibrateWheelEncoderOffset(int index) {
    if (index < 0 || index >= NUM_LOCAL_ODRIVES) return false;
    if (!wheelMotionAllowed()) {
        Serial.println("Wheel cal blocked: reconnect, then Stop before calibrating.");
        return false;
    }
    WheelCurrentSdoIO io{index};
    if (!wheelSaveFreshIdle(index, io) || !wheelSaveErrorsClear(index, io)) {
        Serial.println("Wheel cal blocked: fresh healthy IDLE with zero errors required (run w0 clear first if faulted).");
        return false;
    }
    Get_Version_msg_t v;
    uint32_t commit = 0;
    wheel_tuning::Map map;
    if (!resolveWheelTuningMap(index, io, v, commit, map)) {
        return false;
    }

    uint8_t rs485_status = 255;
    if (!wheel_tuning_byte_sdo.read(io, LOCAL_ODRIVE_NODE_ID, map.rs485_status, rs485_status)) {
        Serial.println("Wheel cal blocked: could not read rs485_encoder_group0.status.");
        return false;
    }
    if (rs485_status != 0) {
        Serial.printf("Wheel cal blocked: rs485_encoder_group0.status=%u (expected 0). Check encoder wiring.\n",
            unsigned(rs485_status));
        return false;
    }
    if (io.cancelled()) {
        Serial.println("Wheel cal cancelled before start.");
        return false;
    }

    wheel_limits_verified[index] = false;
    if (!odrives[index]->setVelocity(0.0f, 0.0f)) {
        Serial.println("Wheel cal blocked: zero-speed queue failed.");
        return false;
    }
    const uint32_t before_seq = odrive_data[index].heartbeat_sequence;
    if (!odrives[index]->setState(ODriveAxisState::AXIS_STATE_ENCODER_OFFSET_CALIBRATION)) {
        Serial.println("Wheel cal blocked: calibration state request failed.");
        return false;
    }

    Serial.println("Running ODrive encoder offset calibration (state 7)... keep hands clear.");
    const uint32_t began = millis();
    static constexpr uint32_t CAL_TIMEOUT_MS = 20000;
    bool saw_cal_state = false;
    bool completed = false;
    while (uint32_t(millis() - began) < CAL_TIMEOUT_MS) {
        pumpCanOnce();
        if (Serial.available() > 0 || !Serial || !odriveOnline(index) || !wheelMotionAllowed()) {
            odrives[index]->setVelocity(0.0f, 0.0f);
            odrives[index]->setState(ODriveAxisState::AXIS_STATE_IDLE);
            Serial.println("Wheel cal aborted by USB command, disconnect, or CAN loss.");
            return false;
        }
        const auto& hb = odrive_data[index].last_heartbeat;
        if (hb.Axis_State == ODriveAxisState::AXIS_STATE_ENCODER_OFFSET_CALIBRATION) {
            saw_cal_state = true;
        }
        if (hb.Axis_Error != 0) {
            odrives[index]->setVelocity(0.0f, 0.0f);
            odrives[index]->setState(ODriveAxisState::AXIS_STATE_IDLE);
            Serial.printf("Wheel cal fault: axis_error=0x%08lX (", (unsigned long)hb.Axis_Error);
            printOdriveErrorBits(hb.Axis_Error);
            Serial.println(").");
            return false;
        }
        if (odrive_data[index].heartbeat_sequence != before_seq &&
            hb.Axis_State == ODriveAxisState::AXIS_STATE_IDLE &&
            (saw_cal_state || (uint32_t(millis() - began) > 1000 && hb.Procedure_Result != 1))) {
            completed = true;
            break;
        }
        delay(5);
    }

    if (!completed) {
        odrives[index]->setVelocity(0.0f, 0.0f);
        odrives[index]->setState(ODriveAxisState::AXIS_STATE_IDLE);
        Serial.println("Wheel cal failed: timed out waiting for IDLE.");
        return false;
    }

    if (io.cancelled()) {
        Serial.println("Wheel cal post-check cancelled.");
        return false;
    }
    Get_Error_msg_t errors;
    if (!odrives[index]->getError(errors, 100) || io.cancelled()) {
        Serial.println("Wheel cal post-check failed: no fresh error response.");
        return false;
    }
    const uint8_t proc_res = odrive_data[index].last_heartbeat.Procedure_Result;
    if (errors.Active_Errors || errors.Disarm_Reason || proc_res != 0) {
        Serial.printf("Wheel cal failed: proc_result=%u active=0x%08lX disarm=0x%08lX (",
            unsigned(proc_res), (unsigned long)errors.Active_Errors, (unsigned long)errors.Disarm_Reason);
        printOdriveErrorBits(errors.Active_Errors | errors.Disarm_Reason);
        Serial.println(").");
        return false;
    }

    uint8_t offset_valid = 255;
    if (!wheel_persistence_bool_sdo.read(io, LOCAL_ODRIVE_NODE_ID, map.comm_offset_valid, offset_valid) ||
        !wheel_tuning_byte_sdo.read(io, LOCAL_ODRIVE_NODE_ID, map.rs485_status, rs485_status)) {
        Serial.println("Wheel cal post-check failed: could not verify offset_valid/rs485_status.");
        return false;
    }
    if (offset_valid != 1 || rs485_status != 0) {
        Serial.printf("Wheel cal incomplete: offset_valid=%u rs485_status=%u.\n",
            unsigned(offset_valid), unsigned(rs485_status));
        return false;
    }

    Serial.println("Wheel encoder offset calibration succeeded in RAM (offset_valid=1, rs485_status=0).");
    Serial.println("Test unloaded at low RPM, ensure 35/50 A + tuning are applied, then run 'w0 tune save' to persist.");
    return true;
}
