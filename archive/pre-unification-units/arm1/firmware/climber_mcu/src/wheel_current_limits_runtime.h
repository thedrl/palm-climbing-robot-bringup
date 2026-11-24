#pragma once
// Include after wheelStartCancelled() and pumpCanOnce() have been defined.
// Declare global wheel_current::Sdo wheel_current_sdo; before onCanMessage,
// and feed frames to wheel_current_sdo.observe(msg.id,msg.len,msg.buffer).
struct WheelCurrentSdoIO {
    int index;
    uint32_t now() const { return millis(); }
    bool cancelled() const {
        return Serial.available()>0 || !Serial || !odriveOnline(index) || !wheelMotionAllowed();
    }
    bool drain() const {
        if (!can_initialized) return false;
        // Let earlier telemetry/zero requests leave the queue without adding
        // more background requests while waiting for this transaction.
        const uint32_t start=millis();
        while (uint32_t(millis()-start)<30) {
            if (cancelled()) return false;
            pumpEvents(can_intf);
            twai_status_info_t s={};
            if (twai_get_status_info(&s)!=ESP_OK || s.state!=TWAI_STATE_RUNNING) return false;
            if (s.msgs_to_rx==0 && s.msgs_to_tx==0) return true;
            delay(1);
        }
        return false;
    }
    bool send(uint32_t id,uint8_t length,const uint8_t* data) const {
        return sendMsg(can_intf,id,length,data);
    }
    void pump() const { pumpCanOnce(); }
    void pause() const { delay(1); }
};
struct WheelCurrentApplyIO {
    int index;
    WheelCurrentSdoIO transport;
    bool idleReady() const {
        return !transport.cancelled() &&
            odrive_data[index].last_heartbeat.Axis_State==ODriveAxisState::AXIS_STATE_IDLE &&
            odrive_data[index].last_heartbeat.Axis_Error==0;
    }
    bool read(uint16_t endpoint,float& value) {
        return wheel_current_sdo.readFloat(transport,LOCAL_ODRIVE_NODE_ID,endpoint,value);
    }
    bool write(uint16_t endpoint,float value) {
        return wheel_current_sdo.writeFloat(transport,LOCAL_ODRIVE_NODE_ID,endpoint,value);
    }
    bool awaitReady(bool hard_changed) {
        const uint32_t started=millis();
        while (uint32_t(millis()-started)<2500) {
            pumpCanOnce();
            if (transport.cancelled() ||
                odrive_data[index].last_heartbeat.Axis_State!=ODriveAxisState::AXIS_STATE_IDLE) return false;
            // Give current-sensor reconfiguration a full settling interval.
            if (hard_changed && uint32_t(millis()-started)<750) {delay(2);continue;}
            Get_Error_msg_t errors;
            const bool fresh=odrives[index]->getError(errors,100);
            if (transport.cancelled()) return false;
            if (fresh) {
                if (errors.Disarm_Reason || (errors.Active_Errors & ~uint32_t(1))) return false;
                if (errors.Active_Errors==0 && idleReady()) return true;
            }
            delay(2);
        }
        return false;
    }
};
bool ensureWheelCurrentLimits(int index) {
    if (index<0 || index>=NUM_LOCAL_ODRIVES) return false;
    wheel_limits_verified[index] = false;
    if (wheelStartCancelled(index)) return false;
    // A settings-only command must not disturb an already healthy IDLE axis.
    // The motion caller has already zeroed and verified stopped before entry.
    const bool already_idle=odriveOnline(index) &&
        odrive_data[index].last_heartbeat.Axis_State==ODriveAxisState::AXIS_STATE_IDLE &&
        odrive_data[index].last_heartbeat.Axis_Error==0;
    uint32_t sequence=odrive_data[index].heartbeat_sequence;
    if (!already_idle) {
        if (!odrives[index]->setVelocity(0.0f,0.0f)) return false;
        pumpCanFor(50);
        if (wheelStartCancelled(index)) return false;
        sequence=odrive_data[index].heartbeat_sequence;
        if (!odrives[index]->setState(ODriveAxisState::AXIS_STATE_IDLE)) return false;
    }
    const uint32_t started=millis();
    while (uint32_t(millis()-started)<1000) {
        pumpCanOnce();
        if (wheelStartCancelled(index)) return false;
        if (odrive_data[index].heartbeat_sequence!=sequence) {
            if (already_idle && (odrive_data[index].last_heartbeat.Axis_State!=ODriveAxisState::AXIS_STATE_IDLE ||
                odrive_data[index].last_heartbeat.Axis_Error!=0)) {
                Serial.println("Wheel current limits blocked: IDLE changed during verification.");return false;
            }
            if (odrive_data[index].last_heartbeat.Axis_State==ODriveAxisState::AXIS_STATE_IDLE &&
                odrive_data[index].last_heartbeat.Axis_Error==0) break;
        }
        delay(2);
    }
    if (odrive_data[index].heartbeat_sequence==sequence ||
        odrive_data[index].last_heartbeat.Axis_State!=ODriveAxisState::AXIS_STATE_IDLE ||
        odrive_data[index].last_heartbeat.Axis_Error!=0) {
        Serial.println("Wheel current limits blocked: fresh healthy IDLE not confirmed.");return false;
    }
    WheelCurrentApplyIO io{index,{index}};
    // A pending persistent fault remains visible; never auto-clear it.
    if (!io.awaitReady(false)) {Serial.println("Wheel current limits blocked: ODrive errors present.");return false;}
    Get_Version_msg_t v;
    if (!odrives[index]->getVersion(v,100) || io.transport.cancelled()) {
        Serial.println("Wheel current limits blocked: version not verified.");return false;
    }
    // Check public version before accessing even the stable commit endpoint.
    if (v.Protocol_Version!=2 || v.Hw_Version_Major!=5 || v.Hw_Version_Minor!=2 ||
        v.Hw_Version_Variant!=0 || v.Fw_Version_Major!=0 || v.Fw_Version_Minor!=6 ||
        (v.Fw_Version_Revision!=11 && v.Fw_Version_Revision!=12) || v.Fw_Version_Unreleased) {
        Serial.println("Wheel current limits blocked: unsupported ODrive firmware/hardware.");return false;
    }
    uint32_t commit=0;
    wheel_current::Map map;
    if (!wheel_current_sdo.readU32(io.transport,LOCAL_ODRIVE_NODE_ID,13,commit) ||
        !wheel_current::supported(v.Protocol_Version,v.Hw_Version_Major,v.Hw_Version_Minor,
          v.Hw_Version_Variant,v.Fw_Version_Major,v.Fw_Version_Minor,v.Fw_Version_Revision,
          v.Fw_Version_Unreleased,commit,map)) {
        Serial.printf("Wheel current limits blocked: unsupported/missing firmware commit 0x%08lx.\n",(unsigned long)commit);
        return false;
    }
    wheel_current::Result result;
    if (!wheel_current::apply(io,map,result)) {
        odrives[index]->setVelocity(0.0f,0.0f);
        Serial.printf("Wheel current limits blocked: %s. No motion.\n",result.failure);
        return false;
    }
    wheel_limits_verified[index] = true;
    wheel_limits_verified_gen[index] = can_recovery.generation;
    Serial.printf("Wheel current limits verified: soft=%.2f A hard=%.2f A; fw=0.6.%u commit=0x%08lx. No NVM save.\n",
        result.soft,result.hard,v.Fw_Version_Revision,(unsigned long)commit);
    return true;
}

bool printWheelCurrentLimits(int index) {
    if (index<0 || index>=NUM_LOCAL_ODRIVES) return false;
    WheelCurrentSdoIO io{index};
    Get_Version_msg_t v;
    if (io.cancelled() || !odrives[index]->getVersion(v,100) || io.cancelled() ||
        v.Protocol_Version!=2 || v.Hw_Version_Major!=5 || v.Hw_Version_Minor!=2 ||
        v.Hw_Version_Variant!=0 || v.Fw_Version_Major!=0 || v.Fw_Version_Minor!=6 ||
        (v.Fw_Version_Revision!=11 && v.Fw_Version_Revision!=12) || v.Fw_Version_Unreleased) {
        Serial.println("Wheel current limits unavailable: version/link not verified.");return false;
    }
    uint32_t commit=0; wheel_current::Map map;
    float soft=NAN,hard=NAN;
    if (!wheel_current_sdo.readU32(io,LOCAL_ODRIVE_NODE_ID,13,commit) ||
        !wheel_current::supported(v.Protocol_Version,v.Hw_Version_Major,v.Hw_Version_Minor,
          v.Hw_Version_Variant,v.Fw_Version_Major,v.Fw_Version_Minor,v.Fw_Version_Revision,
          v.Fw_Version_Unreleased,commit,map) ||
        !wheel_current_sdo.readFloat(io,LOCAL_ODRIVE_NODE_ID,map.soft,soft) ||
        !wheel_current_sdo.readFloat(io,LOCAL_ODRIVE_NODE_ID,map.hard,hard)) {
        Serial.println("Wheel current limits unavailable: fresh endpoint read failed.");return false;
    }
    Serial.printf("Wheel current limits: soft=%.2f A hard=%.2f A; target=%.2f/%.2f A; fw=0.6.%u commit=0x%08lx. Read-only.\n",
        soft,hard,wheel_current::SOFT_AMPS,wheel_current::HARD_AMPS,v.Fw_Version_Revision,(unsigned long)commit);
    return true;
}
