#pragma once
// Include after wheel_current_limits_runtime.h. Declare the BoolSdo global and
// feed its observe() from onCanMessage beside the existing current SDO observer.
// This explicit command never applies settings, clears errors, or arms a motor.
static bool wheelSaveFreshIdle(int index, WheelCurrentSdoIO& io) {
    const uint32_t sequence=odrive_data[index].heartbeat_sequence;
    const uint32_t began=millis();
    while (uint32_t(millis()-began)<1000) {
        if (io.cancelled()) return false;
        pumpCanOnce();
        if (io.cancelled()) return false;
        if (odrive_data[index].heartbeat_sequence!=sequence) {
            return odrive_data[index].last_heartbeat.Axis_State==ODriveAxisState::AXIS_STATE_IDLE &&
                odrive_data[index].last_heartbeat.Axis_Error==0;
        }
        delay(2);
    }
    return false;
}
static bool wheelSaveErrorsClear(int index, WheelCurrentSdoIO& io) {
    Get_Error_msg_t errors;
    if (io.cancelled() || !io.drain() || io.cancelled() ||
        !odrives[index]->getError(errors,100) || io.cancelled()) return false;
    Serial.printf("Wheel save error snapshot: active=0x%08lX disarm=0x%08lX\n",
        (unsigned long)errors.Active_Errors,(unsigned long)errors.Disarm_Reason);
    return errors.Active_Errors==0 && errors.Disarm_Reason==0;
}
bool saveWheelCurrentLimits(int index) {
    if (index<0 || index>=NUM_LOCAL_ODRIVES) return false;
    WheelCurrentSdoIO io{index};
    if (!wheelSaveFreshIdle(index,io) || !wheelSaveErrorsClear(index,io)) {
        Serial.println("Wheel save blocked: fresh healthy IDLE required; no settings saved.");return false;
    }
    Get_Version_msg_t version;
    if (!io.drain() || io.cancelled() || !odrives[index]->getVersion(version,100) || io.cancelled() ||
        version.Protocol_Version!=2 || version.Hw_Version_Major!=5 ||
        version.Hw_Version_Minor!=2 || version.Hw_Version_Variant!=0 ||
        version.Fw_Version_Major!=0 || version.Fw_Version_Minor!=6 ||
        (version.Fw_Version_Revision!=11 && version.Fw_Version_Revision!=12) || version.Fw_Version_Unreleased) {
        Serial.println("Wheel save blocked: supported ODrive firmware/hardware not verified.");return false;
    }
    uint32_t commit=0;
    wheel_current::Map map;
    if (!wheel_current_sdo.readU32(io,LOCAL_ODRIVE_NODE_ID,13,commit) ||
        !wheel_current::supported(version.Protocol_Version,version.Hw_Version_Major,
          version.Hw_Version_Minor,version.Hw_Version_Variant,version.Fw_Version_Major,
          version.Fw_Version_Minor,version.Fw_Version_Revision,version.Fw_Version_Unreleased,commit,map)) {
        Serial.println("Wheel save blocked: known firmware commit not verified.");return false;
    }
    float soft=NAN,hard=NAN;
    if (!wheel_current_sdo.readFloat(io,LOCAL_ODRIVE_NODE_ID,map.soft,soft) ||
        !wheel_current_sdo.readFloat(io,LOCAL_ODRIVE_NODE_ID,map.hard,hard)) {
        Serial.println("Wheel save blocked: current limit read failed.");return false;
    }
    const uint16_t first=wheel_persistence::startupFirst(version.Fw_Version_Revision);
    uint8_t flags[5]={255,255,255,255,255};
    for (unsigned i=0;i<5;++i) {
        if (!first || !wheel_persistence_bool_sdo.read(io,LOCAL_ODRIVE_NODE_ID,first+i,flags[i])) {
            Serial.printf("Wheel save blocked: startup flag endpoint %u not verified.\n",unsigned(first+i));return false;
        }
    }
    Serial.printf("Wheel save snapshot: firmware=0.6.%u commit=0x%08lX soft=%.2f A hard=%.2f A\n",
        unsigned(version.Fw_Version_Revision),(unsigned long)commit,soft,hard);
    Serial.printf("Wheel save startup flags: motor_cal=%u index_search=%u encoder_cal=%u closed_loop=%u homing=%u\n",
        unsigned(flags[0]),unsigned(flags[1]),unsigned(flags[2]),unsigned(flags[3]),unsigned(flags[4]));
    if (!wheel_persistence::validSnapshot(soft,hard,flags)) {
        Serial.println("Wheel save blocked: require existing 35/50 A and all startup motion flags OFF. No settings changed.");return false;
    }
    // Recheck current values and fresh status after the startup reads. No current
    // write or implicit apply is permitted in this persistence-only command.
    if (!wheel_current_sdo.readFloat(io,LOCAL_ODRIVE_NODE_ID,map.soft,soft) ||
        !wheel_current_sdo.readFloat(io,LOCAL_ODRIVE_NODE_ID,map.hard,hard) ||
        !wheel_persistence::validSnapshot(soft,hard,flags) ||
        !wheelSaveFreshIdle(index,io) || !wheelSaveErrorsClear(index,io) ||
        !io.drain() || io.cancelled() ||
        odrive_data[index].last_heartbeat.Axis_State!=ODriveAxisState::AXIS_STATE_IDLE ||
        odrive_data[index].last_heartbeat.Axis_Error!=0) {
        Serial.println("Wheel save blocked: final checks cancelled or changed; no settings saved.");return false;
    }
    // Reboot Action=1 is supported since ODrive 0.6.9. Call once only: a lost
    // acknowledgment or disconnect must never cause automatic repeated saves.
    const bool sent=odrives[index]->reset(ODriveCAN::SaveConfiguration);
    if (!sent) {
        Serial.println("Wheel save transmission failed; persistence NOT verified.");return false;
    }
    Serial.println("Wheel configuration save requested once; ODrive reset expected. Persistence NOT yet verified.");
    Serial.println("After CAN is ready, use 'w0 current' to verify 35/50 A before any apply or movement.");
    return true;
}
