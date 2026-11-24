#pragma once
// USB bring-up transport. Read probes must not retransmit forever without ACK.
#include "ODriveCAN.h"
#include "driver/twai.h"
struct ESP32TWAIIntf {};
struct CanMsg { uint32_t id; uint8_t len; uint8_t buffer[8]; };
void onCanMessage(const CanMsg& msg);
static bool sendMsg(ESP32TWAIIntf&, uint32_t id, uint8_t length, const uint8_t* data) {
    twai_status_info_t s = {};
    if (length > 8 || twai_get_status_info(&s) != ESP_OK ||
        s.state != TWAI_STATE_RUNNING || s.tx_error_counter >= 128 || s.rx_error_counter >= 128) return false;
    twai_message_t tx = {};
    tx.identifier = id; tx.data_length_code = length;
    tx.extd = (id & 0x80000000) != 0;
    tx.rtr = data == nullptr;
    tx.ss = tx.rtr || id == 0x7df; // Setpoints retain normal CAN retry semantics.
    if (data) for (uint8_t i = 0; i < length; ++i) tx.data[i] = data[i];
    return twai_transmit(&tx, 0) == ESP_OK; // Queue acceptance, not delivery proof.
}
static void onReceive(const CanMsg& msg, ODriveCAN& odrive) {
    odrive.onReceive(msg.id, msg.len, msg.buffer);
}
static void pumpEvents(ESP32TWAIIntf&, int max_events = 100) {
    twai_message_t rx = {};
    while (max_events-- > 0 && twai_receive(&rx, 0) == ESP_OK) {
        if (rx.rtr || rx.extd || rx.data_length_code > 8) continue;
        CanMsg msg = {}; msg.id = rx.identifier; msg.len = rx.data_length_code;
        for (uint8_t i = 0; i < msg.len; ++i) msg.buffer[i] = rx.data[i];
        onCanMessage(msg);
    }
}
CREATE_CAN_INTF_WRAPPER(ESP32TWAIIntf)
