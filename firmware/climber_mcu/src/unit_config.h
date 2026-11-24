#pragma once
#include <stdint.h>

#ifndef UNIT_ID
#define UNIT_ID 0
#endif

static_assert(UNIT_ID >= 0 && UNIT_ID <= 3, "UNIT_ID must be in 0..3");

#ifndef LOCAL_ODRIVE_NODE_ID
#define LOCAL_ODRIVE_NODE_ID UNIT_ID
#endif

// --- Hardware pinout ---
// Arm 1 was rewired to D9/D10 after repeated BUS_OFF on D8/D9 (2026-09-18).
#ifndef PIN_CAN_TX
  #if UNIT_ID == 1
    #define PIN_CAN_TX D9
  #else
    #define PIN_CAN_TX D8
  #endif
#endif

#ifndef PIN_CAN_RX
  #if UNIT_ID == 1
    #define PIN_CAN_RX D10
  #else
    #define PIN_CAN_RX D9
  #endif
#endif

#if UNIT_ID == 1
  #define UNIT_CAN_PIN_LABEL "TX=D9 RX=D10"
#else
  #define UNIT_CAN_PIN_LABEL "TX=D8 RX=D9"
#endif

// --- Actuator stroke limits ---
// Unit 2 consistently measures 1343-1344 ticks full stroke (vs >=1350 on units 0, 1, 3).
#ifndef ACTUATOR_MIN_ENCODER_RANGE_TICKS_CONFIG
  #if UNIT_ID == 2
    #define ACTUATOR_MIN_ENCODER_RANGE_TICKS_CONFIG 1330
  #else
    #define ACTUATOR_MIN_ENCODER_RANGE_TICKS_CONFIG 1350
  #endif
#endif

// --- Per-unit actuator behavior traits ---
// Unit 0 stalls at lower centering powers -> retain full power, coarse 40..60% window, no boost.
#ifndef UNIT_CENTER_DISABLE_SLOWDOWN
  #define UNIT_CENTER_DISABLE_SLOWDOWN (UNIT_ID == 0)
#endif
#ifndef UNIT_CENTER_COARSE_WINDOW
  #define UNIT_CENTER_COARSE_WINDOW (UNIT_ID == 0)
#endif
#ifndef UNIT_CENTER_ABORT_ON_STALL
  #define UNIT_CENTER_ABORT_ON_STALL (UNIT_ID == 0)
#endif
#ifndef UNIT_CENTER_VERIFY_SETTLED
  #define UNIT_CENTER_VERIFY_SETTLED (UNIT_ID == 0)
#endif

// Unit 1 uses 80ms limit debounce, 150ms stop-and-confirm, creep-on-transient, FWD encoder zero.
#ifndef UNIT_HOMING_CONFIRM_AND_CREEP
  #define UNIT_HOMING_CONFIRM_AND_CREEP (UNIT_ID == 1)
#endif

// Opt-in bench/touch diagnostics (act bench, act touch, act center probe/checked).
#ifndef ENABLE_BENCH_DIAGNOSTICS
  #define ENABLE_BENCH_DIAGNOSTICS 0
#endif
