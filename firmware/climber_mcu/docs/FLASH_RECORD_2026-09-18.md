# Arm 1 CAN Pin Fix - 2026-09-18

## Change

Arm 1 only now uses ESP Nano ESP32 D9 (GPIO18) for CAN TX and D10 (GPIO21) for CAN RX.
Connect these to the CAN transceiver's TX input and RX output, respectively.
Profiles for units 0, 2, and 3 retain TX D8 / RX D9. They were compile-checked but not flashed.

No actuator, RoboClaw, ToF, wheel speed, motor calibration, or protection settings were changed.
ToF remains off. This change is scoped to the USB bring-up firmware in `src/bringup.cpp`, not the separate micro-ROS `main.cpp`.

## Source and Build

Based on `/Users/stem/dev/farm_robotics/climber_arm1_recovery_2026-09-18`.
The only source change is a `UNIT_ID == 1` conditional around the two CAN pin definitions.

PlatformIO builds passed for `bringup_unit0`, `bringup_unit1`, `bringup_unit2`, and `bringup_unit3`.
Existing missing-return warnings in the vendored RoboClaw library remain; all four builds completed successfully.

Firmware SHA-256:
`f81a47569c17fbadbf21621537f6b03fbf59f6d33725abf4a2997ab8aabe5029`

Source `src/bringup.cpp` SHA-256:
`bd2f8466850a0e9c91b01f52636be95e079561712ab957f32e0ec5463fdab0cc`

## Flash

- Operator confirmed motor power off and supervisor/GUI closed.
- Flashed only Nano ESP32 serial `744DBDA10314` using serial-filtered native `dfu-util` 0.11.
- Application download: 402240 bytes; successful DFU firmware validation.
- DFU detach rebooted the ESP. No full erase or bootloader replacement.
- DFU readback is not implemented by this firmware; validation is upload status plus post-reboot communication checks.

## Verified Result

Before changing the pins, ESP CAN repeatedly entered BUS_OFF, with zero received frames, including after a CAN reset.
The ODrive's USB configuration already matched: CANSimple enabled, 1000000 bitrate, node 0, heartbeat interval 100 ms.

After flashing:

- ESP reports unit 1, local ODrive node 0.
- Initialization reports `TX=D9(GPIO18) RX=D10(GPIO21)`.
- CAN state RUNNING; TX and RX error counters zero; bus errors zero.
- Two separated `wheel diag` checks both report `heartbeat: online`.
- ODrive responds with protocol 2, hardware 5.2.0, firmware 0.6.11.
- ODrive state IDLE; bus approximately 1.08-1.09 V with main motor power off.
- Active errors INITIALIZING and DC_BUS_UNDER_VOLTAGE were observed with main power off; no errors were cleared.
- CAN startup shows 17 missed RX frames during initialization, but no overrun or bus errors. Do not describe this as zero dropped frames.
- REV switch pressed, FWD switch open; ToF off.

No motor motion commands were issued. Powered wheel operation remains to be tested separately.
Serial connections were closed after diagnostics.

## Next Communication Check

After confirming wiring and restoring appropriate ODrive main power, open the supervisor and run `u1 wheel diag` before any motion test.
Verify heartbeat online and examine active errors. The earlier RoboClaw serial acknowledgement issue is separate and is not resolved or tested by this CAN pin change.
