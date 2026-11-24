# Arm 1 Stop Fix Installation

- Operator confirmed motor power off and connected the ESP to this Mac.
- GUI/supervisor process check found no matching processes.
- Target Nano ESP32 USB serial: 744DBDA10314.
- Application image: firmware_unit1_stop_fix.bin, 402192 bytes.
- SHA-256: 2ba7ed33b47dce78322bf3d38a4df7f16754c59ba290930bb29977f9893f3962.
- Serial-filtered native dfu-util download completed, exit 0, DFU no error.
- Serial-filtered DFU detach rebooted into the application.
- Console reported unit 1 and local ODrive node 0.
- Status reported Actuator stop revision: 2026-09-19-u1-open-loop-stop1.
- Both actuator limits open; encoder invalid with motor power off;
  CAN uninitialized, wheel offline, ToF uninitialized and off.
- Console needed normal DTR assertion to respond; opening with DTR false
  initially produced no console output. Normal connection then verified status.
- USB console was closed at the end. No motion commands sent.
- No full-chip erase, other-unit flashing or controller setting changes.
- DFU byte-for-byte flash readback was not performed. Verification is uploader
  completion plus the new revision running after reboot.
- Physical jolt correction remains unverified until a controlled bench test.

The previous image and September 18 record remain available for rollback.
The older farm bot 19sep2026.zip was not changed by this installation.
