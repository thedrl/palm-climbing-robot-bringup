# Replacement Unit 1 restored, 2026-09-22 UTC

User confirmed replacement ESP32 belongs to GUI Unit 1 and explicitly authorized
flashing with Unit 1 motor power off, robot supported and USB connected.

Old ESP32: 744DBDA10314. New ESP32: 48CA432DF684.
The replacement enumerated as USB CDC/DFU but did not respond to the robot
firmware identity command. No conclusion about its previous firmware was made.

Uploaded existing verified V17 Unit 1 image once; upload returned zero.
SHA-256: 7085533e3073678a2dcc8c08d9efa2434a9b316c3a5c0979e6c2837cba085742.
Runtime unit=1, revision=2026-09-21-v17-current-persistence confirmed afterward.
No source, GUI mapping or other arm firmware changed.

After the user restored ODrive power, stationary connection checks confirmed:
- CAN recovery READY, driver RUNNING, transmit/receive/error/drop counters zero.
- Fresh ODrive node 0 heartbeat; IDLE; active_errors=0; disarm_reason=0.
- Bus 48.10 V and 0.000 A. Approximately -0.01 RPM at diagnostic readback.
- ODrive firmware 0.6.11, hardware 5.2.0; 35 A operating / 38 A trip readback.
- Current limits were read only, not applied or saved.
- Historical startup fault snapshot remains separate from clear current faults.

No movement test was performed. USB ports were closed afterward.
V18 GUI auto-discovers Unit 1 from its firmware identity; use Auto-detect/Connect.
The original package FIRMWARE_MAP.json is preserved; use CURRENT_FIRMWARE_MAP.json
in this evidence folder for subsequent hardware scripts to include the replacement.

Powered readback: ../replacement-unit1-can-20260922T105525Z/result.json
Upload and identity: result.json
