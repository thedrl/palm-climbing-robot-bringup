# Arm 1 Actuator Stop Candidate - 2026-09-19

Status: built, offline-tested, and flashed to arm 1 on 2026-09-19.
Identity and new stop revision verified after reboot. NOT motor-tested.
See FLASH_RECORD_2026-09-19.md for installation details.

## Purpose

The operator reports an arm 1 actuator jolt when releasing either extend or
retract, without a wheel button held, particularly with multiple arms connected.
This copy isolates one plausible cause; it is not a confirmed hardware diagnosis.

Base: climber_arm1_can_d9_d10_2026-09-18. Older source and the existing submission
ZIP were not modified. FLASH_RECORD_2026-09-18.md describes that PREVIOUS image,
not this candidate. The previous image is retained with PREVIOUS in its filename.

## Change

For UNIT_ID=1 only, actuator stop retains the two existing zero-power commands
but no longer follows them with SpeedM1(0) or SpeedM2(0). Those speed commands
engage encoder velocity PID after open-loop motion. This candidate keeps the
stop in open-loop mode, as the preserved arm 0 and arm 2 sources already do.

Status reports: 2026-09-19-u1-open-loop-stop1.

Unchanged: CAN TX D9 / RX D10, RoboClaw settings, motion powers, ramping,
limit switches, timeouts, stop acknowledgements/retries, ToF, wheel control,
GUI and supervisor. No current or voltage limits were raised or disabled.

Build only bringup_unit1. Other profiles exist in the inherited configuration
but this is not a replacement for the separate working arm 0/2/3 projects.

## Verification

- PlatformIO bringup_unit1 build passed.
- test_stop.py compiles the actual drive and actuatorStop function bodies with
  a fake RoboClaw. 13 scenarios for each of four UNIT_ID values passed (52 total).
- Cases cover both channels, forward/reverse followed by zero, acknowledgement
  combinations, and stopping both channels. Unit 1 emits no SpeedM command;
  non-unit-1 behavior in this source remains unchanged.
- The harness uses a one-attempt retry adapter, not a full hardware simulation.
- Existing vendored RoboClaw warnings were not addressed by this narrow fix.
- Initial offline preparation involved no hardware access. Subsequent flashing
  and non-motion serial verification are recorded separately below; no motion
  commands or RoboClaw/ODrive configuration changes were made.

## Remaining Uncertainty

The Jetson SSH connection timed out, so its active GUI version is unverified.
Sequential global stop dispatch in the local GUI may delay later arms if a USB
connection blocks. A separate simultaneous-wheel-button stop/restart issue was
reproduced, but the operator explicitly rules that scenario out for this jolt.
This firmware change alone does not establish the cause of the multi-arm effect.

## Controlled Comparison

Flash only the identified arm 1 ESP with both motor supplies off and the
GUI/supervisor closed. Confirm the new revision using u1 status afterward.
With the robot secured and unloaded, compare brief forward/reverse button
release first with arm 1 alone, then with other ESP USB connections present.
Keep the same supply, speed and physical setup for the comparison. Stop testing
if unexpected motion persists. Do not test on a tree to validate this change.
Record the release time, commands, encoder change and any other arm movement.

The software tests do not verify braking distance, holding force or physical
safety under load. Do not disable stop or limit protection to test this candidate.
