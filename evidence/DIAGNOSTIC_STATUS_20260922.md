# Supervised robot diagnostics, 2026-09-22 UTC

## Current result

Actuator 3 manual retraction works. The user saw it retract, its forward limit
changed from pressed to open, and fresh diagnostics confirmed zero PWM and M1
current after Stop. User asked to consider actuator 3 good; further actuator
motion testing stopped. Encoder count remained -1 through the manual test, so
automatic positioning/homing is not validated by this result.

Wheels 0 and 3 remain unresolved. Direct native-USB tests reproduced irregular
low-speed responses while bypassing the robot GUI and ESP32 command path.
These tests do not establish a defective encoder, incorrect calibration, or
the specific cause of previously reported high-speed trips.

## Measurements and limits

- Wheel 0: two +5 RPM tests with a temporary 3 A operating limit stopped on
  small negative encoder velocity (approximately -2.4 RPM). User heard the
  motor but did not see clear wheel movement.
- Wheel 3: +5 RPM / 1 s test completed, reaching only about +1.8 RPM. A later
  +10 RPM / 3 s plan stopped at 1.219 s when feedback reached +25.05 RPM.
  Position moved +0.041988 motor turns (15.1 degrees); peak sampled motor
  current magnitude was approximately 1.64 A, and supply stayed near 48 V.
  No native ODrive fault occurred. Post-IDLE encoder feedback briefly reached
  +36.27 RPM and reversed before settling. User could not tell physical motion.
- Wheel 0 alignment calibration, temporary 10 A: host speed guard stopped
  state 7 after 0.195 s at about -63.4 RPM, after only -0.028214 motor turns.
  Native result was CANCELLED, with no active/disarm fault. Initial alignment
  can plausibly exceed the later commanded scan speed; this is inconclusive.
- Wheel 0 alignment calibration, temporary 3 A: host speed guard stopped
  state 7 after 1.047 s at +38.93 RPM. Last full sample had Id setpoint 3 A,
  measured Id 2.895 A, Iq 0.034 A and valid RS485 status. Native result again
  CANCELLED, not a native calibration failure. Calibration remains incomplete.

Every attempted powered test ended with confirmed IDLE and stationary
feedback. Temporary settings were restored to 35 A operating / 38 A trip.
The low-current calibration explicitly verified runtime mapper offsets/status
unchanged before restoring higher limits. Original wheel-0 alignment remained
direction +1, scale 7, offset 1.430450, valid true. Runtime working offset
5.356415 was unchanged by the low-current attempt. No calibration was saved,
no firmware was flashed, and no error-clearing retry loop was used.

## Next step pending user

User confirmed wheel power OFF and ODrive lights OUT, leaving Jetson/ESP32
connected. They were asked to inspect wheel 0's encoder mounting and hub for
play/slippage while gently turning the wheel, without loosening screws.
Do not start more powered tests while that physical check is pending.
Historical user documentation describes a 1:1 wheel drive with no gearing;
software contains no gearbox factor. Exact physical coupling still needs
inspection. The documented 3.7:1 gearbox belongs to the linear actuator.

Arm 1 ESP32 remains absent following the reported broken USB connector.
Selecting actuator pair 1+3 therefore correctly prevents that pair from moving.
Select arm 3 individually for its demonstrated manual control.

## Evidence in this revision

- evidence/act3-manual-retract-20260922T081120Z/result.json
- evidence/arm3-followup-10rpm-20260922T081827Z/result.json
- evidence/arm0-encoder-alignment-20260922T082003Z/result.json
- evidence/arm0-encoder-alignment-low-20260922T082702Z/result.json
- evidence/usb-comparison-20260922T082353Z/snapshot.json
- evidence/usb-arm0-20260922T082519Z/snapshot.json

## Sources used for interpretation

Current native API: https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html

The API documents scan speed in electrical revolutions and notes that user
writes to mapper configuration take effect on reboot, while native calibration
changes take effect immediately. Configuration readback alone cannot establish
runtime rollback.

Legacy calibration source (illustrative only, not S1 firmware source):
https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/encoder.cpp

Velocity tuning: https://docs.odriverobotics.com/v/latest/guides/tuning.html
