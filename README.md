# V18 GUI actuator recovery

This is a GUI update compatible with the V17 ESP32 firmware already installed. It does not require a firmware flash. V17 and its logs remain preserved in their existing folder. The USB bring-up GUI is separate from the micro-ROS packages.

Run:

```bash
bash /home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery/RUN_GUI.sh
```

A selection strip below Stop All displays the chosen actuator(s), with selection-only buttons for 0, 1, 2, 3, 0+2 and 1+3. Clicking one does not move anything or change your Learn mappings. Release controls before pressing extend/retract.

The Global actuator group now has **Clear All Actuator Errors**. Recovery disables controller input, sends Stop, releases acknowledged actuator software latches through the existing `act stop` command, and checks fresh actuator telemetry separately for each connected unit. It reports missing units and unresolved faults. It does not clear ODrive faults, reset actuator encoders, change limits, or start motion. A failed USB connection must not prevent later units receiving Stop or an explicitly requested fault-clear command. Gamepad operation requires the user to enable control again, release the controls, and select the desired actuator or pair; existing Learn mappings are retained.

The button cannot override an active RoboClaw fault, a pressed destination limit, a lost USB connection, or failed current/voltage checks. “Zero acknowledged” is not proof of physical movement or a successful retract test.

## Findings from this session

- Unit 1's ESP32 USB stream failed during the earlier session. The next GUI connection only found units 0, 2, and 3. This is not evidence of an isolated Unit 1 CAN fault.
- The old GUI aborted several global broadcasts when it reached Unit 1's failed port; arms 2 and 3 never received those commands.
- The latest controller actuator attempt was rejected before transmission because no actuator or pair was selected.
- Current stationary reads on units 0, 2 and 3 show the retract switches open. Units 2 and 3 report their extension switches pressed. Unit 0 reports BOTH switches open despite the user's fully extended description; its switch position/wiring still needs physical verification.
- All three available RoboClaws acknowledged actuator Stop; fresh diagnostics reported zero PWM and zero connected-M1 current. Unit 3's known unused-M2 current offset remains separate.
- Wheel 0 CAN has recovered and reports voltage. Earlier +500 RPM attempts caused SPINOUT; latest attempts caused VELOCITY_LIMIT_VIOLATION about 0.1 seconds after a +500 RPM request, with encoder feedback around -753 RPM. This is reported encoder motion, not independent proof of physical direction. The cause is not established, and no encoder settings were changed.
- Wheel 3's earlier CURRENT_LIMIT_VIOLATION survived some global clear attempts because Unit 1's USB failure interrupted delivery. Later logs show Wheel 3 tracking about +/-500 RPM, followed by a latched undervoltage fault on the current read. Its CAN currently responds around 48 V. Historical reading gaps align with the actuator Stop handler taking 5.7-8.8 seconds; normal CAN updates resume afterward. The exact low-level delay and undervoltage cause remain unverified.
- Direct USB at 07:29 UTC confirms Wheel 0 IDLE, is_armed=0, bus current=0 A and electrical power=0 W, while its phase-current property retains 29.786 A. This is not a continuous present 25-30 A draw. Wheel 0/3 fault causes require separate controlled investigation; this GUI change does not claim to fix them.

## Scope and limits

No motion test, firmware upload, calibration, current-limit change, or nonvolatile save was performed for this update. The requested operating settings remain 35 A / 38 A for ODrive and a maximum 6.75 A for the actuator; this revision does not raise them. Physical reversal remains untested.

The copied `units/` and persistence helpers are the pre-existing V17 source/reference package, not newly built V18 firmware. `FIRMWARE_MAP.json` describes that prior deployment. Do not interpret the GUI revision number as an ESP32 firmware update.

Current diagnostics are recorded under the preserved V17 `evidence/all-recovery-status-20260922T065347Z/` and `evidence/actuator-stop-readback-20260922T065726Z/` directories. Historical GUI evidence is in its `logs/` directory, sessions `20260922T064702Z-5963-8193c6c2` and `20260922T065008Z-6020-b5a3a736`.

Latest read-only evidence after resuming: preserved V17 `evidence/all-recovery-status-20260922T072738Z/`, `evidence/usb-arm0-20260922T072924Z/` and `evidence/usb-comparison-20260922T072925Z/`. Latest actuator attempts in sessions `072113Z` and `072347Z` still contain no dispatched actuator move: the GUI reports no actuator selected. Arm 1 remains absent after the user reported its damaged ESP32 USB-C connector.

Offline verification: 158 GUI/transport/recovery tests passed, including partial serial framing, stale telemetry, Stop acknowledgment ordering, disconnected-unit delivery, cancellation, held-control release, missing targets and CLI cleanup. No physical retract test was performed.
