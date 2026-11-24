# palm-climbing-robot-bringup

Standalone ESP32 USB bring-up firmware, ODrive/RoboClaw tuning and persistence tools, and non-ROS USB supervisor GUI for the `palm-tree-climber` robot.

## V19 — Unified ESP32 Firmware & GUI CAN Controls

Firmware revision `2026-11-19-v19-unified` consolidates the four diverged `units/arm0`–`units/arm3` bring-up projects into a single PlatformIO project (`firmware/climber_mcu/`) and adds GUI controls and live fault decoding for the V19 ODrive CAN wheel tuning and 35 A / 50 A current-limit settings.

### Firmware Unification & Packaging
- **Single canonical project:** Replaced the four separate `units/arm0`–`units/arm3` source trees with `firmware/climber_mcu/` (`platformio.ini` targeting `espressif32@7.0.0`, default envs `bringup_unit0`–`bringup_unit3`, opt-in bench envs `bringup_unit0_bench` and `bringup_unit2_bench`). Legacy per-arm sources were moved to `archive/pre-unification-units/`; rollback `.bin` images remain in `units/arm{0..3}/firmware_images/`.
- **Declarative per-unit config (`firmware/climber_mcu/src/unit_config.h`):** Centralizes genuine hardware differences by `UNIT_ID`:
  - CAN pins: `D9` TX / `D10` RX on Unit 1; `D8` TX / `D9` RX on Units 0, 2, 3.
  - Minimum encoder stroke: `1330` ticks on Unit 2; `1350` ticks on Units 0, 1, 3.
  - Unit 0 centering profile: full centering power ceiling, coarse 40–60% window, settled-stop verification, and abort-on-stall without breakaway boost.
  - Unit 1 homing profile: 80 ms limit debounce, 150 ms stop-and-confirm creep, and FWD-limit encoder zeroing.
- **Consolidated vendor libraries (`firmware/climber_mcu/vendor/`):** Standardized all 4 units onto `ODriveArduino`, Arm 0's patched `roboclaw_v448` (ESP32 RX `clear()` before transactions/retries, 5-byte v4.4.8 replies for commands 59 and 75, command 15 `ReadSerialTimeout`), and Arm 0/2's checked-I2C `Adafruit_VL53L0X` driver, dropping unused OLED library dependencies.
- **Single-source headers & updated tooling:** Removed duplicated persistence/tuning headers from `persistence/` (tests now compile directly against `firmware/climber_mcu/src/`). Updated `build_all.py` (portable `platformio` lookup + sequential `v19_unified.bin` export) and `verify_persistence.py`, and added `persistence/test_unified_bringup.py`.

### Cross-Arm Firmware Fixes & ODrive CAN Control
- **Arm 3 open-loop stop fix:** Removed `SpeedM1(0)` / `SpeedM2(0)` after `ForwardM*(0)` / `BackwardM*(0)` across all four units so stopping no longer engages RoboClaw velocity PID.
- **Fleet-wide RoboClaw & ToF parity:** Enabled full v4.4.8 `act config` readback (fixing the GUI **Read controller settings** button on Arms 1 and 3), `delay(12); roboclaw.clear();` packet synchronization, verified TCA9548A ToF mux readback, `tof scan|pins|direct|mux`, and the 50 ms `pumpCanFor(50)` settle pause in `clearLocalWheelErrors()` across all 4 arms.
- **Opt-in bench diagnostics:** Gated `act bench`, `act touch`, `act center probe`, and `act center checked` under `-DENABLE_BENCH_DIAGNOSTICS=0` (parameterized `actuator_touch*.h` for Unit 0 at 253 mm / 4 s and Unit 2 at 236 mm / 8 s / 800-tick cap in `*_bench` builds).
- **ODrive CAN tuning & limits:** Retains `INPUT_MODE_VEL_RAMP`, 35 A soft / 50 A hard current limits, `wheel tune` / `wheel tune apply` / `wheel tune set` / `wheel tune save`, and `wheel cal`; updated remaining stale `"38 A"` messages in `wheel_current_limits.h` to `"50 A"`.
- **Status reporting:** `status` now prints the compiled CAN pinout, min-stroke, centering/homing flags, and bench mode; `firmware` reports `2026-11-19-v19-unified`.

### GUI CAN Controls & Fault Decoding (`Gui/`)
- **Live ODrive fault decoding (`Gui/odrive_live.py`):** Added `ERROR_NAMES` and `format_error()` to decode `SPINOUT_DETECTED`, `CURRENT_LIMIT_VIOLATION`, `VELOCITY_LIMIT_VIOLATION`, `DC_BUS_UNDER_VOLTAGE`, `DC_BUS_OVER_VOLTAGE`, and other `ODriveEnums.h` flags alongside the hex error code.
- **Global & per-unit CAN UI (`Gui/climber_gui5.py`):**
  - Expanded the Global **Live ODrive readings** table with **State** and **Status** columns.
  - Added read-only **Read Tuning** (`wheel tune`) and **Read Limits** (`wheel current`) buttons under **Wheels (all units)** with per-unit USB failure isolation in `_run_silent()`.
  - Added a per-unit **CAN Tuning & Limits (35 A / 50 A)** panel on each Unit tab with read buttons, confirmation-gated **Apply Climb Preset (RAM)**, **Save Tuning to Flash**, and **Calibrate Encoder (State 7)** actions (`_confirm_wheel_action()`), plus a range-validated parameter setter (`WHEEL_TUNING_PARAMS` matching `wheel_tuning_policy.h`).

### Verification Scope (V19)
- **Built offline:** `python3 build_all.py` built `bringup_unit{0..3}_v19_unified.bin` under `units/arm{0..3}/firmware_images/` (`bringup_unit0_bench` and `bringup_unit2_bench` also compiled cleanly).
- **Offline host tests passing:** `verify_persistence.py` passed all ASan/UBSan C++ persistence & tuning tests (`76` wheel persistence, `101` wheel tuning, `42` actuator persistence, `21` runtime adapter), 4-unit open-loop stop (`13` scenarios × 4 units), `unit_config.h` matrix (`8` combos), and `roboclaw_v448` synthetic-UART (`4` checks). `Gui/test_controller_inputs.py` passed `29/29`, alongside standalone `odrive_live` fault checks and `WHEEL_TUNING_PARAMS` parity checks.
- **Pending on-robot / Orin checks:** Full Tk/`pyserial` GUI tests (`Gui/test_*.py`) on the Jetson Orin Python+Tk environment, and staged pilot flashing + stationary/unloaded hardware validation (`FIRMWARE_MAP.json` still reflects the physically installed V17 images until flashed).

---

## V18 — GUI Actuator Recovery

This is a GUI update compatible with the V17 ESP32 firmware already installed. It does not require a firmware flash. V17 and its logs remain preserved in their existing folder. The USB bring-up GUI is separate from the micro-ROS packages.

Run:

```bash
bash /home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery/RUN_GUI.sh
```

A selection strip below Stop All displays the chosen actuator(s), with selection-only buttons for 0, 1, 2, 3, 0+2 and 1+3. Clicking one does not move anything or change your Learn mappings. Release controls before pressing extend/retract.

The Global actuator group now has **Clear All Actuator Errors**. Recovery disables controller input, sends Stop, releases acknowledged actuator software latches through the existing `act stop` command, and checks fresh actuator telemetry separately for each connected unit. It reports missing units and unresolved faults. It does not clear ODrive faults, reset actuator encoders, change limits, or start motion. A failed USB connection must not prevent later units receiving Stop or an explicitly requested fault-clear command. Gamepad operation requires the user to enable control again, release the controls, and select the desired actuator or pair; existing Learn mappings are retained.

The button cannot override an active RoboClaw fault, a pressed destination limit, a lost USB connection, or failed current/voltage checks. “Zero acknowledged” is not proof of physical movement or a successful retract test.

### Findings from this session

- Unit 1's ESP32 USB stream failed during the earlier session. The next GUI connection only found units 0, 2, and 3. This is not evidence of an isolated Unit 1 CAN fault.
- The old GUI aborted several global broadcasts when it reached Unit 1's failed port; arms 2 and 3 never received those commands.
- The latest controller actuator attempt was rejected before transmission because no actuator or pair was selected.
- Current stationary reads on units 0, 2 and 3 show the retract switches open. Units 2 and 3 report their extension switches pressed. Unit 0 reports BOTH switches open despite the user's fully extended description; its switch position/wiring still needs physical verification.
- All three available RoboClaws acknowledged actuator Stop; fresh diagnostics reported zero PWM and zero connected-M1 current. Unit 3's known unused-M2 current offset remains separate.
- Wheel 0 CAN has recovered and reports voltage. Earlier +500 RPM attempts caused SPINOUT; latest attempts caused VELOCITY_LIMIT_VIOLATION about 0.1 seconds after a +500 RPM request, with encoder feedback around -753 RPM. This is reported encoder motion, not independent proof of physical direction. The cause is not established, and no encoder settings were changed.
- Wheel 3's earlier CURRENT_LIMIT_VIOLATION survived some global clear attempts because Unit 1's USB failure interrupted delivery. Later logs show Wheel 3 tracking about +/-500 RPM, followed by a latched undervoltage fault on the current read. Its CAN currently responds around 48 V. Historical reading gaps align with the actuator Stop handler taking 5.7-8.8 seconds; normal CAN updates resume afterward. The exact low-level delay and undervoltage cause remain unverified.
- Direct USB at 07:29 UTC confirms Wheel 0 IDLE, is_armed=0, bus current=0 A and electrical power=0 W, while its phase-current property retains 29.786 A. This is not a continuous present 25-30 A draw. Wheel 0/3 fault causes require separate controlled investigation; this GUI change does not claim to fix them.

### Scope and limits

No motion test, firmware upload, calibration, current-limit change, or nonvolatile save was performed for this update. The requested operating settings remain 35 A / 38 A for ODrive and a maximum 6.75 A for the actuator; this revision does not raise them. Physical reversal remains untested.

The copied `units/` and persistence helpers are the pre-existing V17 source/reference package, not newly built V18 firmware. `FIRMWARE_MAP.json` describes that prior deployment. Do not interpret the GUI revision number as an ESP32 firmware update.

Current diagnostics are recorded under the preserved V17 `evidence/all-recovery-status-20260922T065347Z/` and `evidence/actuator-stop-readback-20260922T065726Z/` directories. Historical GUI evidence is in its `logs/` directory, sessions `20260922T064702Z-5963-8193c6c2` and `20260922T065008Z-6020-b5a3a736`.

Latest read-only evidence after resuming: preserved V17 `evidence/all-recovery-status-20260922T072738Z/`, `evidence/usb-arm0-20260922T072924Z/` and `evidence/usb-comparison-20260922T072925Z/`. Latest actuator attempts in sessions `072113Z` and `072347Z` still contain no dispatched actuator move: the GUI reports no actuator selected. Arm 1 remains absent after the user reported its damaged ESP32 USB-C connector.

Offline verification: 158 GUI/transport/recovery tests passed, including partial serial framing, stale telemetry, Stop acknowledgment ordering, disconnected-unit delivery, cancellation, held-control release, missing targets and CLI cleanup. No physical retract test was performed.
