# Explicit RoboClaw current-limit persistence helper

Canonical firmware headers (`actuator_persistence*.h`, `wheel_persistence*.h`, `wheel_tuning*.h`) now live directly in `firmware/climber_mcu/src/`. Host tests in this directory compile against `-Ifirmware/climber_mcu/src`.

Copy `actuator_persistence.h` and `actuator_persistence_runtime.h` to the firmware `src` (already integrated in `firmware/climber_mcu/src`). Include the runtime file immediately before handleCommand, after actuator, CAN, guard, and UART helpers exist.

Add `act current save` to the ActuatorCommandScope read/settings exclusions (touches=false) so the helper can inspect the prior acknowledged Stop state. Keep the foreground scope. In the command handler, call `saveActuatorCurrentLimits()` and return. Do not clear faults, call ensureActuatorCurrentCeiling, or reset known_idle first. The operator/script prepares active limits using the existing explicit commands before invoking save. The helper does not change active current/voltage settings or send motion commands.

Preconditions: M1 selected; foreground scope; previous acknowledged Stop; zero command power; no manual movement; known_idle; no telemetry/current/voltage guard fault; exact firmware `USB Roboclaw 2x7A v4.4.8`; standard config exactly0x0063 (known packet serial configuration), all5 input modes zero (rejecting automatic homing/startup motion); current M1 maximum already670–675; fresh zero PWM both channels and exact zero current M1. Unused M2 current offset is disregarded, its PWM must be zero. Status permits only the existing idle warning bits 0x03000000.

Snapshot: both raw current fields on M1 and M2 (commands135/136), main voltage min/max/offset (59,5bytes), standard config (99,2), all5 input modes (75,5), serial timeout (15,1), version (21,NULLterminated+CRC). Every field must read back identically after save. This is a bounded snapshot of relevant settings, not a complete dump of every RoboClaw parameter.

Command94 is sent once with key0xE22EAB7A and CRC; address0x80 packet: `80 5e e2 2e ab 7a e4 a6`. A per-ESP-boot attempt latch prevents repeated NVM writes after ambiguous transmission/ACK. Official BasicMicro manual says this saves all configuration and resets the RoboClaw. The helper waits750ms before post-save reads, then retries reads only within a3s window. Save ACK must be0xFF as well as exact independent readback for success. Missing ACK remains unverified even if settings match. It never invokes command95 or rewrites limits to manufacture successful readback.

After any transmission, known_idle and cached safety-timeout acknowledgement are invalidated. Successful save still requires a subsequent explicit Stop before motion; regular movement paths must re-establish timeout. On failure the existing telemetry fault and drain-required flags latch. Pending USB input/USB disconnect cancel before save, or stop verification after save; pending commands are never consumed. No automatic save at startup and no automatic save retry.

Protocol source: https://downloads.basicmicro.com/docs/roboclaw_user_manual.pdf page90 (all-config save and reset), page64–65 (ACK/CRC). Current library confirms key/protocol: https://raw.githubusercontent.com/basicmicro/basicmicro_arduino/main/src/Basicmicro.cpp

Host validation commands:
```
g++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined test_actuator_persistence.cpp -o /tmp/test_actuator_persistence
g++ -std=c++11 -Wall -Wextra -Werror -fsanitize=address,undefined test_actuator_persistence_runtime.cpp -o /tmp/test_actuator_persistence_runtime
ASAN_OPTIONS=detect_leaks=0 /tmp/test_actuator_persistence
ASAN_OPTIONS=detect_leaks=0 /tmp/test_actuator_persistence_runtime
```

LeakSanitizer is disabled because it cannot operate under this execution environment's ptrace; ASan/UBSan remain enabled. Runtime tests compile the actual runtime header with fake USB/UART/global adapters. No hardware commands have been run by this implementation task.
