"""Host C++ checks for unified bringup.cpp stop behavior, unit_config.h, and RoboClaw v4.4.8 RX sync."""

from __future__ import annotations

from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parent.parent
FIRMWARE_DIR = ROOT / "firmware" / "climber_mcu"
SRC_DIR = FIRMWARE_DIR / "src"
ROBOCLAW_DIR = FIRMWARE_DIR / "vendor" / "roboclaw_v448"


def _run(cmd: list[str]) -> str:
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"Command failed ({proc.returncode}): {' '.join(cmd)}\n{proc.stdout}\n{proc.stderr}")
    return proc.stdout


def test_open_loop_stop_all_units() -> str:
    """Compile actual actuatorDriveChannelOpenLoop and actuatorStop for UNIT_ID 0..3."""
    source = (SRC_DIR / "bringup.cpp").read_text()
    drive = source.split("bool actuatorDriveChannelOpenLoop(uint8_t channel, int power) {", 1)[1]
    drive = "bool actuatorDriveChannelOpenLoop(uint8_t channel, int power) {" + drive.split(
        "\nbool actuatorManualDrivePermitted(int power)", 1
    )[0]
    stop = source.split("void actuatorStop() {", 1)[1]
    stop = "void actuatorStop() {" + stop.split("\nvoid resetRoboClawSerialLink", 1)[0]

    prefix = r"""
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>
struct Call { int channel, mode, power; };
struct Fake {
    std::vector<Call> calls;
    bool fwd = true, rev = true;
    int clear_calls = 0;
    bool ForwardM1(int, int p) { calls.push_back({1, 1, p}); return fwd; }
    bool BackwardM1(int, int p) { calls.push_back({1, -1, p}); return rev; }
    bool ForwardM2(int, int p) { calls.push_back({2, 1, p}); return fwd; }
    bool BackwardM2(int, int p) { calls.push_back({2, -1, p}); return rev; }
    bool SpeedM1(int, int p) { calls.push_back({1, 2, p}); return true; }
    bool SpeedM2(int, int p) { calls.push_back({2, 2, p}); return true; }
    void clear() { ++clear_calls; }
} roboclaw;
struct FakeGuard { bool failed() const { return false; } } actuator_current_guard;
struct FakeTelemetry {
    bool drain_required = false;
    bool last_stop_ok = false;
    void stopResult(bool ok) { last_stop_ok = ok; }
} actuator_telemetry;
int roboclaw_addr = 128;
void delay(uint32_t) {}
uint8_t clampActuatorPower(int p) { return std::min(std::abs(p), 127); }
bool ensureActuatorVoltageCeiling() { return true; }
bool ensureActuatorCurrentCeiling(uint8_t) { return true; }
"""
    adapter = """
bool actuatorDriveChannelOpenLoopWithRetry(uint8_t c, int p) {
    return actuatorDriveChannelOpenLoop(c, p);
}
"""
    suffix = r"""
int main() {
    int scenarios = 0;
    for (int channel : {1, 2}) {
        for (int direction : {-40, 40}) {
            roboclaw.calls.clear();
            roboclaw.fwd = roboclaw.rev = true;
            assert(actuatorDriveChannelOpenLoop(channel, direction));
            assert(roboclaw.calls.size() == 1);
            assert(roboclaw.calls[0].power == 40);
            assert(roboclaw.calls[0].mode == (direction > 0 ? 1 : -1));

            roboclaw.calls.clear();
            assert(actuatorDriveChannelOpenLoop(channel, 0));
            // Unified open-loop stop across all 4 units: never engage SpeedM*(0).
            assert(roboclaw.calls.size() == 2);
            for (auto c : roboclaw.calls) {
                assert(c.channel == channel && c.power == 0);
                assert(c.mode != 2);
            }
            ++scenarios;
        }
    }
    for (int channel : {1, 2}) {
        for (bool f : {false, true}) {
            for (bool r : {false, true}) {
                roboclaw.fwd = f;
                roboclaw.rev = r;
                assert(actuatorDriveChannelOpenLoop(channel, 0) == (f || r));
                ++scenarios;
            }
        }
    }
    roboclaw.calls.clear();
    roboclaw.fwd = roboclaw.rev = true;
    actuatorStop();
    assert(roboclaw.calls.size() == 4);
    bool seen1 = false, seen2 = false;
    for (auto c : roboclaw.calls) {
        assert(c.power == 0);
        assert(c.mode != 2);
        seen1 |= (c.channel == 1);
        seen2 |= (c.channel == 2);
    }
    assert(seen1 && seen2);
    std::cout << "open-loop stop unit " << UNIT_ID << ": " << scenarios + 1 << " scenarios passed\n";
    return 0;
}
"""
    outputs = []
    with tempfile.TemporaryDirectory() as tmpdir:
        cpp = Path(tmpdir) / "stop.cpp"
        cpp.write_text(prefix + drive + adapter + stop + suffix)
        for unit in (0, 1, 2, 3):
            binary = Path(tmpdir) / f"stop_{unit}"
            _run([
                "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", f"-DUNIT_ID={unit}", str(cpp), "-o", str(binary),
            ])
            outputs.append(_run([str(binary)]).strip())
    return "\n".join(outputs) + "\n"


def test_unit_config_matrix() -> str:
    """Verify unit_config.h pins, stroke floor, traits, and bench toggle for units 0..3."""
    cpp_code = r"""
#include <cassert>
#include <cstring>
#include <iostream>

#define D8 8
#define D9 9
#define D10 10

#include "unit_config.h"

int main() {
    assert(LOCAL_ODRIVE_NODE_ID == UNIT_ID);
    if (UNIT_ID == 1) {
        assert(PIN_CAN_TX == D9);
        assert(PIN_CAN_RX == D10);
        assert(std::strcmp(UNIT_CAN_PIN_LABEL, "TX=D9 RX=D10") == 0);
        assert(UNIT_HOMING_CONFIRM_AND_CREEP == true);
    } else {
        assert(PIN_CAN_TX == D8);
        assert(PIN_CAN_RX == D9);
        assert(std::strcmp(UNIT_CAN_PIN_LABEL, "TX=D8 RX=D9") == 0);
        assert(UNIT_HOMING_CONFIRM_AND_CREEP == false);
    }
    if (UNIT_ID == 2) {
        assert(ACTUATOR_MIN_ENCODER_RANGE_TICKS_CONFIG == 1330);
    } else {
        assert(ACTUATOR_MIN_ENCODER_RANGE_TICKS_CONFIG == 1350);
    }
    const bool expect_unit0_center = (UNIT_ID == 0);
    assert(UNIT_CENTER_DISABLE_SLOWDOWN == expect_unit0_center);
    assert(UNIT_CENTER_COARSE_WINDOW == expect_unit0_center);
    assert(UNIT_CENTER_ABORT_ON_STALL == expect_unit0_center);
    assert(UNIT_CENTER_VERIFY_SETTLED == expect_unit0_center);
    assert(ENABLE_BENCH_DIAGNOSTICS == EXPECT_BENCH);
    std::cout << "unit_config unit " << UNIT_ID << " bench=" << ENABLE_BENCH_DIAGNOSTICS << ": passed\n";
    return 0;
}
"""
    outputs = []
    with tempfile.TemporaryDirectory() as tmpdir:
        cpp = Path(tmpdir) / "test_unit_config.cpp"
        cpp.write_text(cpp_code)
        for unit in (0, 1, 2, 3):
            for bench in (0, 1):
                binary = Path(tmpdir) / f"cfg_{unit}_{bench}"
                _run([
                    "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                    f"-I{SRC_DIR}",
                    f"-DUNIT_ID={unit}",
                    f"-DENABLE_BENCH_DIAGNOSTICS={bench}",
                    f"-DEXPECT_BENCH={bench}",
                    str(cpp), "-o", str(binary),
                ])
                outputs.append(_run([str(binary)]).strip())
    return "\n".join(outputs) + "\n"


def test_roboclaw_v448_rx_sync_and_replies() -> str:
    """Compile vendor/roboclaw_v448/RoboClaw.cpp against a synthetic ESP32-semantics UART."""
    arduino_h = r"""
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdarg>
#include <cstring>
inline uint32_t g_fake_us = 0;
inline uint32_t micros() { g_fake_us += 100; return g_fake_us; }
inline uint32_t millis() { return g_fake_us / 1000; }
inline void delay(uint32_t ms) { g_fake_us += ms * 1000; }
inline void delayMicroseconds(uint32_t us) { g_fake_us += us; }
"""
    stream_h = r"""
#pragma once
#include <cstddef>
#include <cstdint>
class Stream {
public:
    virtual ~Stream() = default;
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual size_t write(uint8_t) = 0;
    virtual void flush() = 0;
};
"""
    hwserial_h = r"""
#pragma once
#include "Stream.h"
#include <deque>
#include <vector>
class HardwareSerial : public Stream {
public:
    std::deque<uint8_t> rx;
    std::vector<uint8_t> tx;
    int begin_calls = 0;
    int flush_calls = 0;
    void begin(long) { ++begin_calls; }
    int available() override { return static_cast<int>(rx.size()); }
    int read() override {
        if (rx.empty()) return -1;
        uint8_t b = rx.front();
        rx.pop_front();
        return b;
    }
    int peek() override { return rx.empty() ? -1 : rx.front(); }
    size_t write(uint8_t b) override { tx.push_back(b); return 1; }
    // Match ESP32 HardwareSerial::flush(): waits for TX, does NOT drain RX.
    void flush() override { ++flush_calls; }
};
"""
    test_cpp = r"""
#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>
#include "HardwareSerial.h"
#include "RoboClaw.h"

namespace {
uint16_t crc16(const std::vector<uint8_t>& bytes) {
    uint16_t crc = 0;
    for (uint8_t b : bytes) {
        crc ^= static_cast<uint16_t>(b) << 8;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

void push_reply(HardwareSerial& uart, uint8_t addr, uint8_t cmd, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> full{addr, cmd};
    full.insert(full.end(), payload.begin(), payload.end());
    uint16_t c = crc16(full);
    for (uint8_t b : payload) uart.rx.push_back(b);
    uart.rx.push_back(static_cast<uint8_t>(c >> 8));
    uart.rx.push_back(static_cast<uint8_t>(c & 0xFF));
}
}  // namespace

int main() {
    HardwareSerial uart;
    RoboClaw rc(&uart, 10000);

    // 1. Stale 0xFF ACK in RX before write_n must be cleared, NOT accepted.
    uart.rx = {0xFF};
    assert(rc.ForwardM1(0x80, 0) == false);
    assert(uart.rx.empty());
    assert(!uart.tx.empty());

    // 2. ResponderSerial simulates RoboClaw replying only after full TX packet arrives;
    // pre-seeded RX garbage (including stale 0xFF/0x00) must be cleared before TX.
    struct ResponderSerial : HardwareSerial {
        void flush() override {
            HardwareSerial::flush();
        }
        size_t write(uint8_t b) override {
            HardwareSerial::write(b);
            if (tx.size() == 5 && tx[0] == 0x80 && tx[1] == 0) {
                rx.push_back(0xFF);
            } else if (tx.size() == 2 && tx[0] == 0x80) {
                uint8_t cmd = tx[1];
                if (cmd == 59) {
                    push_reply(*this, 0x80, 59, {0x00, 0x3C, 0x00, 0xFC, 0x07});
                } else if (cmd == 75) {
                    push_reply(*this, 0x80, 75, {0x01, 0x02, 0x03, 0x04, 0x05});
                } else if (cmd == 15) {
                    push_reply(*this, 0x80, 15, {0x02});
                }
            }
            return 1;
        }
    } responder;

    RoboClaw rc2(&responder, 10000);

    // Pre-seed garbage byte 0x11; clear() drains it before TX so post-TX 0xFF succeeds cleanly.
    responder.rx = {0x11, 0xFF};
    responder.tx.clear();
    assert(rc2.ForwardM1(0x80, 0) == true);
    assert(responder.rx.empty());

    // Seed stale RX garbage before each Read*; clear() must discard it before sending cmd.
    responder.rx = {0xFF, 0x00, 0x99};
    responder.tx.clear();
    uint16_t vmin = 0, vmax = 0;
    uint8_t auto_max = 0;
    assert(rc2.ReadMinMaxMainVoltages(0x80, vmin, vmax, auto_max));
    assert(vmin == 60 && vmax == 252 && auto_max == 7);

    responder.rx = {0xDE, 0xAD};
    responder.tx.clear();
    uint8_t s3 = 0, s4 = 0, s5 = 0, c1 = 0, c2 = 0;
    assert(rc2.GetPinFunctions(0x80, s3, s4, s5, c1, c2));
    assert(s3 == 1 && s4 == 2 && s5 == 3 && c1 == 4 && c2 == 5);

    responder.rx = {0x42};
    responder.tx.clear();
    uint8_t timeout_tenths = 0;
    assert(rc2.ReadSerialTimeout(0x80, timeout_tenths));
    assert(timeout_tenths == 2);

    std::cout << "roboclaw_v448 synthetic UART: 4 checks passed\n";
    return 0;
}
"""
    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        (tmp / "Arduino.h").write_text(arduino_h)
        (tmp / "Stream.h").write_text(stream_h)
        (tmp / "HardwareSerial.h").write_text(hwserial_h)
        (tmp / "test_roboclaw_v448.cpp").write_text(test_cpp)

        rc_obj = tmp / "RoboClaw.o"
        test_obj = tmp / "test_roboclaw_v448.o"
        binary = tmp / "test_roboclaw_v448"

        # Compile vendored RoboClaw.cpp separately without -Werror (upstream non-AVR peek/read/write),
        # then compile our test strictly with -Wall -Wextra -Werror + ASan/UBSan.
        _run([
            "g++", "-std=c++17", "-fsanitize=address,undefined",
            f"-I{tmp}", f"-I{ROBOCLAW_DIR}",
            "-c", str(ROBOCLAW_DIR / "RoboClaw.cpp"), "-o", str(rc_obj),
        ])
        _run([
            "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined",
            f"-I{tmp}", f"-I{ROBOCLAW_DIR}",
            "-c", str(tmp / "test_roboclaw_v448.cpp"), "-o", str(test_obj),
        ])
        _run([
            "g++", "-fsanitize=address,undefined", str(test_obj), str(rc_obj), "-o", str(binary),
        ])
        return _run([str(binary)])


def run_all() -> list[dict]:
    results = []
    for name, fn in (
        ("open_loop_stop_all_units", test_open_loop_stop_all_units),
        ("unit_config_matrix", test_unit_config_matrix),
        ("roboclaw_v448_rx_sync_and_replies", test_roboclaw_v448_rx_sync_and_replies),
    ):
        output = fn()
        print(output, end="", flush=True)
        results.append({"test": name, "returncode": 0, "output": output, "stderr": ""})
    return results


if __name__ == "__main__":
    run_all()
