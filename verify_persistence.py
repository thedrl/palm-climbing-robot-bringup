from pathlib import Path
import json
import os
import subprocess
import sys

root = Path(__file__).resolve().parent
directory = root / "persistence"
src_dir = root / "firmware" / "climber_mcu" / "src"
sys.path.insert(0, str(directory))
import test_unified_bringup  # noqa: E402

canonical_headers = (
    "wheel_persistence_policy.h",
    "wheel_persistence_runtime.h",
    "wheel_tuning_policy.h",
    "wheel_tuning_runtime.h",
    "actuator_persistence.h",
    "actuator_persistence_runtime.h",
)

# Verify single-source layout: canonical headers live only in firmware/climber_mcu/src/
for name in canonical_headers:
    assert (src_dir / name).is_file(), f"Missing canonical header: {src_dir / name}"
    assert not (directory / name).exists(), f"Duplicate header still present: {directory / name}"

stray_bringup = [
    p
    for p in root.rglob("bringup.cpp")
    if (root / "archive") not in p.parents
]
assert stray_bringup == [src_dir / "bringup.cpp"], f"Unexpected bringup.cpp copies: {stray_bringup}"

for unit in range(4):
    legacy_fw = root / "units" / f"arm{unit}" / "firmware"
    assert not legacy_fw.exists(), f"Legacy unarchived firmware dir remains: {legacy_fw}"

# Verify firmware integration & GUI-to-firmware serial contract
live_runtime = (src_dir / "actuator_live_runtime.h").read_text()
assert 'command!="act current save"' in live_runtime
assert 'command!="act config"' in live_runtime

code = (src_dir / "bringup.cpp").read_text()
for required in (
    '#include "unit_config.h"',
    "2026-11-19-v19-unified",
    "saveWheelCurrentLimits(0);",
    "saveActuatorCurrentLimits();",
    "CLIMBER_UNIT_ID=%d LOCAL_ODRIVE_NODE_ID=%d",
    'command == "act config"',
    'command == "act center"',
    'command == "act stop"',
):
    assert required in code, f"Missing expected contract/integration in bringup.cpp: {required}"
assert code.count("saveWheelCurrentLimits(0);") == 2
assert code.count("saveActuatorCurrentLimits();") == 1

results = []
env = dict(os.environ, ASAN_OPTIONS="detect_leaks=0")

for name in (
    "test_wheel_persistence",
    "test_wheel_tuning",
    "test_actuator_persistence",
    "test_actuator_persistence_runtime",
):
    binary = directory / name
    command = [
        "g++",
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-fsanitize=address,undefined",
        f"-I{src_dir}",
        str(directory / f"{name}.cpp"),
        "-o",
        str(binary),
    ]
    subprocess.run(command, check=True)
    process = subprocess.run([str(binary)], env=env, capture_output=True, text=True)
    results.append(
        {
            "test": name,
            "returncode": process.returncode,
            "output": process.stdout,
            "stderr": process.stderr,
        }
    )
    print(process.stdout, end="", flush=True)
    if process.returncode:
        print(process.stderr, flush=True)
        raise SystemExit(process.returncode)

results.extend(test_unified_bringup.run_all())

ver_dir = root / "verification"
ver_dir.mkdir(parents=True, exist_ok=True)
(ver_dir / "tests.json").write_text(json.dumps(results, indent=2) + "\n")
print("All unified firmware checks and host persistence/stop/config/RoboClaw tests passed.")
