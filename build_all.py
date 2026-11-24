import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


def resolve_platformio() -> tuple[str, Path | None]:
    """Locate the PlatformIO CLI executable and matching core directory."""
    candidates: list[Path] = []
    if env_bin := os.environ.get("PLATFORMIO_BIN"):
        candidates.append(Path(env_bin))
    if which_bin := shutil.which("platformio"):
        candidates.append(Path(which_bin))
    candidates.append(Path.home() / ".platformio" / "penv" / "bin" / "platformio")
    candidates.append(Path("/home/frc-orin/.platformio/penv/bin/platformio"))

    for candidate in candidates:
        if candidate.is_file():
            resolved = candidate
            core_dir: Path | None = None
            if (
                len(resolved.parts) >= 4
                and resolved.parent.name == "bin"
                and resolved.parent.parent.name == "penv"
            ):
                core_dir = resolved.parent.parent.parent
            return str(resolved), core_dir
    raise FileNotFoundError("Could not locate platformio executable")


def build_env(
    pio_bin: str,
    project_dir: Path,
    log_path: Path,
    env_vars: dict[str, str],
    env_name: str,
    unit: int,
    image_dest: Path,
) -> dict:
    """Build one PlatformIO environment sequentially and export its firmware.bin."""
    command = [pio_bin, "run", "-c", "platformio.ini", "-e", env_name]
    started = time.monotonic()
    with log_path.open("w") as log:
        result = subprocess.run(
            command,
            cwd=project_dir,
            env=env_vars,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
    elapsed = round(time.monotonic() - started, 2)
    record: dict = {
        "unit": unit,
        "env": env_name,
        "returncode": result.returncode,
        "seconds": elapsed,
        "command": command,
    }
    if result.returncode == 0:
        built_bin = project_dir / ".pio" / "build" / env_name / "firmware.bin"
        if built_bin.is_file():
            image_dest.parent.mkdir(parents=True, exist_ok=True)
            data = built_bin.read_bytes()
            image_dest.write_bytes(data)
            record["image"] = str(image_dest.relative_to(project_dir.parents[1]))
            record["image_bytes"] = len(data)
            record["sha256"] = hashlib.sha256(data).hexdigest()
        else:
            record["returncode"] = 1
            record["error"] = f"Missing built binary: {built_bin}"
    print(json.dumps(record), flush=True)
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description="Build unified Climber MCU firmware environments.")
    parser.add_argument(
        "--bench",
        action="store_true",
        help="Also build opt-in bench environments (bringup_unit0_bench, bringup_unit2_bench).",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parent
    project_dir = root / "firmware" / "climber_mcu"
    out = root / "verification" / "build"
    out.mkdir(parents=True, exist_ok=True)

    pio_bin, inferred_core = resolve_platformio()
    env_vars = dict(os.environ)
    env_vars.setdefault("PLATFORMIO_SETTING_ENABLE_TELEMETRY", "No")
    env_vars.setdefault("PLATFORMIO_SETTING_CHECK_PLATFORMIO_INTERVAL", "30")
    if "PLATFORMIO_CORE_DIR" not in env_vars and inferred_core is not None:
        env_vars["PLATFORMIO_CORE_DIR"] = str(inferred_core)

    targets: list[tuple[int, str, Path, Path]] = [
        (
            unit,
            f"bringup_unit{unit}",
            out / f"arm{unit}.log",
            root / "units" / f"arm{unit}" / "firmware_images" / f"bringup_unit{unit}_v19_unified.bin",
        )
        for unit in range(4)
    ]
    if args.bench:
        for unit in (0, 2):
            env_name = f"bringup_unit{unit}_bench"
            targets.append(
                (
                    unit,
                    env_name,
                    out / f"arm{unit}_bench.log",
                    root / "units" / f"arm{unit}" / "firmware_images" / f"{env_name}_v19_unified.bin",
                )
            )

    results = [
        build_env(pio_bin, project_dir, log_path, env_vars, env_name, unit, image_dest)
        for unit, env_name, log_path, image_dest in targets
    ]
    (out / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    return int(any(item["returncode"] for item in results))


if __name__ == "__main__":
    raise SystemExit(main())
