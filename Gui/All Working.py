#!/usr/bin/env python3
"""All Working - USB Serial supervisor for four climber ESP32 controllers.

Each ESP32 runs the bringup firmware and owns one actuator, one local wheel
ODrive, and its ToF sensor. This script runs on the computer/Jetson and sends
commands to one unit or broadcasts the same command to all units.

Terminal: python3 "All Working.py" --auto
The adjacent climber_gui5.py imports this same script. Run only one front-end
at a time. Motion routines and pin assignments live in each ESP's firmware.
"""

from __future__ import annotations

import argparse
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing pyserial. Install it with: python3 -m pip install pyserial", file=sys.stderr)
    raise


BAUD_DEFAULT = 115200
IDENTITY_RES = (
    re.compile(r"CLIMBER_UNIT_ID=(\d+)\s+LOCAL_ODRIVE_NODE_ID=(\d+)"),
    re.compile(r"unit id:\s*(\d+),\s*local ODrive CAN node id:\s*(\d+)", re.IGNORECASE),
)


@dataclass
class Unit:
    index: int
    port: str
    serial_port: serial.Serial
    stop_reader: threading.Event
    reader_thread: threading.Thread
    identified: bool = True
    odrive_node_id: int | None = None

    @property
    def label(self) -> str:
        return f"u{self.index}"

    def send(self, command: str, *, flush: bool = True) -> None:
        line = command.strip()
        if not line:
            return
        if self.stop_reader.is_set():
            raise OSError(f"{self.label} USB reader disconnected; reconnect this unit")
        payload = (line + "\n").encode("utf-8")
        if self.serial_port.write(payload) != len(payload):
            raise OSError(f"{self.label} incomplete USB write")
        if flush:
            self.serial_port.flush()

    def close(self) -> None:
        self.stop_reader.set()
        self.reader_thread.join(timeout=1.0)
        self.serial_port.close()


def reader_loop(unit_index: int, ser: serial.Serial, stop_event: threading.Event, output: queue.Queue[str], line_handler=None) -> None:
    pending = bytearray()
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except (serial.SerialException, OSError) as exc:
            stop_event.set()
            output.put(f"[u{unit_index}] serial read error: {exc}")
            return
        if raw:
            pending.extend(raw)
            while b"\n" in pending:
                line, _, remaining = pending.partition(b"\n")
                pending = bytearray(remaining)
                text = line.decode("utf-8", errors="replace").rstrip()
                if not text:
                    continue
                if line_handler is not None and line_handler(unit_index, text):
                    continue
                output.put(f"[u{unit_index}] {text}")
            if len(pending) > 8192:
                pending.clear()
                output.put(f"[u{unit_index}] Oversized serial line discarded.")


def printer_loop(output: queue.Queue[str], stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            print(output.get(timeout=0.1), flush=True)
        except queue.Empty:
            continue


def list_serial_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(f"{port.device}\t{port.description}")


def likely_usb_serial_ports() -> list[str]:
    ports: list[str] = []
    for port in list_ports.comports():
        device = port.device
        description = getattr(port, "description", "") or ""
        manufacturer = getattr(port, "manufacturer", "") or ""
        text = f"{device} {description} {manufacturer}".lower()
        if "odrive" in text or (getattr(port, "vid", None), getattr(port, "pid", None)) == (0x1209, 0x0D32):
            continue
        if any(marker in text for marker in ("usbmodem", "usbserial", "ttyacm", "ttyusb", "arduino", "esp32")):
            ports.append(device)
    return ports


def discover_identity(ser: serial.Serial) -> tuple[int | None, int | None]:
    try:
        ser.reset_input_buffer()
    except serial.SerialException:
        pass

    deadline = time.time() + 10.0
    next_request = 0.0

    while time.time() < deadline:
        now = time.time()
        if now >= next_request:
            try:
                ser.write(b"id\nunit\n")
                ser.flush()
            except serial.SerialException:
                return None, None
            next_request = now + 0.75

        try:
            raw = ser.readline()
        except serial.SerialException:
            return None, None

        if not raw:
            continue
        text = raw.decode("utf-8", errors="replace").strip()
        for pattern in IDENTITY_RES:
            match = pattern.search(text)
            if match:
                return int(match.group(1)), int(match.group(2))

    return None, None


def open_units(
    ports: list[str],
    baud: int,
    output: queue.Queue[str],
    discover: bool = True,
    allow_unknown: bool = False,
    line_handler=None,
) -> list[Unit]:
    units: list[Unit] = []
    seen_ids: set[int] = set()

    for fallback_index, port in enumerate(ports):
        try:
            ser = serial.Serial(port=port, baudrate=baud, timeout=0.05, write_timeout=1.0)
        except serial.SerialException as exc:
            print(f"Could not open {port}: {exc}")
            continue

        unit_id: int | None = None
        odrive_node_id: int | None = None

        if discover:
            time.sleep(2.0)
            unit_id, odrive_node_id = discover_identity(ser)

        index = unit_id if unit_id is not None else fallback_index
        if unit_id is None and discover and not allow_unknown:
            ser.close()
            print(
                f"Skipped {port}: no firmware id response. "
                "Check USB or reset the unresponsive ESP32 while supported/stationary, then reconnect. Do not bypass unit identification."
            )
            continue

        if index in seen_ids:
            ser.close()
            raise RuntimeError(
                f"Duplicate unit id {index} detected. Re-flash one ESP32 with a different bringup_unitN target."
            )
        seen_ids.add(index)

        stop_reader = threading.Event()
        thread = threading.Thread(
            target=reader_loop,
            args=(index, ser, stop_reader, output, line_handler),
            daemon=True,
        )
        thread.start()
        units.append(Unit(index, port, ser, stop_reader, thread, identified=unit_id is not None, odrive_node_id=odrive_node_id))
        if unit_id is None:
            print(f"Opened {port} as u{index} by port order; no firmware id response.")
        else:
            print(f"Opened {port} as u{index}; local ODrive node {odrive_node_id}.")

    return sorted(units, key=lambda unit: unit.index)


def print_help() -> None:
    print(
        """
Supervisor commands:
  help                         show this help
  ports                        list detected serial ports
  unit                         ask all units to print their saved unit id
  status                       ask all units for status
  limits                       watch limit switches on all units for 20 seconds
  start                        run startup on all units at the same time
  start 1 | start1 | u1 start  run startup on one unit only
  stop | s | estop             stop all units

Wheel commands:
  can init                     initialize CAN on all connected units
  can status                   print ESP32 CAN controller error counters
  can reset                    reset ESP32 CAN after BUS_OFF or wiring changes
  can beacon                   send CAN frames to wake ODrive auto-baud
  can watch                    print raw CAN frames seen by each unit for 5 seconds
  wheel diag                   ODrive diagnostics on all units
  wheel clear                  clear ODrive errors on all units
  wheel idle                   stop wheels and set ODrives to IDLE
  wheel monitor                print actual RPM/current for 5 seconds
  wheel turn 1                 move each local wheel exactly +1 encoder turn
  wheel test                   all local wheels jog at +5 RPM for 2 seconds
  wheel test 10rpm             all local wheels jog at +10 RPM for 2 seconds
  wheel tune                   read ODrive tuning, bus limits, and encoder status
  wheel tune apply             apply climbing preset + 35/50 A in RAM
  wheel tune set P V           set one tuning/bus parameter P to V in RAM
  wheel tune save              save verified 35/50 A & tuning to ODrive flash
  wheel cal                    run guarded encoder offset calibration (state 7)
  all 10rpm                    all local wheels forward at 10 RPM
  all -10                      all local wheels reverse at 10 RPM
  all 0                        stop all local wheels
  w0 diag                      unit 0 ODrive diagnostics
  w0 clear                     clear unit 0 ODrive errors
  w0 monitor                   unit 0 actual RPM/current for 5 seconds
  w0 turn 1                    unit 0 exact +1 encoder turn test
  w0 test                      unit 0 wheel jog at +5 RPM for 2 seconds
  w1 tune                      unit 1 ODrive tuning & bus config
  w1 tune set vel_gain 0.5     set one tuning parameter on unit 1
  w1 cal                       calibrate unit 1 wheel encoder offset
  w0 10rpm                     unit 0 local wheel at 10 RPM
  w1 -5                        unit 1 local wheel reverse at 5 RPM
  w2 0                         stop unit 2 local wheel

Actuator commands:
  act diag                     ask all units for RoboClaw diagnostics
  act enc                      ask all units for RoboClaw encoder counts/speeds
  act enc watch                watch RoboClaw encoder readback
  act enc reset                reset RoboClaw encoder counters
  act reset                    reset RoboClaw serial link and send stop commands
  act safety                   set RoboClaw serial deadman timeout
  act scan                     scan common RoboClaw serial settings
  act use m1                   set actuator commands to RoboClaw M1
  act use m2                   set actuator commands to RoboClaw M2
  act pulse fwd                pulse all actuators forward for 1 second
  act pulse rev                pulse all actuators reverse for 1 second
  act pulse m1 fwd             pulse RoboClaw M1 forward for 1 second
  act pulse m2 fwd             pulse RoboClaw M2 forward for 1 second
  act fwd                      all actuators extend until their FWD limit
  act rev                      all actuators retract until their REV limit
  act center                   all actuators find limits and center
  act stop                     stop all actuators
  act0 fwd                     unit 0 actuator only
  act1 center                  unit 1 actuator only

Raw routing:
  u0 status                    send a raw command to unit 0
  u1 limits                    watch limit switches on unit 1 only
  u2 tof read                  send a raw command to unit 2
  broadcast tof read           send a raw command to all units
  quit                         close serial ports and exit
""".strip()
    )


def send_all(units: list[Unit], command: str) -> None:
    for unit in units:
        unit.send(command)


def send_one(units: list[Unit], index: int, command: str) -> None:
    unit = next((candidate for candidate in units if candidate.index == index), None)
    if unit is None:
        raise ValueError("No unit " + str(index) + ". Opened units are: " + ", ".join(unit.label for unit in units))
    unit.send(command)


def parse_unit_prefix(token: str) -> int | None:
    token = token.lower()
    if len(token) >= 2 and token[0] in {"u", "w"} and token[1:].isdigit():
        return int(token[1:])
    if token.startswith("act") and token[3:].isdigit():
        return int(token[3:])
    return None


def parse_unit_selector(token: str) -> int | None:
    token = token.lower()
    if token.isdigit():
        return int(token)
    if len(token) >= 2 and token[0] == "u" and token[1:].isdigit():
        return int(token[1:])
    return None


def handle_supervisor_command(units: list[Unit], line: str) -> bool:
    command = line.strip()
    if not command:
        return True

    lower = command.lower()
    parts = lower.split()
    head = parts[0]

    if lower in {"quit", "exit"}:
        return False
    if lower in {"help", "?"}:
        print_help()
        return True
    if lower == "ports":
        list_serial_ports()
        return True
    if head == "start" and len(parts) >= 2:
        if parts[1] == "all":
            send_all(units, "start")
            return True
        unit_index = parse_unit_selector(parts[1])
        if unit_index is not None:
            send_one(units, unit_index, "start")
            return True
    if head.startswith("start") and head[5:].isdigit() and len(parts) == 1:
        send_one(units, int(head[5:]), "start")
        return True
    if lower in {"start", "unit", "status", "limits", "stop", "s", "estop"}:
        send_all(units, "stop" if lower in {"s", "estop"} else lower)
        return True
    if lower.startswith("can "):
        send_all(units, lower)
        return True
    if lower.startswith("tof "):
        send_all(units, lower)
        return True
    if head == "wheel" and len(parts) >= 2:
        send_all(units, "wheel " + " ".join(parts[1:]))
        return True

    if head in {"all", "wall"} and len(parts) >= 2:
        send_all(units, f"wheel {parts[1]}")
        return True

    if head.startswith("w") and head[1:].isdigit() and len(parts) >= 2:
        send_one(units, int(head[1:]), "wheel " + " ".join(parts[1:]))
        return True

    if head == "act":
        if len(parts) == 2:
            send_all(units, f"act {parts[1]}")
            return True
        if len(parts) >= 3 and parts[1] == "all":
            send_all(units, "act " + " ".join(parts[2:]))
            return True
        if len(parts) >= 3 and parts[1].isdigit():
            send_one(units, int(parts[1]), "act " + " ".join(parts[2:]))
            return True
        if len(parts) >= 3:
            send_all(units, "act " + " ".join(parts[1:]))
            return True

    unit_index = parse_unit_prefix(head)
    if unit_index is not None and len(parts) >= 2:
        if head.startswith("act"):
            send_one(units, unit_index, "act " + " ".join(parts[1:]))
        elif head.startswith("w"):
            send_one(units, unit_index, "wheel " + " ".join(parts[1:]))
        else:
            send_one(units, unit_index, " ".join(parts[1:]))
        return True

    if head == "broadcast" and len(parts) >= 2:
        send_all(units, command.split(maxsplit=1)[1])
        return True

    print("Unknown supervisor command. Type help.")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Control four climber ESP32 units over USB Serial.")
    parser.add_argument("--ports", nargs="+", help="Serial ports in any order; unit ids are discovered from firmware")
    parser.add_argument("--auto", action="store_true", help="Use all likely Arduino/ESP32 USB serial ports")
    parser.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    parser.add_argument("--no-discover", action="store_true", help="Use port order instead of asking firmware for unit ids")
    parser.add_argument("--allow-unknown", action="store_true", help="Allow unidentified boards to be controlled by port order")
    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return 0

    ports = args.ports
    if args.auto:
        ports = likely_usb_serial_ports()
    if not ports:
        parser.error("provide --ports, use --auto, or use --list to find ports")

    output: queue.Queue[str] = queue.Queue()
    printer_stop = threading.Event()
    printer = threading.Thread(target=printer_loop, args=(output, printer_stop), daemon=True)
    printer.start()

    units = []
    try:
        units = open_units(ports, args.baud, output, discover=not args.no_discover, allow_unknown=args.allow_unknown)
        if not units:
            print("No climber units opened.")
            print("Available serial ports right now:")
            list_serial_ports()
            return 1
        print(f"Ready with {len(units)} unit(s): " + ", ".join(f"u{unit.index}={unit.port}" for unit in units))
        print_help()
        while True:
            try:
                line = input("climber> ")
            except EOFError:
                break
            try:
                if not handle_supervisor_command(units, line):
                    break
            except (ValueError, OSError, serial.SerialException) as exc:
                print(f"Command error: {exc}")
    finally:
        try:
            # Attempt every Stop before waiting for reader threads to close.
            for unit in units:
                try:
                    unit.send("stop")
                except Exception as exc:
                    print(f"[{unit.label}] Stop delivery failed: {exc}")
            for unit in units:
                try:
                    unit.close()
                except Exception as exc:
                    print(f"[{unit.label}] Close failed: {exc}")
        finally:
            printer_stop.set()
            printer.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
