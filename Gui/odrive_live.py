"""Read-only live ODrive data model. This module never opens or writes a port."""
from __future__ import annotations
from dataclasses import dataclass
import math
import threading
import time

DISPLAY_MS = 200
STALE_SECONDS = 0.6
STATES = {1: 'IDLE', 2: 'STARTUP', 3: 'CALIBRATING', 4: 'MOTOR CAL', 7: 'ENCODER CAL', 8: 'CLOSED LOOP'}
LINKS = {0: 'CAN not initialized', 1: 'RUNNING', 2: 'CAN stopped', 3: 'CAN BUS_OFF', 4: 'CAN recovering', 5: 'CAN unavailable'}
ERROR_NAMES = (
    (0x04000000, 'SPINOUT_DETECTED'),
    (0x00001000, 'CURRENT_LIMIT_VIOLATION'),
    (0x00008000, 'VELOCITY_LIMIT_VIOLATION'),
    (0x00000200, 'DC_BUS_UNDER_VOLTAGE'),
    (0x00000100, 'DC_BUS_OVER_VOLTAGE'),
    (0x00000400, 'DC_BUS_OVER_CURRENT'),
    (0x00000800, 'DC_BUS_OVER_REGEN_CURRENT'),
    (0x00002000, 'MOTOR_OVER_TEMP'),
    (0x00004000, 'INVERTER_OVER_TEMP'),
    (0x40000000, 'CALIBRATION_ERROR'),
    (0x02000000, 'ESTOP_REQUESTED'),
    (0x01000000, 'WATCHDOG_TIMER_EXPIRED'),
    (0x08000000, 'BRAKE_RESISTOR_DISARMED'),
    (0x00000020, 'DRV_FAULT'),
    (0x00000010, 'BAD_CONFIG'),
    (0x00000008, 'MISSING_ESTIMATE'),
    (0x00000001, 'INITIALIZING'),
)


def format_error(error: int) -> str:
    """Format an ODrive Active_Errors bitmask with known fault names and hex code."""
    names = [name for mask, name in ERROR_NAMES if error & mask]
    if names:
        return f"{', '.join(names)} · 0x{error:08X}"
    return f'Error 0x{error:08X}'

@dataclass(frozen=True)
class Sample:
    unit: int
    sequence: int
    link: int
    state: int
    error: int
    ages: tuple[int, int, int, int]
    voltage: float
    supply_current: float
    iq_target: float
    iq_measured: float
    rpm: float
    received: float


def parse_sample(text: str, unit: int, now: float) -> Sample | None:
    # A prompt may precede the first streamed record on a serial line.
    marker = text.find('ODRIVE_LIVE,')
    if marker < 0:
        return None
    parts = text[marker:].strip().split(',')
    if len(parts) != 16 or parts[0:2] != ['ODRIVE_LIVE', '1']:
        return None
    try:
        board, seq, link, state, error = map(int, parts[2:7])
        ages = tuple(map(int, parts[7:11]))
        values = tuple(map(float, parts[11:16]))
    except (ValueError, OverflowError):
        return None
    if board != unit or not 0 <= board < 4 or not 0 <= seq <= 0xffffffff:
        return None
    if link not in LINKS or not -1 <= state <= 255 or not 0 <= error <= 0xffffffff:
        return None
    if any(age < -1 or age > 0xffffffff for age in ages) or any(math.isinf(v) for v in values):
        return None
    return Sample(board, seq, link, state, error, ages, *values, now)


class TelemetryStore:
    """Latest sample per arm, timestamped by the existing serial reader thread."""
    def __init__(self):
        self._lock = threading.Lock()
        self._samples: dict[int, Sample] = {}
        self._parts: dict[int, dict] = {}

    def clear(self):
        with self._lock:
            self._samples.clear()
            self._parts.clear()

    def feed(self, unit: int, text: str) -> bool:
        if 'ODRIVE_PART,' in text:
            self._feed_part(unit, text, time.monotonic())
            return True
        if 'ODRIVE_LIVE,' not in text:
            return False
        sample = parse_sample(text, unit, time.monotonic())
        if sample is not None:
            with self._lock:
                self._parts.pop(unit, None)
                self._samples[unit] = sample
        return True # Even malformed telemetry must not flood the command console.

    def _feed_part(self, unit: int, text: str, now: float) -> None:
        # Every USB piece is a complete short line. Console replies can occur
        # between pieces; publish only a complete, contiguous snapshot.
        parts = text[text.find('ODRIVE_PART,'):].strip().split(',', 7)
        try:
            if len(parts) != 8 or parts[:2] != ['ODRIVE_PART', '1']:
                return
            board, seq, index, total, age_ms = map(int, parts[2:7])
            fragment = parts[7]
            if (board != unit or not 0 <= board < 4 or not 0 <= seq <= 0xffffffff
                    or not 1 <= total <= 11 or not 0 <= index < total
                    or not 0 <= age_ms < 200 or not 1 <= len(fragment) <= 24
                    or (index < total - 1 and len(fragment) != 24)):
                return
        except (ValueError, OverflowError):
            return
        with self._lock:
            if index == 0:
                self._parts[unit] = {'seq': seq, 'total': total, 'fragments': [],
                                     'first_received': now, 'age_ms': 0}
            pending = self._parts.get(unit)
            if pending is None:
                return
            if (pending['seq'] != seq or pending['total'] != total
                    or len(pending['fragments']) != index
                    or now - pending['first_received'] > STALE_SECONDS
                    or age_ms < pending['age_ms']):
                self._parts.pop(unit, None)
                return
            pending['fragments'].append(fragment)
            pending['age_ms'] = age_ms
            if index != total - 1:
                return
            self._parts.pop(unit, None)
            # Include time spent waiting/transmitting on the ESP in data age.
            sample = parse_sample(''.join(pending['fragments']), unit, now - age_ms / 1000)
            if sample is not None and sample.sequence == seq:
                self._samples[unit] = sample

    def rows(self, connected: set[int], now: float | None = None):
        now = time.monotonic() if now is None else now
        with self._lock:
            samples = dict(self._samples)
        return {unit: display_row(samples.get(unit), unit in connected, now) for unit in range(4)}


def display_row(sample: Sample | None, connected: bool, now: float):
    # V, supply A, measured motor Iq A, target Iq A, RPM, state, data status.
    empty = ('—',) * 6
    if not connected:
        return (*empty, 'Disconnected')
    if sample is None:
        return (*empty, 'Waiting for live data')
    age = max(0.0, now - sample.received)
    if age > STALE_SECONDS:
        return (*empty, f'Stale · {age:.1f}s')
    if sample.link != 1:
        return (*empty, LINKS[sample.link])
    hb_age = sample.ages[0]
    if hb_age < 0 or hb_age + age * 1000 >= 1000 or sample.state < 0:
        return (*empty, 'No fresh heartbeat')

    def value(number, group):
        source_age = sample.ages[group]
        if not math.isfinite(number) or source_age < 0 or source_age + age * 1000 > 600:
            return '—'
        return f'{number:.2f}'

    motor_live = sample.state == 8
    voltage, current = value(sample.voltage, 1), value(sample.supply_current, 1)
    measured = value(sample.iq_measured, 2) if motor_live else '—'
    target = value(sample.iq_target, 2) if motor_live else '—'
    rpm = value(sample.rpm, 3)
    state = STATES.get(sample.state, f'State {sample.state}')
    status = f'Live · {age:.1f}s'
    if sample.error:
        status = format_error(sample.error)
    elif '—' in (voltage, current, rpm) or (motor_live and '—' in (measured, target)):
        status = 'Waiting for fresh CAN data'
    return voltage, current, measured, target, rpm, state, status
