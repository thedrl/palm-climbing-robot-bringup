"""One RAM-only, guarded arm-0 encoder-offset calibration; default is plan only.

No hardware is opened without --run and documented current-session user authorization.
No motor R/L calibration, error clear, closed-loop request, save or retry exists.
"""
import argparse
import datetime
import json
import math
import os
import threading
import time
from pathlib import Path

SERIAL = '003CF6749599'
COMMIT = 0xb3c4cf45
PORT = Path('/dev/serial/by-id') / f'usb-ODrive_Robotics_ODrive_S1_{SERIAL}-if00'
ENDPOINTS = Path('/home/frc-orin/V17-FarmBot21SEP2026-CurrentPersistence/persistence/endpoints-0.6.11-1.json')
DEFAULT_ROOT = Path('/home/frc-orin/V18-FarmBot21SEP2026-ActuatorRecovery/evidence')
STATE = 'axis0.current_state'
ACTIVE = 'axis0.active_errors'
DISARM = 'axis0.disarm_reason'
POS = 'axis0.pos_estimate'
VEL = 'axis0.vel_estimate'
RESULT = 'axis0.procedure_result'
DIRECTION = 'axis0.config.motor.direction'
SOFT = 'axis0.config.motor.current_soft_max'
HARD = 'axis0.config.motor.current_hard_max'
WATCH = 'axis0.config.enable_watchdog'
WATCH_TIME = 'axis0.config.watchdog_timeout'
COMM = 'axis0.commutation_mapper.config.'
LOAD = 'axis0.pos_vel_mapper.config.'
MAPPER_FIELDS = ('circular', 'circular_output_range', 'scale', 'offset_valid',
                 'offset', 'approx_init_pos_valid', 'approx_init_pos',
                 'index_offset_valid', 'index_offset', 'use_index_gpio',
                 'passive_index_search', 'index_gpio', 'use_endstop')
MAPPERS = tuple(prefix + field for prefix in (COMM, LOAD) for field in MAPPER_FIELDS)
ALIGNMENT = (DIRECTION, COMM + 'scale', COMM + 'offset', COMM + 'offset_valid')
TEMP = {
    SOFT: 10.0,
    'axis0.config.motor.calibration_current': 10.0,
    'axis0.config.calibration_lockin.current': 10.0,
    'axis0.config.calib_scan_distance': 8.0,
    'axis0.config.calib_scan_vel': 2.0,
    WATCH_TIME: 0.5,
    WATCH: 1,
}
CAL_SETTINGS = (
    'axis0.config.calib_range',
    'axis0.config.calibration_lockin.ramp_time',
    'axis0.config.calibration_lockin.ramp_distance',
    'axis0.config.calibration_lockin.accel',
    'axis0.config.calibration_lockin.vel',
    'axis0.config.motor.resistance_calib_max_voltage',
)
BASE = ('serial_number', 'commit_hash', STATE, ACTIVE, DISARM, RESULT, POS, VEL,
        'vbus_voltage', 'axis0.controller.input_vel', 'axis0.controller.input_torque',
        HARD, DIRECTION, 'axis0.config.motor.pole_pairs',
        'axis0.config.motor.phase_resistance', 'axis0.config.motor.phase_inductance',
        'axis0.config.motor.phase_resistance_valid',
        'axis0.config.motor.phase_inductance_valid',
        'axis0.config.load_encoder', 'axis0.config.commutation_encoder',
        'rs485_encoder_group0.status', 'axis0.motor.effective_current_lim',
        'axis0.motor.fet_thermistor.temperature', 'axis0.motor.motor_thermistor.temperature',
        'axis0.observed_encoder_scale_factor')
SNAP = tuple(dict.fromkeys(BASE + MAPPERS + CAL_SETTINGS + tuple(TEMP)))
TRACE = (STATE, ACTIVE, DISARM, POS, VEL, 'axis0.motor.foc.Id_measured',
         'axis0.motor.foc.Iq_measured', 'vbus_voltage',
         'axis0.motor.fet_thermistor.temperature', 'axis0.motor.motor_thermistor.temperature',
         RESULT)
READ_ALLOWED = set(SNAP + TRACE)
WRITE_ALLOWED = set(TEMP) | set(ALIGNMENT) | {
    'axis0.controller.input_vel', 'axis0.controller.input_torque', 'axis0.requested_state'}
PLAN = {
    'arm': 0, 'serial': SERIAL, 'state_request': 7,
    'procedure': 'encoder offset calibration only',
    'temporary_soft_current_a': 10, 'temporary_motor_calibration_current_a': 10,
    'temporary_lockin_current_a': 10, 'hard_trip_a_unchanged': 38,
    'scan_distance_electrical_turns': 8, 'scan_speed_electrical_turns_per_second': 2,
    'nominal_scan_mechanical_rpm': 120 / 7,
    'host_deadline_seconds': 20, 'watchdog_seconds': 0.5,
    'abort_abs_rpm': 30, 'abort_motor_current_vector_a': 12,
    'abort_abs_displacement_turns': 1.6, 'bus_voltage_window': [40, 55],
    'abort_temperature_c': 60, 'maximum_trace_seconds': 0.2,
    'retries': 0, 'clear_errors': False, 'save_configuration': False,
    'success': 'keep new alignment in RAM only; restore temporary parameters after verified IDLE and standstill',
    'failure': 'request zero/IDLE; restore previous direction/scale/offset/valid only after verified standstill; ambiguous serial fails closed',
}


def require(ok, message):
    if not ok:
        raise RuntimeError(message)


def equal(a, b):
    return (math.isnan(a) and math.isnan(b)) or math.isclose(a, b, rel_tol=1e-5, abs_tol=1e-5)


def guard_value(key, value, baseline):
    require(math.isfinite(value), 'Invalid feedback: ' + key)
    if key == STATE:
        require(value in (1, 7), 'Unexpected axis state')
    elif key in (ACTIVE, DISARM):
        require(value == 0, 'ODrive fault: ' + key)
    elif key == POS:
        require(abs(value - baseline) <= 1.6, 'Displacement threshold')
    elif key == VEL:
        require(abs(value * 60) <= 30, 'Speed threshold')
    elif key in ('axis0.motor.foc.Id_measured', 'axis0.motor.foc.Iq_measured'):
        require(abs(value) <= 12, 'Current component threshold')
    elif key == 'vbus_voltage':
        require(40 <= value <= 55, 'Bus voltage threshold')
    elif key.endswith('.temperature'):
        require(-20 <= value <= 60, 'Temperature threshold')


def validate_row(row):
    require(math.hypot(row['axis0.motor.foc.Id_measured'], row['axis0.motor.foc.Iq_measured']) <= 12,
            'Motor current vector threshold')


class AsciiIO:
    """Every transaction has a finite serial timeout; malformed replies latch uncertainty."""
    def __init__(self, serial_port, record):
        self.serial = serial_port
        self.record = record
        self.deadline = None
        self.uncertain = False
        self.cancelled = threading.Event()

    def check_deadline(self):
        require(not self.cancelled.is_set(), 'Operator cancelled')
        require(self.deadline is None or time.monotonic() < self.deadline, 'Calibration deadline reached')

    def send(self, command, *, cleanup=False):
        if not cleanup:
            self.check_deadline()
        payload = command.encode('ascii')
        try:
            require(self.serial.write(payload) == len(payload), 'Incomplete serial write')
        except BaseException:
            self.uncertain = True
            raise

    def read(self, key):
        require(key in READ_ALLOWED, 'Read not permitted: ' + key)
        began = time.monotonic()
        self.send('r ' + key + '\n')
        try:
            reply = self.serial.readline()
            pending = self.serial.in_waiting
            self.record['read_log'].append({'property': key, 'began': began,
                                           'ended': time.monotonic(), 'reply': reply.decode(errors='replace').strip()})
            require(reply.endswith(b'\n') and not pending, 'Ambiguous serial reply: ' + key)
            value = float(reply.decode().strip())
        except BaseException:
            self.uncertain = True
            raise
        self.check_deadline()
        return value

    def write(self, key, value, *, cleanup=False):
        require(key in WRITE_ALLOWED and math.isfinite(value), 'Write not permitted: ' + key)
        if key == 'axis0.requested_state':
            require(value in (1, 7), 'Only IDLE or encoder-offset calibration is permitted')
        if key in ('axis0.controller.input_vel', 'axis0.controller.input_torque'):
            require(value == 0, 'Nonzero velocity/torque command prohibited')
        entry = {'property': key, 'value': value, 'at': time.monotonic(), 'written': False}
        self.record['writes'].append(entry)
        self.send('w ' + key + ' ' + format(value, '.9g') + '\n', cleanup=cleanup)
        entry['written'] = True

    def feed(self):
        self.send('u 0\n')


def readback(io, key, expected):
    require(equal(io.read(key), expected), 'Readback mismatch: ' + key)


def stopped(io, record, *, timeout=1.2):
    end = time.monotonic() + timeout
    quiet = 0
    while time.monotonic() < end:
        state = io.read(STATE)
        velocity = io.read(VEL)
        quiet = quiet + 1 if state == 1 and math.isfinite(velocity) and abs(velocity * 60) < 2 else 0
        record.setdefault('settling', []).append({'at': time.monotonic(), 'state': state, 'rpm': velocity * 60})
        if quiet == 3:
            record['stationary_confirmed_at'] = time.monotonic()
            return
        time.sleep(.03)
    raise RuntimeError('Fresh IDLE/standstill not confirmed; keep temporary low limit and use cutoff')


def perform(io, record, checkpoint=lambda: None):
    before = {}
    mutated = False
    success = False
    seen_calibrating = False
    try:
        before = {key: io.read(key) for key in SNAP}
        record['before'] = before
        checkpoint()
        require(int(before['serial_number']) == int(SERIAL, 16), 'ODrive identity mismatch')
        require(int(before['commit_hash']) == COMMIT, 'Unexpected firmware build')
        require(before[STATE] == 1 and abs(before[VEL] * 60) < 2, 'Axis must be stationary in IDLE')
        require(before[ACTIVE] == 0 and before[DISARM] == 0, 'Existing fault; no automatic clear')
        require(before[SOFT] == 35 and before[HARD] == 38, 'Expected 35 A / 38 A limits')
        require(before[WATCH] == 0, 'Expected watchdog initially disabled')
        require(before['axis0.controller.input_vel'] == 0 and before['axis0.controller.input_torque'] == 0,
                'Nonzero retained input')
        require(before['axis0.config.motor.pole_pairs'] == 7, 'Unexpected pole-pair count')
        require(before['axis0.config.load_encoder'] == before['axis0.config.commutation_encoder'] == 10,
                'Expected shared RS485 encoder')
        require(before['rs485_encoder_group0.status'] == 0, 'Encoder communication fault')
        require(all(math.isfinite(before[k]) for k in TEMP), 'Temporary settings cannot be restored reliably')
        require(0 < before['axis0.config.motor.calibration_current'] <= 38 and
                0 < before['axis0.config.calibration_lockin.current'] <= 38,
                'Existing calibration current exceeds 38 A or is invalid; review before any changes')
        require(math.isfinite(before['axis0.config.calib_range']) and 0 < before['axis0.config.calib_range'] < 1,
                'Unexpected encoder calibration tolerance')
        for field in ('phase_resistance', 'phase_inductance'):
            require(before['axis0.config.motor.' + field + '_valid'] == 1 and
                    math.isfinite(before['axis0.config.motor.' + field]) and before['axis0.config.motor.' + field] > 0,
                    'Valid R/L calibration is required')
        require(before[DIRECTION] in (-1, 1) and all(math.isfinite(before[k]) for k in ALIGNMENT),
                'Previous alignment cannot be restored reliably')
        for key in ('vbus_voltage', 'axis0.motor.fet_thermistor.temperature', 'axis0.motor.motor_thermistor.temperature'):
            guard_value(key, before[key], before[POS])
        for key, value in TEMP.items():
            if key == WATCH:
                # Seed immediately BEFORE enabling, avoiding a stale watchdog timer.
                require(io.read(STATE) == 1 and io.read(ACTIVE) == io.read(DISARM) == 0, 'Pre-calibration fault/state changed')
                io.feed()
            mutated = True
            record['settings_mutated'] = True
            io.write(key, value)
            readback(io, key, value)
        readback(io, HARD, 38)
        effective = io.read('axis0.motor.effective_current_lim')
        record['effective_current_limit_before_calibration'] = effective
        require(math.isfinite(effective) and 0 < effective <= 10.001, 'Effective current limit is not within 10 A test bound')
        baseline = io.read(POS)
        require(math.isfinite(baseline), 'Invalid position baseline')
        # Fresh state/fault checks precede every calibration feed thereafter.
        require(io.read(STATE) == 1 and io.read(ACTIVE) == io.read(DISARM) == 0, 'Fault before calibration')
        io.feed()
        record['calibration_requested_at'] = time.monotonic()
        record['calibration_attempted'] = True
        io.deadline = record['calibration_requested_at'] + 20
        io.write('axis0.requested_state', 7)
        while True:
            row = {'began': time.monotonic(), 'complete': False}
            record['samples'].append(row)
            for key in TRACE:
                row[key] = io.read(key)
                guard_value(key, row[key], baseline)
                if key == 'axis0.motor.foc.Iq_measured':
                    validate_row(row)
            row['state_after'] = io.read(STATE)
            guard_value(STATE, row['state_after'], baseline)
            row['ended'] = time.monotonic()
            row['complete'] = True
            require(row['ended'] - row['began'] <= .2, 'Feedback sample too slow')
            if row[STATE] == 7 or row['state_after'] == 7:
                seen_calibrating = True
                record['seen_calibrating'] = True
            if seen_calibrating and row['state_after'] == 1:
                require(io.read(ACTIVE) == io.read(DISARM) == 0 and io.read(RESULT) == 0,
                        'Calibration ended without healthy SUCCESS')
                break
            require(row['ended'] - record['calibration_requested_at'] < 1 or seen_calibrating,
                    'Calibration did not enter state 7')
            io.feed()  # Only after a complete, healthy, bounded feedback row.
            time.sleep(.01)
        stopped(io, record)
        # Standstill confirmed: avoid incidental watchdog expiry during RAM reads.
        io.write(WATCH, 0)
        readback(io, WATCH, 0)
        after = {key: io.read(key) for key in (DIRECTION,) + MAPPERS + ('axis0.observed_encoder_scale_factor',)}
        record['alignment_after'] = after
        require(after[COMM + 'offset_valid'] == 1 and all(math.isfinite(after[k]) for k in ALIGNMENT),
                'Calibration did not produce valid alignment')
        require(after[DIRECTION] in (-1, 1) and equal(after[COMM + 'scale'], 7 * after[DIRECTION]),
                'Direction/commutation scale is inconsistent')
        observed = after['axis0.observed_encoder_scale_factor']
        require(math.isfinite(observed) and abs(observed - 1) <= before['axis0.config.calib_range'],
                'Observed encoder scale factor is outside the captured calibration tolerance')
        require(all(equal(after[k], before[k]) for k in MAPPERS if k not in ALIGNMENT),
                'Unexpected additional mapper change; manual review required')
        success = True
        record['result'] = 'Encoder-offset calibration succeeded; new alignment retained in RAM only'
    except BaseException as exc:
        record['result'] = type(exc).__name__ + ': ' + str(exc)
    finally:
        record['calibration_success'] = success
        io.deadline = None
        # Cancellation may not prevent stop or subsequent read-only cleanup.
        io.cancelled.clear()
        if mutated:
            record['cleanup_confirmed'] = False
            try:
                failures = []
                for key, value in (('axis0.controller.input_vel', 0), ('axis0.requested_state', 1)):
                    try:
                        io.write(key, value, cleanup=True)
                    except BaseException as exc:
                        failures.append(str(exc))
                record['stop_attempted_at'] = time.monotonic()
                require(not io.uncertain and not failures,
                        'Serial/Stop ambiguous; no alignment or higher-limit restoration. Switch wheel power off.')
                stopped(io, record)
                io.write(WATCH, 0)
                readback(io, WATCH, 0)
                io.write('axis0.controller.input_torque', 0)
                # Check even an aborted procedure: an unexpected mapper mutation
                # must never be followed by raising the current limit.
                extras = {key: io.read(key) for key in MAPPERS if key not in ALIGNMENT}
                record['mapper_invariants_before_restoration'] = extras
                require(all(equal(value, before[key]) for key, value in extras.items()),
                        'Unexpected mapper configuration changed; keep low current limit and review manually')
                if not success:
                    # Invalidate first; restore captured direction/scale/offset, valid LAST.
                    io.write(COMM + 'offset_valid', 0)
                    readback(io, COMM + 'offset_valid', 0)
                    for key in ALIGNMENT:
                        io.write(key, before[key])
                        readback(io, key, before[key])
                    record['previous_alignment_restored'] = True
                for key in TEMP:
                    if key in (WATCH, WATCH_TIME):
                        continue
                    io.write(key, before[key])
                    readback(io, key, before[key])
                io.write(WATCH, before[WATCH])
                readback(io, WATCH, before[WATCH])
                io.write(WATCH_TIME, before[WATCH_TIME])
                readback(io, WATCH_TIME, before[WATCH_TIME])
                record['after'] = {key: io.read(key) for key in (STATE, ACTIVE, DISARM, RESULT, SOFT, HARD,
                                                               'axis0.controller.input_vel', 'axis0.controller.input_torque') + ALIGNMENT}
                require(record['after'][STATE] == 1 and record['after'][HARD] == 38 and
                        record['after']['axis0.controller.input_vel'] == record['after']['axis0.controller.input_torque'] == 0,
                        'Final IDLE/zero/trip readback failed')
                record['cleanup_confirmed'] = True
            except BaseException as exc:
                record['cleanup_error'] = str(exc)
                record['operator_action_required'] = 'Switch wheel power off; retain evidence; do not retry'
        checkpoint()
    return record


def validate_endpoints():
    metadata = json.loads(ENDPOINTS.read_text())
    require(metadata['fw_version'] == '0.6.11-1', 'Wrong endpoint schema')
    for key in READ_ALLOWED:
        require(key in metadata['endpoints'] and 'r' in metadata['endpoints'][key]['access'], 'Missing readable endpoint: ' + key)
    for key in WRITE_ALLOWED:
        require(key in metadata['endpoints'] and 'w' in metadata['endpoints'][key]['access'], 'Missing writable endpoint: ' + key)


def check_ownership():
    require(PORT.exists(), 'Expected arm0 ODrive USB absent')
    nodes = set()
    for device in Path('/sys/bus/usb/devices').iterdir():
        try:
            if (device / 'serial').read_text().strip() == SERIAL:
                nodes.add('/dev/bus/usb/%03d/%03d' % (int((device / 'busnum').read_text()), int((device / 'devnum').read_text())))
        except (OSError, ValueError):
            pass
    require(len(nodes) == 1, 'Cannot uniquely identify arm0 USB')
    for proc in Path('/proc').glob('[0-9]*'):
        if int(proc.name) == os.getpid():
            continue
        try:
            require(b'climber_gui5.py' not in (proc / 'cmdline').read_bytes(), 'Close robot GUI')
            for fd in (proc / 'fd').iterdir():
                try:
                    target = os.readlink(fd)
                except OSError:
                    continue
                require(not target.startswith(('/dev/ttyACM', '/dev/ttyUSB')) and target not in nodes,
                        'Another process owns robot serial/USB: ' + proc.name)
        except (FileNotFoundError, PermissionError):
            pass


def run(auth_path, output_root):
    auth = json.loads(Path(auth_path).read_text())
    # Preserve original confirmation and current-session operator-presence evidence.
    # No fabricated fresh timestamp or arbitrary authorization expiry is required.
    require(auth.get('physical_confirmation') and auth.get('ongoing_presence_evidence'),
            'Document actual physical confirmation and continuing operator presence')
    for key in ('approved_diagnostic_testing', 'supported', 'wheel_free', 'hands_clear', 'at_cutoff', 'gui_closed'):
        require(auth.get(key) is True, 'Missing confirmation: ' + key)
    require(auth.get('user_message') and auth.get('plan') == PLAN, 'Authorization must match exact calibration plan')
    validate_endpoints()
    check_ownership()
    import serial
    directory = Path(output_root) / ('arm0-encoder-alignment-' + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()))
    directory.mkdir(exist_ok=False)
    record = {'plan': PLAN, 'authorization': auth, 'samples': [], 'writes': [], 'read_log': [],
              'calibration_attempted': False, 'settings_mutated': False, 'nvm_save': False, 'clear_count': 0}
    def checkpoint():
        temporary = directory / 'result.tmp'
        temporary.write_text(json.dumps(record, indent=2) + '\n')
        temporary.replace(directory / 'result.json')
    port = serial.Serial(port=None, baudrate=115200, timeout=.025, write_timeout=.025, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = str(PORT)
    try:
        port.open()
        time.sleep(.1)
        require(not port.read_all(), 'Unexpected pending ASCII output')
        perform(AsciiIO(port, record), record, checkpoint)
    finally:
        port.close()
        checkpoint()
        (directory / 'runner.py').write_text(Path(__file__).read_text())
        print(json.dumps({key: record.get(key) for key in ('result', 'calibration_attempted', 'calibration_success',
                                                          'cleanup_confirmed', 'cleanup_error', 'operator_action_required', 'after')}, indent=2))
        print('EVIDENCE', directory)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--authorization')
    parser.add_argument('--output-root', default=str(DEFAULT_ROOT))
    args = parser.parse_args()
    if not args.run:
        print(json.dumps(PLAN, indent=2))
    else:
        require(args.authorization, 'Authorization file required')
        run(args.authorization, args.output_root)
