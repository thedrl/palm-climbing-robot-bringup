"""Recovery regressions using fake boards; no ports, Tk windows or motion."""
import copy
import importlib.util
from pathlib import Path
import queue
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('recovery_test_gui', Path(__file__).with_name('climber_gui5.py'))
gui = importlib.util.module_from_spec(spec)
with patch('serial.Serial', side_effect=AssertionError('Hardware forbidden')):
    spec.loader.exec_module(gui)


class Var:
    def __init__(self, value=None): self.value = value
    def get(self): return self.value
    def set(self, value): self.value = value


class Board:
    def __init__(self, index, history, fault=None):
        self.index = index
        self.identified = True
        self.serial_port = types.SimpleNamespace(is_open=True)
        self.stop_reader = threading.Event()
        self.history = history
        self.fault = fault
        self.recovery = None
        self.sample_after = 0

    def send(self, command, *, flush=True):
        self.history.append((self.index, command, flush))
        if self.fault == 'write': raise OSError('USB write failed')
        if command == 'stop':
            self.recovery.feed(self.index, 'Actuator stop requested.')
            self.recovery.feed(self.index, 'bringup> ODRIVE_PART,1,...')
        elif command == 'act stop':
            if self.fault == 'ack': return
            if self.fault == 'unconfirmed':
                self.recovery.feed(self.index, 'Actuator stop NOT confirmed; telemetry paused.')
                return
            # A preceding prompt is commonly attached to the ACK; it is NOT
            # evidence that the act-stop command completed after that ACK.
            self.recovery.feed(self.index, 'bringup> Actuator zero commands acknowledged.')
            self.recovery.feed(self.index, 'bringup> ACT_PART,1,...')
            self.sample_after = time.monotonic()
            if self.fault == 'reader': self.stop_reader.set()
        else:
            raise AssertionError('Unexpected command: ' + command)

    def sample(self):
        if not self.sample_after: return None
        now = time.monotonic()
        values = [240, 0, -97, 0, 0, 674, 0x03000000, 0, 60]
        ages = [0, 0, 0, 1000, 0]
        state, channel, started = 1, 1, self.sample_after
        if self.fault == 'old': started = 0
        if self.fault == 'pwm': values[3] = 10
        if self.fault == 'command': values[7] = 40
        if self.fault == 'current': values[1] = 90
        if self.fault == 'limit': values[5] = 700
        if self.fault == 'status': values[6] = 0x03000010
        if self.fault == 'stale': ages[4] = 601
        if self.fault == 'missing': ages[3] = -1
        if self.fault == 'moving': state = 2
        return gui.act_live.Sample(self.index, 123, channel, state, tuple(ages), tuple(values), now, started)


class RecoveryTests(unittest.TestCase):
    def make(self, faults=None):
        faults = faults or {}
        history, reports = [], []
        boards = [Board(i, history, faults.get(i)) for i in range(4)]
        recovery = gui.recovery.Recovery(boards, lambda i: boards[i].sample(),
                                        lambda i, msg: reports.append((i, msg)), timeout=.02)
        for board in boards: board.recovery = recovery
        return recovery, boards, history, reports

    def test_stop_all_before_actuator_recovery_and_preserve_unused_m2_baseline(self):
        recovery, boards, history, reports = self.make()
        recovery.run()
        self.assertEqual(history[:4], [(i, 'stop', False) for i in range(4)])
        self.assertEqual(history[4:], [(i, 'act stop', False) for i in range(4)])
        self.assertEqual(len(reports), 4)
        self.assertTrue(all('Software latch reset' in msg for _, msg in reports))
        self.assertTrue(recovery.done.is_set())

    def test_failed_write_does_not_block_other_units(self):
        recovery, boards, history, reports = self.make({1: 'write'})
        recovery.run()
        self.assertIn((3, 'act stop', False), history)
        self.assertIn('not verified', dict(reports)[1])
        self.assertIn('Software latch reset', dict(reports)[3])

    def test_bad_or_missing_verification_never_reports_recovered(self):
        for fault in ('ack', 'unconfirmed', 'reader', 'old', 'pwm', 'command', 'current', 'limit', 'stale', 'missing', 'moving'):
            with self.subTest(fault=fault):
                recovery, boards, history, reports = self.make({0: fault})
                recovery.run()
                self.assertIn('not verified', dict(reports)[0])
                self.assertIn('Software latch reset', dict(reports)[3])

    def test_remaining_controller_warning_is_reported_not_cleared(self):
        recovery, boards, history, reports = self.make({0: 'status'})
        recovery.run()
        self.assertIn('Controller status remains 0x03000010', dict(reports)[0])
        self.assertNotIn('Software latch reset', dict(reports)[0])

    def test_cancel_before_start_sends_nothing(self):
        recovery, boards, history, reports = self.make()
        recovery.cancel()
        recovery.run()
        self.assertEqual(history, [])
        self.assertTrue(recovery.done.is_set())

    def test_prompt_before_ack_is_not_completion_even_after_rescan(self):
        recovery, boards, history, reports = self.make()
        began = time.monotonic()
        recovery.feed(0, 'bringup>')
        recovery.feed(0, 'bringup> Actuator zero commands acknowledged.')
        with self.assertRaisesRegex(RuntimeError, 'no completed'):
            recovery.wait_prompt(boards[0], began, acknowledgement=True)

    def test_actual_fragmented_sample_with_prompt_prefix_starts_new_generation(self):
        recovery, boards, history, reports = self.make()
        store = gui.act_live.TelemetryStore()
        recovery.sample = store.sample
        record = 'ACT_LIVE,1,0,9,1,1,0,0,0,1000,0,240,0,-97,0,0,674,50331648,0,60'
        chunks = [record[i:i+24] for i in range(0, len(record), 24)]
        base = time.monotonic()
        with patch.object(gui.recovery.time, 'monotonic', return_value=base):
            recovery.feed(0, 'bringup> Actuator zero commands acknowledged.')
        for index, chunk in enumerate(chunks):
            text = ('bringup> ' if index == 0 else '') + f'ACT_PART,1,0,9,{index},{len(chunks)},{index},{chunk}'
            with patch.object(gui.recovery.time, 'monotonic', return_value=base + .001 + index * .001):
                recovery.feed(0, text)
                store.feed(0, text)
        completed = recovery.wait_prompt(boards[0], base - .001, acknowledgement=True)
        self.assertEqual(store.sample(0).started, completed)
        recovery.verify(boards[0], completed)
        self.assertIn('Software latch reset', reports[0][1])

    def test_prior_stage_ack_cannot_complete_new_request(self):
        recovery, boards, history, reports = self.make()
        recovery.feed(0, 'Actuator zero commands acknowledged.')
        recovery.feed(0, 'bringup> ACT_PART,1,...')
        began = time.monotonic()
        recovery.feed(0, 'bringup>')
        with self.assertRaisesRegex(RuntimeError, 'no completed'):
            recovery.wait_prompt(boards[0], began, acknowledgement=True)

    def test_fragment_generation_time_rejects_frame_started_before_prompt(self):
        recovery, boards, history, reports = self.make({0: 'old'})
        recovery.run()
        self.assertIn('verification unavailable', dict(reports)[0])


class GuiRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.app = gui.ClimberGUI.__new__(gui.ClimberGUI)
        a = self.app
        a.root = Mock()
        a.output_queue = queue.Queue()
        a.controller_enabled = Var(True)
        a.control_notice_var = Var()
        a._active_actuator_pulse = ((0, 2), 'rev')
        a.last_wheel_rpm_sent = 500
        a.selected_actuators = (0, 2)
        a.selected_actuator = None
        a.selected_actuator_var = Var('Actuators 0 + 2')
        a._act_recovery = None
        a.act_store = Mock()
        a.mapping = copy.deepcopy(gui.DEFAULT_MAPPING)
        a.units = [Board(i, []) for i in range(4)]
        a._log_event = Mock()

    def test_button_disables_motion_preserves_mapping_and_selection(self):
        before = copy.deepcopy(self.app.mapping)
        with patch.object(gui.recovery, 'Recovery') as factory:
            self.app._clear_actuator_errors()
        self.assertFalse(self.app.controller_enabled.get())
        self.assertTrue(self.app._controller_wait_for_release)
        self.assertIsNone(self.app._active_actuator_pulse)
        self.assertEqual(self.app.last_wheel_rpm_sent, 0)
        self.assertEqual(self.app.selected_actuators, (0, 2))
        self.assertEqual(self.app.mapping, before)
        factory.return_value.start.assert_called_once()

    def test_missing_and_failed_readers_are_reported_and_excluded(self):
        self.app.units.pop(1)
        self.app.units[1].stop_reader.set()
        with patch.object(gui.recovery, 'Recovery') as factory:
            self.app._clear_actuator_errors()
        self.assertEqual([u.index for u in factory.call_args.args[0]], [0, 3])
        log = '\n'.join(self.app.output_queue.queue)
        self.assertIn('u1] Unavailable', log)
        self.assertIn('u2] Unavailable', log)

    def test_raw_motion_and_config_blocked_during_recovery_but_stop_cancels(self):
        pending = self.app._act_recovery = Mock()
        pending.done.is_set.return_value = False
        for unit in self.app.units: unit.send = Mock()
        for command in ('act1 rev', 'act power 60', 'wheel clear', 'can reset'):
            self.assertFalse(self.app._run_silent(command))
        self.assertTrue(self.app._run_silent('stop'))
        pending.cancel.assert_called_once()
        for unit in self.app.units: unit.send.assert_called_once_with('stop')

    def test_safe_broadcast_continues_after_failure(self):
        for command, expected in (('act stop', 'act stop'), ('act all stop', 'act stop'),
                                  ('wheel clear', 'wheel clear'), ('all 0', 'wheel 0')):
            with self.subTest(command=command):
                for unit in self.app.units: unit.send = Mock()
                self.app.units[1].send.side_effect = OSError('disconnected')
                self.assertFalse(self.app._run_silent(command))
                for unit in self.app.units: unit.send.assert_called_once_with(expected)

    def test_motion_broadcast_keeps_existing_fail_fast_behavior(self):
        for unit in self.app.units: unit.send = Mock()
        self.app.units[1].send.side_effect = OSError('disconnected')
        self.assertFalse(self.app._run_silent('all 10'))
        self.app.units[2].send.assert_not_called()

    def test_missing_unit_is_dispatch_failure(self):
        self.app.units = [self.app.units[0]]
        self.app.units[0].send = Mock()
        self.assertFalse(self.app._run_silent('act1 rev'))
        self.app.units[0].send.assert_not_called()

    def test_selection_only_requires_release_without_movement(self):
        self.app._active_actuator_pulse = None
        self.app.last_wheel_rpm_sent = 0
        self.app._run_silent = Mock()
        self.app._choose_actuators((1, 3))
        self.assertEqual(self.app.selected_actuators, (1, 3))
        self.assertTrue(self.app._controller_wait_for_release)
        self.app._run_silent.assert_not_called()


class SerialFailureTests(unittest.TestCase):
    def test_reader_failure_marks_unit_unavailable_without_hardware_access(self):
        stop = threading.Event()
        serial = types.SimpleNamespace(is_open=True, readline=Mock(side_effect=OSError('disconnected')))
        gui.sup.reader_loop(1, serial, stop, queue.Queue())
        self.assertTrue(stop.is_set())
        unit = gui.sup.Unit(1, 'fake', serial, stop, Mock())
        self.assertFalse(gui.ClimberGUI._unit_available(unit))
        with self.assertRaisesRegex(OSError, 'USB reader disconnected'):
            unit.send('act stop')

    def test_partial_write_detected_and_recovery_does_not_flush(self):
        serial = types.SimpleNamespace(is_open=True, write=Mock(return_value=9), flush=Mock())
        unit = gui.sup.Unit(0, 'fake', serial, threading.Event(), Mock())
        unit.send('act stop', flush=False)
        serial.flush.assert_not_called()
        serial.write.return_value = 2
        with self.assertRaisesRegex(OSError, 'incomplete'):
            unit.send('act stop', flush=False)


if __name__ == '__main__': unittest.main()
