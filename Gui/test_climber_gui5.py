"""Controller regression tests; no serial ports or robot hardware are used."""

import copy
import importlib.util
import json
import os
from pathlib import Path
import queue
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch


spec = importlib.util.spec_from_file_location("gui", Path(__file__).with_name("climber_gui5.py"))
gui = importlib.util.module_from_spec(spec)
with patch("serial.Serial", side_effect=AssertionError("Hardware access forbidden in tests")):
    spec.loader.exec_module(gui)
supervisor = gui.sup


def setUpModule():
    guard = patch.object(supervisor.serial, "Serial", side_effect=AssertionError("Hardware access forbidden in tests"))
    guard.start()
    unittest.addModuleCleanup(guard.stop)


class Var:
    def __init__(self, value=None):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class MappingTests(unittest.TestCase):
    def read_mapping(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mapping.json"
            path.write_text(json.dumps(data))
            with patch.object(gui, "CONTROLLER_MAPPING_FILE", str(path)):
                return gui.load_mapping()

    def test_old_mapping_keys_preserved(self):
        data = {"wheel_axis": 1, "actuator_buttons": [{"fwd": 3}], "wheel_fwd_button": 13}
        result = self.read_mapping(data)
        for key in data:
            self.assertEqual(result[key], data[key])

    def test_malformed_containers_and_indices(self):
        for raw in (None, 3, True, "bad", {}, [-1, True, 2.5, "3", 99], [1, 2]):
            with self.subTest(raw=raw):
                result = self.read_mapping({"actuator_select_buttons": raw, "stop_button": -1})
                self.assertEqual(len(result["actuator_select_buttons"]), 4)
                self.assertIsNone(result["stop_button"])
                self.assertTrue(all(i is None or type(i) is int and i >= 0
                                    for i in result["actuator_select_buttons"]))

    def test_invalid_json_roots(self):
        for data in (None, 7, [], "broken"):
            self.assertEqual(self.read_mapping(data), gui.DEFAULT_MAPPING)

    def test_bad_scalar_indices_are_unmapped(self):
        for value in (True, -1, 0.5, "3", [], {}):
            self.assertIsNone(self.read_mapping({"wheel_fwd_button": value})["wheel_fwd_button"])


class ListenerTests(unittest.TestCase):
    def setUp(self):
        self.pg = Mock()
        self.pg.display.get_init.return_value = False
        self.pg.joystick.get_count.return_value = 1
        self.pg.event.get.return_value = []
        self.pg.JOYDEVICEREMOVED = 999
        self.joy = self.pg.joystick.Joystick.return_value
        self.joy.get_name.return_value = "Test controller"
        self.joy.get_instance_id.return_value = 42
        self.joy.get_numaxes.return_value = 1
        self.joy.get_axis.return_value = 0.25
        self.joy.get_numhats.return_value = 1
        self.joy.get_hat.return_value = (0, 1)
        self.joy.get_numbuttons.return_value = 2
        self.joy.get_button.side_effect = lambda i: i == 1
        self.addCleanup(patch.stopall)
        patch.object(gui, "pygame", self.pg).start()
        patch.object(gui, "PYGAME_AVAILABLE", True).start()
        patch.dict(os.environ).start()
        self.listener = gui.ControllerListener()

    def test_connect_uses_headless_display_not_global_init(self):
        self.assertEqual(self.listener.connect(), (True, "Test controller"))
        self.assertEqual(os.environ["SDL_VIDEODRIVER"], "dummy")
        self.assertEqual(os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"], "1")
        self.pg.init.assert_not_called()
        self.pg.display.init.assert_called_once()
        self.pg.mixer.init.assert_not_called()

    def test_native_video_backend_never_pumped(self):
        self.pg.display.get_init.return_value = True
        self.pg.display.get_driver.return_value = "cocoa"
        self.assertFalse(self.listener.connect()[0])
        self.pg.event.get.assert_not_called()

    def test_no_controller_is_normal_connection_failure(self):
        self.pg.joystick.get_count.return_value = 0
        self.assertFalse(self.listener.connect()[0])
        self.assertIsNone(self.listener.joystick)

    def test_initialization_error_is_reported(self):
        self.pg.display.init.side_effect = RuntimeError("driver unavailable")
        ok, info = self.listener.connect()
        self.assertFalse(ok)
        self.assertIn("driver unavailable", info)

    def test_poll_reads_axes_and_buttons(self):
        self.listener.connect()
        self.assertEqual(self.listener.poll(), ([0.25], [False, True], [(0, 1)]))

    def test_unplug_detected_before_reading_stale_buttons(self):
        self.listener.connect()
        self.pg.event.get.return_value = [types.SimpleNamespace(type=999, instance_id=42)]
        with self.assertRaisesRegex(RuntimeError, "disconnected"):
            self.listener.poll()
        self.joy.get_button.assert_not_called()

    def test_other_controller_unplug_is_ignored(self):
        self.listener.connect()
        self.pg.event.get.return_value = [types.SimpleNamespace(type=999, instance_id=101)]
        self.assertEqual(self.listener.poll()[1], [False, True])

    def test_disconnect_and_reconnect(self):
        self.listener.connect()
        self.listener.disconnect()
        self.assertEqual(self.listener.poll(), ([], [], []))
        self.assertTrue(self.listener.connect()[0])


class GuiControllerTests(unittest.TestCase):
    def setUp(self):
        self.app = a = gui.ClimberGUI.__new__(gui.ClimberGUI)
        a.root = Mock()
        a.root.after.return_value = "timer"
        a.units = [types.SimpleNamespace(index=i, identified=True, serial_port=types.SimpleNamespace(is_open=True)) for i in range(4)]
        a.output_queue = queue.Queue()
        a.controller = Mock(joystick=object())
        a.controller.poll.return_value = ([], [False] * 16, [])
        a.controller_enabled = Var(True)
        a.controller_status_var = Var("connected")
        a.selected_actuator_var = Var()
        a.telemetry_var = Var()
        a.current_speed_var = Var()
        a.speed_increment_var = Var("50")
        a.max_wheel_speed_var = Var("600")
        a.current_wheel_speed = 500.0
        a.mapping = copy.deepcopy(gui.DEFAULT_MAPPING)
        a.mapping["actuator_select_buttons"] = [0, 1, 2, 3]
        for key, value in zip(list(gui.DEFAULT_MAPPING)[1:], range(4, 11)):
            a.mapping[key] = value
        a.selected_actuator = 0
        a.selected_actuators = (0,)
        a.awaiting_learn = None
        a.prev_buttons = []
        a._previous_actions = {}
        a._input_snapshot = ([], [False] * 16, [])
        a._learner = None
        a.last_wheel_rpm_sent = 0.0
        a._active_actuator_pulse = None
        a._controller_wait_for_release = False
        a._closing = False
        a._controller_after_id = None
        a._queue_after_id = None
        a.map_value_vars = {}
        a._run_silent = Mock(return_value=True)
        self.addCleanup(patch.stopall)
        patch.object(gui, "PYGAME_AVAILABLE", True).start()

    def tick(self, *pressed):
        self.app.controller.poll.return_value = ([], [i in pressed for i in range(16)], [])
        self.app._controller_tick()

    def commands(self):
        return [call.args[0] for call in self.app._run_silent.call_args_list]

    def test_wheel_hold_release_commands_unchanged(self):
        self.tick(6)
        self.tick(6)
        self.tick()
        self.assertEqual(self.commands(), ["all 500.0", "stop"])

    def test_actuator_hold_release_commands_unchanged(self):
        self.tick(4)
        self.tick(4)
        self.tick()
        self.assertEqual(self.commands(), ["act0 fwd", "stop"])

    def test_stop_precedes_motion_and_latches_disabled(self):
        self.tick(4, 6, 10)
        self.tick(4, 6)
        self.assertEqual(self.commands(), ["stop"])
        self.assertFalse(self.app.controller_enabled.get())

    def test_input_error_requests_stop_and_reschedules(self):
        self.app.controller.poll.side_effect = RuntimeError("USB unplugged")
        self.app._controller_tick()
        self.assertEqual(self.commands(), ["stop"])
        self.assertFalse(self.app.controller_enabled.get())
        self.app.controller.disconnect.assert_called_once()
        self.app.root.after.assert_called_once_with(30, self.app._controller_tick)

    def test_disconnect_stops_and_clears_motion_state(self):
        self.app.last_wheel_rpm_sent = 500
        self.app._active_actuator_pulse = (0, "fwd")
        self.app._disconnect_controller()
        self.assertEqual(self.commands(), ["stop"])
        self.assertEqual(self.app.last_wheel_rpm_sent, 0)
        self.assertIsNone(self.app._active_actuator_pulse)
        self.assertFalse(self.app.controller_enabled.get())

    def test_disable_checkbox_stops(self):
        self.app.controller_enabled.set(False)
        self.app._controller_enable_changed()
        self.assertEqual(self.commands(), ["stop"])

    def test_enable_requires_release_before_new_press(self):
        self.app._controller_enable_changed()
        self.tick(6)
        self.tick()
        self.assertEqual(self.commands(), [])
        self.tick(6)
        self.assertEqual(self.commands(), ["all 500.0"])

    def test_learning_motion_button_does_not_move_robot(self):
        with patch.object(gui, "save_mapping"):
            self.app._start_learn("wheel_fwd")
            self.tick(12)
            self.tick()
        self.assertEqual(self.app.mapping["wheel_fwd_button"], 12)
        self.assertEqual(self.commands(), ["stop"])
        self.assertFalse(self.app.controller_enabled.get())

    def test_bad_button_indices_never_crash_or_select_negative(self):
        for idx in (None, -1, -100, "0", 0.5, True, 999, [], {}):
            self.assertFalse(self.app._button_down(idx, [True]))

    def test_invalid_speed_disables_control_without_motion(self):
        for bad in ("nan", "inf", "-1", "bad", ""):
            self.app.controller_enabled.set(True)
            self.app._controller_wait_for_release = False
            self.app.max_wheel_speed_var.set(bad)
            self.app._run_silent.reset_mock()
            self.tick(6)
            self.assertEqual(self.commands(), ["stop"])

    def test_reduced_speed_cap_is_applied(self):
        self.app.max_wheel_speed_var.set("25")
        self.tick(6)
        self.assertEqual(self.commands(), ["all 25.0"])

    def test_command_failure_disables_and_attempts_stop(self):
        self.app._run_silent.side_effect = [False, True]
        self.tick(6)
        self.assertEqual(self.commands(), ["all 500.0", "stop"])
        self.assertFalse(self.app.controller_enabled.get())

    def test_stop_failure_does_not_prevent_polling_recovery(self):
        self.app._run_silent.return_value = False
        self.app.controller.poll.side_effect = RuntimeError("unplugged")
        self.app._controller_tick()
        self.app.root.after.assert_called_once()
        logs = list(self.app.output_queue.queue)
        self.assertTrue(any("cutoff" in line for line in logs))

    def test_stop_attempted_for_every_unit_after_one_fails(self):
        handler = Mock(side_effect=[OSError("disconnected"), None, None, None])
        with patch.object(supervisor, "handle_supervisor_command", handler):
            result = gui.ClimberGUI._run_silent(self.app, "stop")
        self.assertFalse(result)
        self.assertEqual(handler.call_count, 4)
        self.assertEqual(handler.call_args_list[1].args, ([self.app.units[1]], "stop"))

    def test_closed_window_does_not_poll_or_reschedule(self):
        self.app._closing = True
        self.app._controller_tick()
        self.app.controller.poll.assert_not_called()
        self.app.root.after.assert_not_called()

    def test_console_flood_is_bounded(self):
        self.app.console = Mock()
        for i in range(1000):
            self.app.output_queue.put(str(i))
        self.app._poll_queue()
        self.assertEqual(self.app.console.insert.call_count, 200)
        self.assertEqual(self.app.output_queue.qsize(), 800)
        self.app.root.after.assert_called_once_with(80, self.app._poll_queue)

    def test_repeated_controller_cycles(self):
        for _ in range(1000):
            self.app.controller_enabled.set(True)
            self.app._controller_enable_changed()
            self.tick()
            self.tick(6)
            self.tick(10)
        self.assertEqual(self.commands().count("all 500.0"), 1000)
        self.assertEqual(self.commands().count("stop"), 1000)

    def test_validate_wheel_tune_param_bounds(self):
        self.assertEqual(gui.validate_wheel_tune_param("vel_ramp_rate", "10.0"), ("vel_ramp_rate", 10.0))
        self.assertEqual(gui.validate_wheel_tune_param("spinout_mech", "-80"), ("spinout_mech", -80.0))
        for param, bad in (
            ("unknown_param", "1.0"),
            ("vel_gain", "nan"),
            ("vel_gain", "inf"),
            ("vel_gain", "0.01"),
            ("vel_ramp_rate", "100"),
            ("spinout_mech", "10"),
        ):
            with self.assertRaises(ValueError):
                gui.validate_wheel_tune_param(param, bad)

    def test_confirm_wheel_action_guards_and_dispatch(self):
        self.app._run = Mock()
        with patch.object(gui.messagebox, "askyesno", return_value=False) as ask:
            self.assertFalse(self.app._confirm_wheel_action(1, "t", "m", "w1 tune save"))
            ask.assert_called_once()
        self.app._run.assert_not_called()

        with patch.object(gui.messagebox, "askyesno", return_value=True):
            self.assertTrue(self.app._confirm_wheel_action(1, "t", "m", "w1 cal"))
        self.assertFalse(self.app.controller_enabled.get())
        self.app._run.assert_called_once_with("w1 cal")

        self.app.units = []
        self.app._run.reset_mock()
        with patch.object(gui.messagebox, "showwarning") as warn:
            self.assertFalse(self.app._confirm_wheel_action(1, "t", "m", "w1 cal"))
            warn.assert_called_once()
        self.app._run.assert_not_called()

    def test_set_wheel_tune_param_validation_and_dispatch(self):
        self.app._run = Mock()
        with patch.object(gui.messagebox, "showerror") as err:
            self.assertFalse(self.app._set_wheel_tune_param(2, "vel_gain", "99"))
            err.assert_called_once()
        self.app._run.assert_not_called()

        self.assertTrue(self.app._set_wheel_tune_param(2, "vel_ramp_rate", "12.5"))
        self.assertFalse(self.app.controller_enabled.get())
        self.app._run.assert_called_once_with("w2 tune set vel_ramp_rate 12.5")

    def test_wheel_tune_and_current_broadcasts_isolate_failing_unit(self):
        for cmd in ("wheel tune", "wheel current"):
            handler = Mock(side_effect=[OSError("disconnected"), None, None, None])
            with patch.object(supervisor, "handle_supervisor_command", handler):
                result = gui.ClimberGUI._run_silent(self.app, cmd)
            self.assertFalse(result)
            self.assertEqual(handler.call_count, 4)
            self.assertEqual(handler.call_args_list[1].args, ([self.app.units[1]], cmd))


if __name__ == "__main__":
    unittest.main(verbosity=2)
