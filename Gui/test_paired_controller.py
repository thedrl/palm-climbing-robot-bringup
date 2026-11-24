"""Controller-to-command integration regressions; no Tk or robot I/O."""

import copy
import importlib.util
from pathlib import Path
import queue
import sys
import types
import threading
import unittest
from unittest.mock import Mock, patch


GUI_DIR = Path(__file__).resolve().parent
if str(GUI_DIR) not in sys.path:
    sys.path.insert(0, str(GUI_DIR))
spec = importlib.util.spec_from_file_location("paired_controller_gui", GUI_DIR / "climber_gui5.py")
gui = importlib.util.module_from_spec(spec)
with patch("serial.Serial", side_effect=AssertionError("Hardware access forbidden in tests")):
    spec.loader.exec_module(gui)


class Var:
    def __init__(self, value=None):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class PairedControllerTests(unittest.TestCase):
    def setUp(self):
        guard = patch.object(gui.sup.serial, "Serial", side_effect=AssertionError("Hardware access forbidden in tests"))
        guard.start()
        self.addCleanup(guard.stop)
        available = patch.object(gui, "PYGAME_AVAILABLE", True)
        available.start()
        self.addCleanup(available.stop)
        self.app = a = gui.ClimberGUI.__new__(gui.ClimberGUI)
        a.root = Mock()
        a.root.after.return_value = "test-timer"
        a.units = [gui.sup.Unit(index=i, port=f"mock:{i}",
                   serial_port=types.SimpleNamespace(is_open=True),
                   stop_reader=threading.Event(), reader_thread=Mock(), identified=True)
                   for i in range(4)]
        a.output_queue = queue.Queue()
        a.controller = Mock(joystick=object())
        a.controller_enabled = Var(True)
        a.controller_status_var = Var("Connected")
        a.selected_actuator_var = Var("None selected")
        a.telemetry_var = Var()
        a.current_speed_var = Var()
        a.speed_increment_var = Var("50")
        a.max_wheel_speed_var = Var("600")
        a.current_wheel_speed = 500.0
        a.mapping = copy.deepcopy(gui.DEFAULT_MAPPING)
        for button, action in enumerate(gui.controls.ACTIONS):
            gui.controls.set_binding(a.mapping, action, button)
        a.selected_actuator = None
        a.selected_actuators = ()
        a.awaiting_learn = None
        a._learner = None
        a.prev_buttons = []
        a._previous_actions = {}
        a._input_snapshot = ([0., 0., -1., 0., 0., -1.], [False] * 16, [(0, 0)])
        a.last_wheel_rpm_sent = 0.0
        a._active_actuator_pulse = None
        a._controller_wait_for_release = False
        a._closing = False
        a._controller_after_id = None
        a._run_silent = Mock(return_value=True)
        a._log_event = Mock()
        a.map_value_vars = {}

    def tick(self, *pressed, axes=None, hat=(0, 0)):
        if axes is None:
            axes = [0., 0., -1., 0., 0., -1.]
        self.app.controller.poll.return_value = (axes, [i in pressed for i in range(16)], [hat])
        self.app._controller_tick()

    def commands(self):
        return [call.args[0] for call in self.app._run_silent.call_args_list]

    def choose(self, pair):
        self.tick({(0, 2): 11, (1, 3): 12}[pair])
        self.tick()
        self.assertEqual(self.app.selected_actuators, pair)
        self.assertEqual(self.commands(), [])

    def test_pair_02_hold_sends_both_once_and_release_stops_all(self):
        self.choose((0, 2))
        self.tick(4)
        self.tick(4)
        self.tick(4)
        self.tick()
        self.assertEqual(self.commands(), ["act0 fwd", "act2 fwd", "stop"])
        self.assertIsNone(self.app._active_actuator_pulse)

    def test_pair_13_reverse_sends_both_once_and_release_stops_all(self):
        self.choose((1, 3))
        self.tick(5)
        self.tick(5)
        self.tick()
        self.assertEqual(self.commands(), ["act1 rev", "act3 rev", "stop"])

    def test_missing_pair_member_prevents_either_motor_command(self):
        self.choose((0, 2))
        self.app.units = [unit for unit in self.app.units if unit.index != 2]
        self.tick(4)
        self.assertEqual(self.commands(), ["stop"])
        self.assertTrue(self.app.controller_enabled.get())
        self.assertIn("Selected actuator(s) unavailable: 2", "\n".join(self.app.output_queue.queue))

    def test_unidentified_or_closed_pair_member_prevents_either_command(self):
        self.choose((1, 3))
        for fault in ("unidentified", "closed"):
            with self.subTest(fault=fault):
                self.app.units[3].identified = fault != "unidentified"
                self.app.units[3].serial_port.is_open = fault != "closed"
                self.app.controller_enabled.set(True)
                self.app._controller_wait_for_release = False
                self.app._run_silent.reset_mock()
                self.tick(5)
                self.assertEqual(self.commands(), ["stop"])
                self.assertTrue(self.app.controller_enabled.get())

    def test_second_pair_write_failure_stops_every_unit_and_disables(self):
        self.choose((0, 2))
        self.app._run_silent.side_effect = [True, False, True]
        self.tick(4)
        self.assertEqual(self.commands(), ["act0 fwd", "act2 fwd", "stop"])
        self.assertFalse(self.app.controller_enabled.get())
        self.assertIsNone(self.app._active_actuator_pulse)
        self.app.controller.disconnect.assert_not_called()

    def test_missing_arm_does_not_disconnect_gamepad_or_block_other_arms(self):
        self.choose((0, 2))
        self.app.units = [unit for unit in self.app.units if unit.index != 0]
        self.tick(4)
        self.tick(4)
        self.assertEqual(self.commands(), ["stop"])
        self.assertTrue(self.app.controller_enabled.get())
        self.app.controller.disconnect.assert_not_called()
        self.tick()
        self.tick(1)
        self.tick()
        self.tick(4)
        self.assertEqual(self.commands(), ["stop", "act1 fwd"])
        self.tick()
        self.assertEqual(self.commands()[-1], "stop")

    def test_selection_change_while_held_stops_and_requires_full_release(self):
        self.choose((0, 2))
        self.tick(4)
        self.tick(4, 12)
        self.assertEqual(self.app.selected_actuators, (1, 3))
        self.tick(4)
        self.tick(4)
        self.assertEqual(self.commands(), ["act0 fwd", "act2 fwd", "stop"])
        self.tick()
        self.tick(4)
        self.assertEqual(self.commands(), ["act0 fwd", "act2 fwd", "stop", "act1 fwd", "act3 fwd"])

    def test_held_wheel_cannot_restart_after_actuator_release_stops_all(self):
        self.choose((0, 2))
        self.tick(4, 6)
        self.assertEqual(self.commands(), ["act0 fwd", "act2 fwd", "all 500.0"])
        self.tick(6)
        self.tick(6)
        self.assertEqual(self.commands(), ["act0 fwd", "act2 fwd", "all 500.0", "stop"])
        self.tick()
        self.tick(6)
        self.assertEqual(self.commands()[-2:], ["stop", "all 500.0"])

    def test_wheel_release_and_pair_press_in_same_poll_sends_only_stop(self):
        self.choose((0, 2))
        self.tick(6)
        self.tick(4)
        self.tick(4)
        self.assertEqual(self.commands(), ["all 500.0", "stop"])
        self.assertTrue(self.app._controller_wait_for_release)
        self.tick()
        self.tick(4)
        self.assertEqual(self.commands(), ["all 500.0", "stop", "act0 fwd", "act2 fwd"])

    def test_controller_disconnect_stops_pair_and_clears_selection(self):
        self.choose((1, 3))
        self.tick(5)
        self.app._disconnect_controller()
        self.assertEqual(self.commands(), ["act1 rev", "act3 rev", "stop"])
        self.assertFalse(self.app.controller_enabled.get())
        self.assertEqual(self.app.selected_actuators, ())
        self.assertIsNone(self.app._active_actuator_pulse)
        self.app.controller.disconnect.assert_called_once()

    def test_input_disconnect_during_pair_motion_stops_and_disables(self):
        self.choose((0, 2))
        self.tick(4)
        self.app.controller.poll.side_effect = RuntimeError("Controller disconnected")
        self.app._controller_tick()
        self.assertEqual(self.commands(), ["act0 fwd", "act2 fwd", "stop"])
        self.assertFalse(self.app.controller_enabled.get())
        self.assertEqual(self.app.controller_status_var.get(), "Disconnected (input error)")

    def test_opposing_actuator_controls_stop_until_both_released(self):
        self.choose((0, 2))
        self.tick(4)
        self.tick(4, 5)
        self.tick(5)
        self.tick(5)
        self.assertEqual(self.commands(), ["act0 fwd", "act2 fwd", "stop"])
        self.tick()
        self.tick(5)
        self.assertEqual(self.commands()[-2:], ["act0 rev", "act2 rev"])

    def test_opposing_wheel_controls_cannot_restart_until_released(self):
        self.tick(6)
        self.tick(6, 7)
        self.tick(7)
        self.assertEqual(self.commands(), ["all 500.0", "stop"])
        self.tick()
        self.tick(7)
        self.assertEqual(self.commands(), ["all 500.0", "stop", "all -500.0"])

    def test_duplicate_physical_mapping_blocks_enable(self):
        gui.controls.set_binding(self.app.mapping, "wheel_rev", 6)
        self.app._controller_enable_changed()
        self.assertFalse(self.app.controller_enabled.get())
        self.assertEqual(self.commands(), ["stop"])
        self.tick(6)
        self.assertEqual(self.commands(), ["stop"])
        self.assertIn("Fix mapping before enabling", "\n".join(self.app.output_queue.queue))

    def test_axis_only_r2_controls_pair_hold_and_release(self):
        gui.controls.set_binding(self.app.mapping, "actuator_pulse_fwd",
            {"kind": "axis", "index": 5, "rest": -1., "direction": 1})
        self.choose((0, 2))
        pressed = [0., 0., -1., 0., 0., 0.]
        self.tick(axes=pressed)
        self.tick(axes=pressed)
        self.assertEqual(self.commands(), ["act0 fwd", "act2 fwd"])
        self.tick()
        self.assertEqual(self.commands(), ["act0 fwd", "act2 fwd", "stop"])

    def test_trigger_held_when_enabled_must_release_before_motion(self):
        gui.controls.set_binding(self.app.mapping, "actuator_pulse_fwd",
            {"kind": "axis", "index": 5, "rest": -1., "direction": 1})
        self.choose((1, 3))
        self.app._controller_enable_changed()
        pressed = [0., 0., -1., 0., 0., 1.]
        self.tick(axes=pressed)
        self.tick(axes=pressed)
        self.assertEqual(self.commands(), [])
        self.assertTrue(self.app._controller_wait_for_release)
        self.tick()
        self.assertFalse(self.app._controller_wait_for_release)
        self.tick(axes=pressed)
        self.assertEqual(self.commands(), ["act1 fwd", "act3 fwd"])

    def test_dpad_selects_pair_and_changes_speed_only_on_press_edges(self):
        for action, direction in (("actuator_pair_02", (-1, 0)), ("actuator_pair_13", (1, 0)),
                                  ("speed_increase", (0, 1)), ("speed_decrease", (0, -1))):
            gui.controls.set_binding(self.app.mapping, action,
                {"kind": "hat", "index": 0, "x": direction[0], "y": direction[1]})
        self.tick(hat=(-1, 0))
        self.tick(hat=(-1, 0))
        self.assertEqual(self.app.selected_actuators, (0, 2))
        self.assertEqual(self.app._log_event.call_count, 1)
        self.tick()
        self.tick(hat=(1, 0))
        self.assertEqual(self.app.selected_actuators, (1, 3))
        self.tick()
        self.tick(hat=(0, 1))
        self.tick(hat=(0, 1))
        self.assertEqual(self.app.current_wheel_speed, 550.)
        self.tick()
        self.tick(hat=(0, -1))
        self.tick(hat=(0, -1))
        self.assertEqual(self.app.current_wheel_speed, 500.)
        self.assertEqual(self.commands(), [])

    def test_actuator_without_selection_stops_existing_wheel_motion(self):
        self.tick(6)
        self.tick(4, 6)
        self.tick(6)
        self.assertEqual(self.commands(), ["all 500.0", "stop"])
        self.assertTrue(self.app._controller_wait_for_release)
        self.tick()
        self.tick(6)
        self.assertEqual(self.commands(), ["all 500.0", "stop", "all 500.0"])

    def test_learn_r2_waits_for_axis_and_late_button_release_and_records_aliases(self):
        expected = {"kind": "axis", "index": 5, "rest": -1., "direction": 1}
        partial = [0., 0., -1., 0., 0., 0.]
        full = [0., 0., -1., 0., 0., 1.]
        with patch.object(gui, "save_mapping") as save:
            self.app._start_learn("actuator_pulse_fwd")
            self.assertFalse(self.app.controller_enabled.get())
            self.tick(axes=partial)
            self.tick(13, axes=full)
            self.assertEqual(gui.controls.get_binding(self.app.mapping, "actuator_pulse_fwd"), 4)
            save.assert_not_called()
            self.tick(13)  # Axis released; its late digital alias is still held.
            self.assertEqual(self.app.awaiting_learn, "actuator_pulse_fwd")
            self.tick()
            save.assert_called_once()
        self.assertEqual(gui.controls.get_binding(self.app.mapping, "actuator_pulse_fwd"), expected)
        groups = self.app.mapping.get("input_alias_groups", [])
        self.assertTrue(any(expected in group and 13 in group for group in groups))
        self.assertIsNone(self.app.awaiting_learn)
        self.assertFalse(self.app.controller_enabled.get())
        self.assertEqual(self.commands(), ["stop"])
        self.assertEqual(self.app._mapping_errors(), [])

    def test_learn_conflicting_alias_rejects_binding_and_retains_observed_aliases(self):
        expected = {"kind": "axis", "index": 4, "rest": 0., "direction": 1}
        gui.controls.set_binding(self.app.mapping, "wheel_rev", None)
        gui.controls.set_binding(self.app.mapping, "speed_increase", 7)
        self.assertEqual(self.app._mapping_errors(), [])
        partial = [0., 0., -1., 0., 0.7, -1.]
        full = [0., 0., -1., 0., 1., -1.]
        with patch.object(gui, "save_mapping") as save:
            self.app._start_learn("actuator_pulse_rev")
            self.tick(axes=partial)
            self.tick(7, axes=full)
            self.tick()
            self.assertEqual(gui.controls.get_binding(self.app.mapping, "actuator_pulse_rev"), 5)
            self.assertIn("Mapping not changed", "\n".join(self.app.output_queue.queue))
            self.assertTrue(any(expected in group and 7 in group
                                for group in self.app.mapping.get("input_alias_groups", [])))
            # Simulate an existing axis mapping whose digital alias was not yet learned.
            self.app.mapping.pop("input_alias_groups", None)
            gui.controls.set_binding(self.app.mapping, "actuator_pulse_rev", expected)
            self.assertEqual(self.app._mapping_errors(), [])
            self.app._start_learn("actuator_pulse_rev")
            self.tick(axes=partial)
            self.tick(7, axes=full)
            self.tick()
            self.assertEqual(save.call_count, 2)
        self.assertEqual(gui.controls.get_binding(self.app.mapping, "actuator_pulse_rev"), expected)
        self.assertTrue(any(expected in group and 7 in group
                            for group in self.app.mapping.get("input_alias_groups", [])))
        errors = "\n".join(self.app._mapping_errors())
        self.assertIn("Actuator reverse / retract", errors)
        self.assertIn("Increase wheel speed", errors)
        self.assertIn("share an input", errors)
        self.assertFalse(self.app.controller_enabled.get())
        self.assertIsNone(self.app.awaiting_learn)
        self.assertEqual(self.commands(), ["stop", "stop"])

    def test_stop_has_priority_over_pair_selection_and_trigger(self):
        self.tick(11, 4, 10)
        self.assertEqual(self.commands(), ["stop"])
        self.assertFalse(self.app.controller_enabled.get())
        self.assertEqual(self.app.selected_actuators, ())


if __name__ == "__main__":
    unittest.main()
