"""Host integration checks using fake arms; never opens robot connections."""

import contextlib
import importlib.util
import io
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch


GUI_PATH = Path(__file__).with_name("climber_gui5.py")
SUPERVISOR_PATH = Path(__file__).with_name("All Working.py")
spec = importlib.util.spec_from_file_location("all_working_gui_test", GUI_PATH)
gui = importlib.util.module_from_spec(spec)
with patch("serial.Serial", side_effect=AssertionError("Hardware access forbidden")):
    spec.loader.exec_module(gui)
sup = gui.sup


class RoutingTests(unittest.TestCase):
    def setUp(self):
        self.units = [types.SimpleNamespace(index=i, label=f"u{i}", send=Mock()) for i in (3, 1, 0, 2)]
        self.app = gui.ClimberGUI.__new__(gui.ClimberGUI)
        self.app.units = self.units
        self.app.output_queue = queue.Queue()

    def check_broadcast(self, command, expected):
        for unit in self.units:
            unit.send.reset_mock()
        self.assertTrue(self.app._run_silent(command))
        for unit in self.units:
            unit.send.assert_called_once_with(expected)

    def test_start_reaches_all_four(self):
        self.check_broadcast("start", "start")

    def test_center_reaches_all_four(self):
        self.check_broadcast("act center", "act center")

    def test_wheel_jog_reaches_all_four(self):
        self.check_broadcast("wheel test 5rpm", "wheel test 5rpm")

    def test_wheel_tune_reaches_all_four(self):
        self.check_broadcast("wheel tune apply", "wheel tune apply")
        self.check_broadcast("wheel tune set vel_gain 0.5", "wheel tune set vel_gain 0.5")
        self.check_broadcast("wheel cal", "wheel cal")

    def test_wheel_speeds_reach_all_four(self):
        for speed in ("10rpm", "-10", "0"):
            with self.subTest(speed=speed):
                self.check_broadcast("all " + speed, "wheel " + speed)

    def test_stop_aliases_reach_all_four(self):
        for command in ("stop", "s", "estop"):
            with self.subTest(command=command):
                self.check_broadcast(command, "stop")

    def test_diagnostics_reach_all_four(self):
        for command in ("unit", "status", "wheel diag", "act diag", "can status"):
            with self.subTest(command=command):
                self.check_broadcast(command, command)

    def test_each_unit_routes_by_identity_not_usb_order(self):
        for index in range(4):
            commands = {
                f"start {index}": "start",
                f"act{index} fwd": "act fwd",
                f"act{index} rev": "act rev",
                f"w{index} 5rpm": "wheel 5rpm",
                f"w{index} tune": "wheel tune",
                f"w{index} tune apply": "wheel tune apply",
                f"w{index} tune set vel_gain 0.5": "wheel tune set vel_gain 0.5",
                f"w{index} tune save": "wheel tune save",
                f"w{index} cal": "wheel cal",
                f"w{index} current save": "wheel current save",
                f"w{index} turn 10": "wheel turn 10",
                f"u{index} wheel diag": "wheel diag",
            }
            for command, expected in commands.items():
                with self.subTest(command=command):
                    for unit in self.units:
                        unit.send.reset_mock()
                    self.assertTrue(self.app._run_silent(command))
                    for unit in self.units:
                        if unit.index == index:
                            unit.send.assert_called_once_with(expected)
                        else:
                            unit.send.assert_not_called()

    def test_missing_unit_never_moves_a_different_arm(self):
        self.app.units = [unit for unit in self.units if unit.index != 1]
        self.app._run_silent("act1 fwd")
        for unit in self.units:
            unit.send.assert_not_called()
        self.assertIn("No unit 1", "\n".join(self.app.output_queue.queue))

    def test_stop_continues_after_one_arm_disconnects(self):
        self.units[0].send.side_effect = OSError("disconnected")
        self.assertFalse(self.app._run_silent("stop"))
        for unit in self.units:
            unit.send.assert_called_once_with("stop")


class ConnectionTests(unittest.TestCase):
    def test_four_discovered_ids_are_sorted_and_only_identity_is_sent(self):
        ports = [f"fake-port-{i}" for i in (3, 1, 0, 2)]
        serials = {}
        for index, port in zip((3, 1, 0, 2), ports):
            ser = Mock()
            ser.readline.return_value = f"CLIMBER_UNIT_ID={index} LOCAL_ODRIVE_NODE_ID=0\n".encode()
            serials[port] = ser
        with patch.object(sup.serial, "Serial", side_effect=lambda **kw: serials[kw["port"]]), \
                patch.object(sup.threading, "Thread"), patch.object(sup.time, "sleep"), \
                contextlib.redirect_stdout(io.StringIO()):
            units = sup.open_units(ports, sup.BAUD_DEFAULT, queue.Queue())
            try:
                self.assertEqual([unit.index for unit in units], [0, 1, 2, 3])
                for unit in units:
                    self.assertTrue(unit.identified)
                    self.assertEqual(unit.odrive_node_id, 0)
                    serials[unit.port].write.assert_called_once_with(b"id\nunit\n")
            finally:
                for unit in units:
                    unit.close()
        for ser in serials.values():
            ser.close.assert_called_once()


class LoadingTests(unittest.TestCase):
    def run_import(self, path, cwd):
        script = """
import importlib.util, pathlib, serial, sys
def forbidden(*args, **kwargs):
    raise AssertionError('No serial access allowed during import')
serial.Serial = forbidden
spec = importlib.util.spec_from_file_location('gui_import_check', sys.argv[1])
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)
assert pathlib.Path(gui.sup.__file__).resolve() == pathlib.Path(sys.argv[1]).with_name('All Working.py').resolve()
assert gui.sup.BAUD_DEFAULT == 115200
unit = gui.sup.Unit(1, 'fake-port', None, None, None)
assert unit.label == 'u1'
print('IMPORT_OK')
"""
        return subprocess.run(
            [sys.executable, "-B", "-c", script, str(path)], cwd=cwd,
            capture_output=True, text=True, timeout=20,
            env={**os.environ, "PYGAME_HIDE_SUPPORT_PROMPT": "1"},
        )

    def test_gui_uses_adjacent_file(self):
        self.assertEqual(Path(sup.__file__).resolve(), SUPERVISOR_PATH.resolve())

    def test_load_works_from_another_directory_and_ignores_its_scripts(self):
        with tempfile.TemporaryDirectory() as directory:
            for name in ("All Working.py", "climber_serial_supervisor.py"):
                Path(directory, name).write_text("raise AssertionError('Wrong supervisor loaded')\n")
            result = self.run_import(GUI_PATH, directory)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("IMPORT_OK", result.stdout)

    def test_missing_new_script_fails_clearly_without_old_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            copied_gui = Path(directory, "climber_gui5.py")
            shutil.copy2(GUI_PATH, copied_gui)
            result = self.run_import(copied_gui, directory)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("All Working.py", result.stderr)
        self.assertIn("Could not load", result.stderr)

    def test_terminal_help_runs_without_connecting(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [sys.executable, "-B", str(SUPERVISOR_PATH), "--help"], cwd=directory,
                capture_output=True, text=True, timeout=20,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--auto", result.stdout)
        self.assertIn("--ports", result.stdout)


if __name__ == "__main__":
    unittest.main(verbosity=2)


class AutoDetectMotorControllerTests(unittest.TestCase):
    def test_odrive_ports_are_not_probed_as_esp32_boards(self):
        ports = [
            types.SimpleNamespace(device="/dev/ttyACM0", description="Arduino NanoESP32", manufacturer="Arduino", vid=0x2341, pid=0x0070),
            types.SimpleNamespace(device="/dev/ttyACM4", description="ODrive S1", manufacturer="ODrive Robotics"),
            types.SimpleNamespace(device="/dev/ttyACM5", description="USB Serial", manufacturer="", vid=0x1209, pid=0x0d32),
            types.SimpleNamespace(device="/dev/ttyUSB0", description="USB Serial", manufacturer=""),
        ]
        with patch.object(sup.list_ports, "comports", return_value=ports):
            self.assertEqual(sup.likely_usb_serial_ports(), ["/dev/ttyACM0", "/dev/ttyUSB0"])


class TerminalRecoveryTests(unittest.TestCase):
    def run_terminal(self, units, commands, open_error=None):
        output = io.StringIO()
        with patch.object(sys, "argv", ["All Working.py", "--ports", "fake"]), \
             patch.object(sup, "open_units", return_value=units, side_effect=open_error), \
             patch.object(sup.threading, "Thread") as thread, \
             patch("builtins.input", side_effect=commands) as prompt, \
             contextlib.redirect_stdout(output):
            result = sup.main()
        printer_stop = thread.call_args.kwargs["args"][1]
        self.assertTrue(printer_stop.is_set())
        thread.return_value.join.assert_called_once_with(timeout=1.0)
        return result, output.getvalue(), prompt.call_count

    def unit(self, index):
        return types.SimpleNamespace(index=index, port=f"fake:{index}", label=f"u{index}", send=Mock(), close=Mock())

    def test_missing_unit_error_then_quit_keeps_session_and_closes(self):
        unit = self.unit(0)
        result, text, calls = self.run_terminal([unit], ["act1 rev", "quit"])
        self.assertEqual(result, 0)
        self.assertEqual(calls, 2)
        self.assertIn("Command error: No unit 1", text)
        unit.send.assert_called_once_with("stop")
        unit.close.assert_called_once()

    def test_dispatch_io_error_then_quit_keeps_session(self):
        unit = self.unit(0)
        unit.send.side_effect = [OSError("USB disconnected"), None]
        result, text, calls = self.run_terminal([unit], ["act0 rev", "quit"])
        self.assertEqual(calls, 2)
        self.assertIn("Command error: USB disconnected", text)
        self.assertEqual([call.args[0] for call in unit.send.call_args_list], ["act rev", "stop"])
        unit.close.assert_called_once()

    def test_cleanup_continues_after_stop_and_close_failures(self):
        units = [self.unit(i) for i in range(4)]
        units[0].send.side_effect = OSError("failed stop")
        units[1].close.side_effect = OSError("failed close")
        result, text, calls = self.run_terminal(units, ["quit"])
        self.assertEqual(result, 0)
        for unit in units:
            unit.send.assert_called_once_with("stop")
            unit.close.assert_called_once()
        self.assertIn("Stop delivery failed: failed stop", text)
        self.assertIn("Close failed: failed close", text)
