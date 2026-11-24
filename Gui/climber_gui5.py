#!/usr/bin/env python3
"""
Climber Supervisor GUI
=======================

A graphical front-end for All Working.py.

Serial commands use the adjacent All Working.py supervisor. This GUI also
handles controller mapping, paired actuator selection, and release-to-stop.

SETUP:
  1. Put this file in the same folder as All Working.py.
  2. pip install pyserial pygame   (pygame is only needed for the controller tab)
  3. Run:  python3 climber_gui5.py

CONTROLLER NOTES:
  A PS4 controller's button/axis numbering is NOT consistent across
  operating systems, drivers, or USB vs Bluetooth connections. Rather than
  hardcode guesses that could fire the wrong actuator, this GUI has a
  "Learn" (calibrate) button for each control: click it, then press and release a
  button, trigger, or D-pad direction, and it records that input. The mapping is
  saved to controller_mapping.json next to this file so you only need to
  calibrate once.
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import queue
import copy
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

# ---------------------------------------------------------------------------
# Load the adjacent script by path, including its space-containing filename.
# ---------------------------------------------------------------------------
SUPERVISOR_FILENAME = "All Working.py"
SUPERVISOR_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), SUPERVISOR_FILENAME)

try:
    _supervisor_spec = importlib.util.spec_from_file_location("climber_all_working", SUPERVISOR_PATH)
    if _supervisor_spec is None or _supervisor_spec.loader is None:
        raise ImportError(f"Cannot load {SUPERVISOR_PATH}")
    sup = importlib.util.module_from_spec(_supervisor_spec)
    # dataclass processing needs the module registered before execution.
    sys.modules[_supervisor_spec.name] = sup
    _supervisor_spec.loader.exec_module(sup)
except (ImportError, OSError) as exc:
    sys.modules.pop("climber_all_working", None)
    print(f"Could not load {SUPERVISOR_PATH}: {exc}", file=sys.stderr)
    print("Make sure it is in the same folder as this GUI file.", file=sys.stderr)
    raise

_live_spec = importlib.util.spec_from_file_location(
    "climber_odrive_live", os.path.join(os.path.dirname(os.path.abspath(__file__)), "odrive_live.py")
)
live = importlib.util.module_from_spec(_live_spec)
sys.modules[_live_spec.name] = live
_live_spec.loader.exec_module(live)
_act_spec = importlib.util.spec_from_file_location(
    "climber_actuator_live", os.path.join(os.path.dirname(os.path.abspath(__file__)), "actuator_live.py"))
act_live = importlib.util.module_from_spec(_act_spec)
sys.modules[_act_spec.name] = act_live
_act_spec.loader.exec_module(act_live)
_recovery_spec = importlib.util.spec_from_file_location(
    "climber_actuator_recovery", os.path.join(os.path.dirname(os.path.abspath(__file__)), "actuator_recovery.py"))
recovery = importlib.util.module_from_spec(_recovery_spec)
_recovery_spec.loader.exec_module(recovery)
_log_spec = importlib.util.spec_from_file_location(
    "climber_session_log", os.path.join(os.path.dirname(os.path.abspath(__file__)), "session_log.py"))
session_log = importlib.util.module_from_spec(_log_spec)
sys.modules[_log_spec.name] = session_log
_log_spec.loader.exec_module(session_log)
_controls_spec = importlib.util.spec_from_file_location(
    "climber_controller_inputs", os.path.join(os.path.dirname(os.path.abspath(__file__)), "controller_inputs.py"))
controls = importlib.util.module_from_spec(_controls_spec)
sys.modules[_controls_spec.name] = controls
_controls_spec.loader.exec_module(controls)



WHEEL_TUNING_PARAMS: dict[str, tuple[float, float]] = {
    "vel_gain": (0.05, 2.0),
    "vel_integrator_gain": (0.0, 15.0),
    "vel_integrator_limit": (0.1, 3.0),
    "vel_limit_tolerance": (1.1, 4.0),
    "vel_ramp_rate": (0.5, 50.0),
    "spinout_elec": (10.0, 300.0),
    "spinout_mech": (-300.0, -10.0),
    "dc_undervoltage": (30.0, 44.0),
    "dc_overvoltage": (51.0, 57.0),
    "dc_max_pos_current": (5.0, 60.0),
    "dc_max_neg_current": (-15.0, -0.01),
}


def validate_wheel_tune_param(param: str, text: str) -> tuple[str, float]:
    """Validate an ODrive wheel tuning parameter name and numeric range."""
    name = param.strip()
    if name not in WHEEL_TUNING_PARAMS:
        raise ValueError(f"Unknown tuning parameter '{param}'.")
    try:
        value = float(text.strip())
    except ValueError as exc:
        raise ValueError(f"Enter a finite numeric value for {name}.") from exc
    min_val, max_val = WHEEL_TUNING_PARAMS[name]
    if not math.isfinite(value) or not min_val <= value <= max_val:
        raise ValueError(f"{name} must be between {min_val:g} and {max_val:g}.")
    return name, value


try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False

CONTROLLER_MAPPING_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "controller_mapping.json"
)

DEFAULT_MAPPING = {
    # One button selects which actuator the pulse buttons below act on.
    "actuator_select_buttons": [None, None, None, None],  # index per actuator 0-3
    # Shared pulse buttons: act on whichever actuator is currently selected.
    "actuator_pulse_fwd_button": None,
    "actuator_pulse_rev_button": None,
    # Shared buttons driving all ODrive/wheel motors together.
    "wheel_fwd_button": None,
    "wheel_rev_button": None,
    # Buttons that adjust the wheel speed used by wheel_fwd/wheel_rev.
    "speed_increase_button": None,
    "speed_decrease_button": None,
    # Single button that stops everything (wheels + actuators).
    "stop_button": None,
    "actuator_pair_02_button": None,
    "actuator_pair_13_button": None,
}


def load_mapping() -> dict:
    mapping = copy.deepcopy(DEFAULT_MAPPING)
    if os.path.exists(CONTROLLER_MAPPING_FILE):
        try:
            with open(CONTROLLER_MAPPING_FILE, "r") as fh:
                saved = json.load(fh)
            if isinstance(saved, dict):
                mapping.update(saved)
        except (OSError, json.JSONDecodeError):
            pass

    # Preserve legacy button indices and accept typed axis/hat bindings.
    for action in controls.ACTIONS:
        controls.set_binding(mapping, action, controls.normalize_binding(
            controls.get_binding(mapping, action)))

    return mapping


def save_mapping(mapping: dict) -> None:
    try:
        with open(CONTROLLER_MAPPING_FILE, "w") as fh:
            json.dump(mapping, fh, indent=2)
    except OSError as exc:
        raise RuntimeError(f"Could not save controller mapping: {exc}") from exc


class ControllerCommandError(RuntimeError):
    """Robot dispatch failed; the gamepad itself may still be connected."""


class ControllerListener:
    """Thin wrapper around pygame's joystick API. No robot logic lives here."""

    def __init__(self) -> None:
        self.joystick = None
        self.name: str | None = None

    def connect(self) -> tuple[bool, str]:
        if not PYGAME_AVAILABLE:
            return False, "pygame is not installed. Run: pip install pygame"
        self.disconnect()
        try:
            # Tk owns the native window loop. SDL's Cocoa event pump re-enters
            # Tk callbacks on macOS and can abort Python, beyond try/except.
            os.environ["SDL_VIDEODRIVER"] = "dummy"
            os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
            if pygame.display.get_init() and pygame.display.get_driver() != "dummy":
                return False, "Restart the GUI to switch to headless controller input."
            pygame.display.init()
            pygame.joystick.init()
            pygame.event.get()
            if pygame.joystick.get_count() == 0:
                return False, "No controller detected. Check USB cable or Bluetooth pairing."
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.name = self.joystick.get_name()
            return True, self.name
        except Exception as exc:
            self.disconnect()
            return False, f"Could not connect controller: {exc}"

    def disconnect(self) -> None:
        if self.joystick is not None:
            try:
                self.joystick.quit()
            except Exception:
                pass
        self.joystick = None
        self.name = None

    def poll(self) -> tuple[list, list, list]:
        if self.joystick is None:
            return [], [], []
        # Drain the queue as well as updating button state, including unplug
        # events. A disconnected joystick can otherwise retain held buttons.
        instance_id = self.joystick.get_instance_id()
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED and event.instance_id == instance_id:
                raise RuntimeError("Controller disconnected. Reconnect and re-enable control.")
        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        buttons = [bool(self.joystick.get_button(i)) for i in range(self.joystick.get_numbuttons())]
        hats = [self.joystick.get_hat(i) for i in range(self.joystick.get_numhats())]
        return axes, buttons, hats


class QueueWriter:
    """A file-like object that pushes writes into a queue instead of stdout.

    Used to catch print() output from your supervisor's own functions
    (e.g. open_units' 'Opened /dev/ttyUSB0 as u0...' messages) so it shows
    up in the GUI console alongside the live serial feed.
    """

    def __init__(self, q: "queue.Queue[str]"):
        self.q = q
        self._buf = ""

    def write(self, text: str) -> None:
        self._buf += text
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            if line:
                self.q.put(line)

    def flush(self) -> None:
        pass


class ClimberGUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Climber Supervisor — V18 Actuator Recovery")
        available_width = max(640, self.root.winfo_screenwidth() - 80)
        available_height = max(480, self.root.winfo_screenheight() - 100)
        self.root.geometry(f"{min(1180, available_width)}x{min(800, available_height)}")
        self.root.minsize(min(900, available_width), min(600, available_height))

        self.live_store = live.TelemetryStore()
        self.act_store = act_live.TelemetryStore()
        self.act_vars = {}
        self.act_global_vars = {}
        self._live_after_id = None
        self.live_vars = {}
        self.units: list = []
        self._act_recovery = None
        self._recovery_after_id = None
        self.output_queue: "queue.Queue[str]" = queue.Queue()
        try:
            self.session_log = session_log.SessionLog(os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs"))
            self.output_queue.put(f"[logging] {session_log.REVISION} session {self.session_log.session}; logs: {self.session_log.directory}")
        except Exception:
            self.session_log = None
            self.output_queue.put("[logging] Diagnostic logger could not start; controls remain available.")


        # Controller state
        self.controller = ControllerListener()
        self.mapping = load_mapping()
        self.controller_enabled = tk.BooleanVar(value=False)
        self.control_notice_var = tk.StringVar(value="Controller disabled")
        self.awaiting_learn: str | None = None
        self.prev_buttons: list = []
        self._previous_actions = {}
        self._input_snapshot = ([], [], [])
        self._learner = None
        self.last_wheel_rpm_sent = 0.0
        self.map_value_vars: dict = {}
        self._active_actuator_pulse = None
        self._controller_wait_for_release = True
        self._closing = False
        self._controller_after_id = None
        self._queue_after_id = None

        # Which actuator (0-3) the pulse buttons currently act on.
        self.selected_actuator: int | None = None
        self.selected_actuators: tuple = ()
        self.selected_actuator_var = tk.StringVar(value="None selected")

        # Wheel speed used by the wheel fwd/rev buttons, adjustable live.
        self.current_wheel_speed = 500.0
        self.current_speed_var = tk.StringVar(value=f"{self.current_wheel_speed:g} RPM")
        self.speed_increment_var = tk.StringVar(value="50")
        self.max_wheel_speed_var = tk.StringVar(value="600")

        self._build_style()
        self._build_layout()
        self._poll_queue()
        self._controller_tick()
        self._refresh_live()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ------------------------------------------------------------------ #
    # Layout
    # ------------------------------------------------------------------ #
    def _build_style(self) -> None:
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TNotebook.Tab", padding=(14, 8))
        style.configure("Group.TLabelframe", padding=10)
        style.configure("Group.TLabelframe.Label", font=("Segoe UI", 10, "bold"))
        style.configure("TButton", padding=6)

    def _build_layout(self) -> None:
        # ---- Top bar: connection controls -------------------------------
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="USB ports:").grid(row=0, column=0, sticky="w")
        self.ports_var = tk.StringVar()
        ttk.Entry(top, textvariable=self.ports_var, width=32).grid(row=0, column=1, padx=5, sticky="ew")
        top.columnconfigure(1, weight=1)

        ttk.Button(top, text="Auto-detect", command=self._auto_detect).grid(row=0, column=2, padx=3)
        ttk.Button(top, text="List Ports", command=self._list_ports).grid(row=0, column=3, padx=3)

        ttk.Label(top, text="Baud:").grid(row=0, column=4, padx=(15, 3))
        self.baud_var = tk.StringVar(value=str(sup.BAUD_DEFAULT))
        ttk.Entry(top, textvariable=self.baud_var, width=8).grid(row=0, column=5)

        self.discover_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Discover unit IDs", variable=self.discover_var).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(5, 0)
        )
        self.allow_unknown_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="Allow unknown boards", variable=self.allow_unknown_var).grid(
            row=1, column=2, columnspan=2, sticky="w", pady=(5, 0)
        )

        self.connect_btn = ttk.Button(top, text="Connect", command=self._connect)
        self.connect_btn.grid(row=0, column=6, padx=(15, 3))
        self.disconnect_btn = ttk.Button(
            top, text="Disconnect", command=self._disconnect, state="disabled"
        )
        self.disconnect_btn.grid(row=0, column=7, padx=3)

        self.status_var = tk.StringVar(value="Not connected")
        ttk.Label(top, textvariable=self.status_var, foreground="#555").grid(
            row=2, column=0, columnspan=8, sticky="w", pady=(6, 0)
        )

        # ---- Persistent safety bar ---------------------------------------
        safety = ttk.Frame(self.root, padding=(10, 0))
        safety.pack(fill="x")
        self.estop_button = tk.Button(
            safety,
            text="\u26A0  E-STOP ALL  \u26A0",
            bg="#c0392b",
            fg="white",
            font=("Segoe UI", 11, "bold"),
            activebackground="#e74c3c",
            command=self._emergency_stop,
        )
        self.estop_button.pack(fill="x", pady=6)
        self.control_notice = ttk.Label(
            safety, textvariable=self.control_notice_var, foreground="#875000",
            wraplength=850, justify="left")
        self.control_notice.pack(fill="x", pady=(0, 5))
        safety.bind("<Configure>", lambda event: self.control_notice.configure(
            wraplength=max(300, event.width - 24)))
        selection = ttk.Frame(safety)
        selection.pack(fill="x", pady=(0, 5))
        ttk.Label(selection, text="Controller actuators:").pack(side="left")
        ttk.Label(selection, textvariable=self.selected_actuator_var, foreground="#06c",
                  font=("Segoe UI", 10, "bold")).pack(side="left", padx=6)
        for group in ((0,), (1,), (2,), (3,), (0, 2), (1, 3)):
            ttk.Button(selection, text=" + ".join(map(str, group)), width=5,
                       command=lambda g=group: self._choose_actuators_from_gui(g)).pack(side="left", padx=2)
        ttk.Label(selection, text="Select only; then release and press extend/retract.").pack(side="left", padx=6)

        # Both panes have minimum heights, so tall tabs cannot hide the console.
        self.main_panes = tk.PanedWindow(
            self.root, orient="vertical", sashwidth=8, sashrelief="raised",
            showhandle=True, opaqueresize=True, borderwidth=0)
        self.main_panes.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        self.notebook = ttk.Notebook(self.main_panes, height=320)
        self.main_panes.add(self.notebook, minsize=120, stretch="always")

        self._build_global_tab()
        for i in range(4):
            self._build_unit_tab(i)
        self._build_raw_tab()
        self._build_controller_tab()

        # ---- Console -------------------------------------------------------
        self.console_frame = ttk.LabelFrame(
            self.main_panes, text="Console — drag the divider to resize", style="Group.TLabelframe")
        self.main_panes.add(self.console_frame, minsize=175, height=210, stretch="never")
        command_row = ttk.Frame(self.console_frame)
        command_row.pack(side="bottom", fill="x", pady=(6, 0))
        ttk.Label(command_row, text="Command:").pack(side="left", padx=(0, 5))
        self.console_command_var = tk.StringVar()
        self.console_command_entry = ttk.Entry(command_row, textvariable=self.console_command_var)
        self.console_command_entry.pack(side="left", fill="x", expand=True, padx=(0, 5))
        self.console_command_entry.bind("<Return>", lambda event: self._send_console_command())
        self.console_send_button = ttk.Button(command_row, text="Send", command=self._send_console_command)
        self.console_send_button.pack(side="right")
        self.console = scrolledtext.ScrolledText(
            self.console_frame, height=7, bg="#111", fg="#0f0", insertbackground="#0f0",
            font=("Consolas", 10), state="disabled", wrap="word"
        )
        self.console.pack(fill="both", expand=True)

    def _send_console_command(self) -> None:
        command = self.console_command_var.get().strip()
        if command:
            self._run(command)
            self.console_command_var.set("")

    def _scrollable_tab(self, title: str) -> ttk.Frame:
        """Keep every control reachable while reserving space for the console."""
        outer = ttk.Frame(self.notebook)
        self.notebook.add(outer, text=title)
        outer.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)
        canvas = tk.Canvas(outer, highlightthickness=0, height=250)
        vertical = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        horizontal = ttk.Scrollbar(outer, orient="horizontal", command=canvas.xview)
        canvas.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        content = ttk.Frame(canvas, padding=10)
        window = canvas.create_window((0, 0), window=content, anchor="nw")

        def size_content(_event=None):
            canvas.itemconfigure(window, width=max(canvas.winfo_width(), content.winfo_reqwidth()))
            canvas.configure(scrollregion=canvas.bbox("all"))

        content.bind("<Configure>", size_content)
        canvas.bind("<Configure>", size_content)
        # Wheel events are scoped to descendants of this canvas; no global
        # binding can steal scrolling from the persistent console.
        def scroll_wheel(event):
            widget = event.widget
            while widget is not None and widget is not canvas:
                widget = getattr(widget, "master", None)
            if widget is canvas:
                delta = -1 if getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0 else 1
                canvas.yview_scroll(delta, "units")
                return "break"

        for sequence in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.root.bind(sequence, scroll_wheel, add="+")
        return content

    def _build_global_tab(self) -> None:
        outer = self._scrollable_tab("Global")
        readings = ttk.LabelFrame(outer, text="Live ODrive readings · 0.2 second refresh", style="Group.TLabelframe")
        readings.pack(fill="x", pady=(0, 6))
        columns = ("Arm", "Supply V", "Motor A", "Actual RPM", "State", "Status")
        for col, title in enumerate(columns):
            ttk.Label(readings, text=title, font=("Segoe UI", 10, "bold")).grid(row=0, column=col, padx=7, pady=3, sticky="w")
            readings.columnconfigure(col, weight=1 if col else 0)
        for unit in range(4):
            ttk.Label(readings, text=f"Arm {unit}", font=("Segoe UI", 11, "bold")).grid(row=unit + 1, column=0, padx=7, pady=4, sticky="w")
            variables = [tk.StringVar(value="—") for _ in range(5)]
            self.live_vars[unit] = variables
            for col, variable in enumerate(variables, 1):
                ttk.Label(readings, textvariable=variable, font=("Consolas", 11)).grid(row=unit + 1, column=col, padx=7, pady=4, sticky="w")
        ttk.Label(readings, text="Supply V is the ODrive input voltage. Motor A is torque-producing current (Iq); shown only in closed loop.").grid(row=5, column=0, columnspan=6, sticky="w", padx=7, pady=(4, 0))
        ttk.Label(readings, text="Missing or stale readings show —. Use CAN Status for connection details.").grid(row=6, column=0, columnspan=6, sticky="w", padx=7, pady=(2, 4))

        actuator_readings = ttk.LabelFrame(outer, text="Actuator motor current · details and drive adjustment in Unit tabs", style="Group.TLabelframe")
        actuator_readings.pack(fill="x", pady=(0, 6))
        for unit in range(4):
            ttk.Label(actuator_readings, text=f"Arm {unit}:").grid(row=0, column=2 * unit, padx=7, pady=5)
            variable = tk.StringVar(value="— A")
            self.act_global_vars[unit] = variable
            ttk.Label(actuator_readings, textvariable=variable, font=("Consolas", 11)).grid(row=0, column=2 * unit + 1, padx=7, pady=5)
            actuator_readings.columnconfigure(2 * unit + 1, weight=1)

        # The whole tab scrolls, including readings, at compact window sizes.
        tab = ttk.Frame(outer)
        tab.pack(fill="both", expand=True)

        core = ttk.LabelFrame(tab, text="Core", style="Group.TLabelframe")
        core.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        for i, (label, cmd) in enumerate([
            ("Start All", "start"),
            ("Status", "status"),
            ("Limits (20s)", "limits"),
            ("Unit IDs", "unit"),
            ("Stop All", "stop"),
        ]):
            ttk.Button(core, text=label, command=lambda c=cmd: self._run(c)).grid(
                row=i // 2, column=i % 2, sticky="ew", padx=3, pady=3
            )

        can = ttk.LabelFrame(tab, text="CAN Bus", style="Group.TLabelframe")
        can.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        for i, (label, cmd) in enumerate([
            ("CAN Init", "can init"),
            ("CAN Status", "can status"),
            ("CAN Reset", "can reset"),
            ("CAN Beacon", "can beacon"),
            ("CAN Watch (5s)", "can watch"),
        ]):
            ttk.Button(can, text=label, command=lambda c=cmd: self._run(c)).grid(
                row=i // 2, column=i % 2, sticky="ew", padx=3, pady=3
            )

        wheel = ttk.LabelFrame(tab, text="Wheels (all units)", style="Group.TLabelframe")
        wheel.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        for i, (label, cmd) in enumerate([
            ("Diag", "wheel diag"),
            ("Clear Errors", "wheel clear"),
            ("Idle", "wheel idle"),
            ("Monitor (5s)", "wheel monitor"),
            ("Read Tuning", "wheel tune"),
            ("Read Limits", "wheel current"),
        ]):
            ttk.Button(wheel, text=label, command=lambda c=cmd: self._run(c)).grid(
                row=i // 2, column=i % 2, sticky="ew", padx=3, pady=3
            )
        rpm_row = ttk.Frame(wheel)
        rpm_row.grid(row=3, column=0, columnspan=2, pady=(8, 2), sticky="ew")
        ttk.Label(rpm_row, text="RPM:").pack(side="left")
        self.all_rpm_var = tk.StringVar(value="10")
        ttk.Entry(rpm_row, textvariable=self.all_rpm_var, width=8).pack(side="left", padx=4)
        ttk.Button(
            rpm_row, text="Go", command=lambda: self._run(f"all {self.all_rpm_var.get()}")
        ).pack(side="left", padx=2)
        ttk.Button(
            rpm_row, text="Reverse", command=lambda: self._run(f"all -{self.all_rpm_var.get()}")
        ).pack(side="left", padx=2)
        ttk.Button(rpm_row, text="Stop", command=lambda: self._run("all 0")).pack(
            side="left", padx=2
        )

        act = ttk.LabelFrame(tab, text="Actuators (all units)", style="Group.TLabelframe")
        act.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)
        ttk.Button(act, text="Clear All Actuator Errors", command=self._clear_actuator_errors).grid(
            row=0, column=0, columnspan=3, sticky="ew", padx=3, pady=3)
        ttk.Label(act, text="Stops all motors, resets actuator software latches, then checks fresh readings.\nController stays disabled. Hardware faults may remain.", wraplength=460).grid(
            row=1, column=0, columnspan=3, sticky="w", padx=3, pady=3)
        for i, (label, cmd) in enumerate([
            ("Diag", "act diag"),
            ("Encoders", "act enc"),
            ("Enc Watch", "act enc watch"),
            ("Enc Reset", "act enc reset"),
            ("Reset Link", "act reset"),
            ("Safety Timeout", "act safety"),
            ("Scan Settings", "act scan"),
            ("Use M1", "act use m1"),
            ("Use M2", "act use m2"),
            ("Pulse Fwd", "act pulse fwd"),
            ("Pulse Rev", "act pulse rev"),
            ("Fwd \u2192 Limit", "act fwd"),
            ("Rev \u2192 Limit", "act rev"),
            ("Center", "act center"),
            ("Stop", "act stop"),
        ]):
            ttk.Button(act, text=label, command=lambda c=cmd: self._run(c)).grid(
                row=2 + i // 3, column=i % 3, sticky="ew", padx=3, pady=3
            )

        for col in range(2):
            tab.columnconfigure(col, weight=1)

    def _build_unit_tab(self, i: int) -> None:
        tab = self._scrollable_tab(f"Unit {i}")

        wheel = ttk.LabelFrame(tab, text=f"Wheel (u{i})", style="Group.TLabelframe")
        wheel.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        for r, (label, cmd) in enumerate([
            ("Diag", f"w{i} diag"),
            ("Clear Errors", f"w{i} clear"),
            ("Monitor (5s)", f"w{i} monitor"),
            ("Turn +1 Enc", f"w{i} turn 1"),
            ("Jog Test", f"w{i} test"),
        ]):
            ttk.Button(wheel, text=label, command=lambda c=cmd: self._run(c)).grid(
                row=r, column=0, columnspan=2, sticky="ew", padx=3, pady=2
            )
        rpm_row = ttk.Frame(wheel)
        rpm_row.grid(row=5, column=0, columnspan=2, pady=(8, 2))
        ttk.Label(rpm_row, text="RPM:").pack(side="left")
        rpm_var = tk.StringVar(value="10")
        ttk.Entry(rpm_row, textvariable=rpm_var, width=8).pack(side="left", padx=4)
        ttk.Button(rpm_row, text="Go", command=lambda v=rpm_var, u=i: self._run(f"w{u} {v.get()}")).pack(
            side="left", padx=2
        )
        ttk.Button(
            rpm_row, text="Reverse", command=lambda v=rpm_var, u=i: self._run(f"w{u} -{v.get()}")
        ).pack(side="left", padx=2)
        ttk.Button(rpm_row, text="Stop", command=lambda u=i: self._run(f"w{u} 0")).pack(
            side="left", padx=2
        )

        tuning = ttk.LabelFrame(wheel, text="CAN Tuning & Limits (35 A / 50 A)", style="Group.TLabelframe")
        tuning.grid(row=6, column=0, columnspan=2, sticky="ew", padx=3, pady=(8, 2))
        tuning.columnconfigure(0, weight=1)
        tuning.columnconfigure(1, weight=1)
        ttk.Button(tuning, text="Read Tuning & Bus", command=lambda u=i: self._run(f"w{u} tune")).grid(
            row=0, column=0, sticky="ew", padx=3, pady=2
        )
        ttk.Button(tuning, text="Read Current Limits", command=lambda u=i: self._run(f"w{u} current")).grid(
            row=0, column=1, sticky="ew", padx=3, pady=2
        )
        ttk.Button(
            tuning,
            text="Apply Climb Preset (RAM)",
            command=lambda u=i: self._confirm_wheel_action(
                u,
                f"Apply climb preset on u{u}",
                f"Apply the climbing velocity/spinout preset + 35 A / 50 A limits to unit {u} in RAM?\n\n"
                "ODrive must be in healthy IDLE. Gamepad control will be disabled.",
                f"w{u} tune apply",
            ),
        ).grid(row=1, column=0, sticky="ew", padx=3, pady=2)
        ttk.Button(
            tuning,
            text="Save Tuning to Flash",
            command=lambda u=i: self._confirm_wheel_action(
                u,
                f"Save u{u} ODrive flash",
                f"Save verified 35 A / 50 A limits and tuning to unit {u} ODrive flash?\n\n"
                "The ODrive will reboot and briefly drop CAN heartbeat. Gamepad control will be disabled.",
                f"w{u} tune save",
            ),
        ).grid(row=1, column=1, sticky="ew", padx=3, pady=2)
        ttk.Button(
            tuning,
            text="Calibrate Encoder (State 7)",
            command=lambda u=i: self._confirm_wheel_action(
                u,
                f"Calibrate u{u} wheel encoder",
                f"Run guarded encoder offset calibration (State 7) on unit {u} in RAM?\n\n"
                "WARNING: Wheel will physically rotate. Ensure the robot is supported, wheel is unloaded, and hands are clear.",
                f"w{u} cal",
            ),
        ).grid(row=2, column=0, columnspan=2, sticky="ew", padx=3, pady=2)
        param_row = ttk.Frame(tuning)
        param_row.grid(row=3, column=0, columnspan=2, sticky="ew", padx=3, pady=(4, 2))
        param_var = tk.StringVar(value="vel_ramp_rate")
        ttk.Combobox(
            param_row,
            textvariable=param_var,
            values=tuple(WHEEL_TUNING_PARAMS.keys()),
            state="readonly",
            width=20,
        ).pack(side="left", padx=(0, 4))
        val_var = tk.StringVar()
        ttk.Entry(param_row, textvariable=val_var, width=8).pack(side="left", padx=(0, 4))
        ttk.Button(
            param_row,
            text="Set (RAM)",
            command=lambda u=i, p=param_var, v=val_var: self._set_wheel_tune_param(u, p.get(), v.get()),
        ).pack(side="left")

        act = ttk.LabelFrame(tab, text=f"Actuator (u{i})", style="Group.TLabelframe")
        act.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        for r, (label, cmd) in enumerate([
            ("Forward \u2192 Limit", f"act{i} fwd"),
            ("Reverse \u2192 Limit", f"act{i} rev"),
            ("Center", f"act{i} center"),
            ("Stop", f"act{i} stop"),
        ]):
            ttk.Button(act, text=label, command=lambda c=cmd: self._run(c)).grid(
                row=r, column=0, sticky="ew", padx=3, pady=2
            )

        detail = ttk.LabelFrame(act, text="Live actuator readings", style="Group.TLabelframe")
        detail.grid(row=4, column=0, sticky="ew", padx=3, pady=6)
        labels = ("Supply V", "Motor A", "Output PWM %", "Controller limit A (last read)", "Commanded drive %", "Manual ceiling %", "Status")
        variables = [tk.StringVar(value="—") for _ in labels]
        self.act_vars[i] = variables
        for row, (label, variable) in enumerate(zip(labels, variables)):
            ttk.Label(detail, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=2)
            ttk.Label(detail, textvariable=variable, wraplength=280).grid(row=row, column=1, sticky="w", padx=4, pady=2)
        effort = ttk.Frame(act)
        effort.grid(row=5, column=0, sticky="ew", padx=3, pady=3)
        ttk.Label(effort, text="Manual power (1–60 / 127):").pack(side="left")
        power_var = tk.StringVar(value="40")
        ttk.Spinbox(effort, from_=1, to=60, increment=4, textvariable=power_var, width=5).pack(side="left", padx=4)
        ttk.Button(effort, text="Apply while stopped", command=lambda u=i, v=power_var: self._set_actuator_power(u, v.get())).pack(side="left")
        ttk.Label(act, text="Default 40 = 31.5%. Maximum 60 = 47.2%. Apply changes the next manual move only; current depends on load.\nTarget ceiling: 6.75 A; verify Controller limit A. Above 40 requires fresh readings and a verified limit.", wraplength=500).grid(row=6, column=0, sticky="w", padx=4, pady=3)
        ttk.Button(act, text="Read controller settings", command=lambda u=i: self._run(f"u{u} act config")).grid(row=7, column=0, sticky="ew", padx=3, pady=3)
        ttk.Button(act, text="Apply 6.75 A ceiling while stopped", command=lambda u=i: self._run(f"u{u} act current")).grid(row=8, column=0, sticky="ew", padx=3, pady=3)
        ttk.Button(act, text="Read current / regen limits", command=lambda u=i: self._run(f"u{u} act limits")).grid(row=9, column=0, sticky="ew", padx=3, pady=3)
        ttk.Button(act, text="Read battery voltage limits", command=lambda u=i: self._run(f"u{u} act voltage")).grid(row=10, column=0, sticky="ew", padx=3, pady=3)

        raw = ttk.LabelFrame(tab, text=f"Raw command to u{i}", style="Group.TLabelframe")
        raw.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=(10, 5))
        raw_var = tk.StringVar()
        entry = ttk.Entry(raw, textvariable=raw_var, width=40)
        entry.pack(side="left", fill="x", expand=True, padx=(0, 5))
        entry.bind(
            "<Return>", lambda e, v=raw_var, u=i: (self._run(f"u{u} {v.get()}"), v.set(""))
        )
        ttk.Button(
            raw, text="Send", command=lambda v=raw_var, u=i: (self._run(f"u{u} {v.get()}"), v.set(""))
        ).pack(side="left")

        for col in range(2):
            tab.columnconfigure(col, weight=1)

    def _build_raw_tab(self) -> None:
        tab = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(tab, text="Raw / Broadcast")

        ttk.Label(
            tab,
            text="Type any supervisor command exactly as you would in the CLI\n"
                 "(e.g. 'broadcast tof read', 'wheel idle', 'act0 fwd').",
            justify="left",
        ).pack(anchor="w", pady=(0, 10))

        row = ttk.Frame(tab)
        row.pack(fill="x")
        self.raw_all_var = tk.StringVar()
        entry = ttk.Entry(row, textvariable=self.raw_all_var, width=60)
        entry.pack(side="left", fill="x", expand=True, padx=(0, 5))
        entry.bind("<Return>", lambda e: self._send_raw_all())
        ttk.Button(row, text="Send", command=self._send_raw_all).pack(side="left")

    def _send_raw_all(self) -> None:
        cmd = self.raw_all_var.get()
        if cmd.strip():
            self._run(cmd)
            self.raw_all_var.set("")

    def _emergency_stop(self) -> None:
        # Force-disable controller control first so it can't immediately
        # re-issue a move command right after the stop.
        self._stop_controller()

    # ------------------------------------------------------------------ #
    # Controller tab
    # ------------------------------------------------------------------ #
    def _build_controller_tab(self) -> None:
        content = self._scrollable_tab("Controller")

        if not PYGAME_AVAILABLE:
            ttk.Label(
                content,
                text="pygame is not installed. Run:  pip install pygame\n"
                     "then restart this GUI to enable controller support.",
                foreground="#a33",
            ).pack(anchor="w", padx=10, pady=10)
            return

        conn = ttk.LabelFrame(content, text="Controller Connection", style="Group.TLabelframe")
        conn.pack(fill="x", pady=(0, 10))
        ttk.Button(conn, text="Connect Controller", command=self._connect_controller).grid(
            row=0, column=0, padx=3, pady=3
        )
        ttk.Button(conn, text="Disconnect Controller", command=self._disconnect_controller).grid(
            row=0, column=1, padx=3, pady=3
        )
        self.controller_status_var = tk.StringVar(value="Not connected")
        ttk.Label(conn, textvariable=self.controller_status_var, foreground="#555").grid(
            row=0, column=2, padx=(15, 3)
        )
        self.controller_enable_chk = ttk.Checkbutton(
            conn, text="Controller control ENABLED", variable=self.controller_enabled,
            command=self._controller_enable_changed,
        )
        self.controller_enable_chk.grid(row=0, column=3, padx=(25, 3))

        settings = ttk.LabelFrame(content, text="Wheel Speed Settings", style="Group.TLabelframe")
        settings.pack(fill="x", pady=(0, 10))
        ttk.Label(settings, text="Speed increment per press:").grid(row=0, column=0, sticky="w")
        ttk.Entry(settings, textvariable=self.speed_increment_var, width=8).grid(row=0, column=1, padx=5)
        ttk.Label(settings, text="Max wheel RPM (cap):").grid(row=0, column=2, sticky="w", padx=(15, 0))
        ttk.Entry(settings, textvariable=self.max_wheel_speed_var, width=8).grid(row=0, column=3, padx=5)
        ttk.Label(settings, text="Current speed:").grid(row=0, column=4, sticky="w", padx=(15, 0))
        ttk.Label(settings, textvariable=self.current_speed_var, foreground="#06c", width=10).grid(
            row=0, column=5, padx=5
        )

        mapping = ttk.LabelFrame(content, text="Controller Mapping — buttons, triggers and D-pad", style="Group.TLabelframe")
        mapping.pack(fill="x", pady=(0, 10))

        def add_mapping_row(row: int, title: str, key: str):
            ttk.Label(mapping, text=title).grid(row=row, column=0, sticky="w", padx=3, pady=4)
            var = tk.StringVar(value="not set")
            self.map_value_vars[key] = var
            ttk.Label(mapping, textvariable=var, width=32, foreground="#06c").grid(
                row=row, column=1, sticky="w", padx=3
            )
            ttk.Button(mapping, text="Learn", command=lambda k=key: self._start_learn(k)).grid(
                row=row, column=2, padx=3
            )
            ttk.Button(mapping, text="Clear", command=lambda k=key: self._clear_mapping(k)).grid(
                row=row, column=3, padx=3)

        row = 0
        for i in range(4):
            add_mapping_row(row, f"Select actuator {i}", f"actuator_select_{i}")
            row += 1
        add_mapping_row(row, "Select actuator pair 0 + 2", "actuator_pair_02")
        row += 1
        add_mapping_row(row, "Select actuator pair 1 + 3", "actuator_pair_13")
        row += 1
        add_mapping_row(row, "Actuators EXTEND (selected arm/pair, hold)", "actuator_pulse_fwd")
        row += 1
        add_mapping_row(row, "Actuators RETRACT (selected arm/pair, hold)", "actuator_pulse_rev")
        row += 1
        add_mapping_row(row, "Wheels FORWARD (all, hold)", "wheel_fwd")
        row += 1
        add_mapping_row(row, "Wheels REVERSE (all, hold)", "wheel_rev")
        row += 1
        add_mapping_row(row, "Speed INCREASE", "speed_increase")
        row += 1
        add_mapping_row(row, "Speed DECREASE", "speed_decrease")
        row += 1
        add_mapping_row(row, "Stop (all wheels + actuators)", "stop")
        row += 1

        ttk.Button(mapping, text="Reset All Mapping", command=self._reset_mapping).grid(
            row=row, column=0, pady=(10, 0), sticky="w"
        )

        ttk.Label(mapping, text="Release every control, click Learn, then fully press and release one button, trigger or D-pad direction.\n"
                  "Learning disables robot control. Clear a conflicting mapping before reassigning it.",
                  wraplength=840).grid(row=row+1, column=0, columnspan=4, sticky="w", pady=6)
        self.mapping_status_var = tk.StringVar()
        ttk.Label(mapping, textvariable=self.mapping_status_var, foreground="#a33", wraplength=840).grid(
            row=row+2, column=0, columnspan=4, sticky="w")
        ttk.Button(mapping, text="Cancel Learn", command=self._cancel_learn).grid(row=row, column=2, columnspan=2)

        targets = ttk.LabelFrame(content, text="Select actuators (selection only)", style="Group.TLabelframe")
        targets.pack(fill="x", pady=(0, 10))
        for column, group in enumerate(((0,), (1,), (2,), (3,), (0, 2), (1, 3))):
            ttk.Button(targets, text=" + ".join(map(str, group)),
                       command=lambda g=group: self._choose_actuators_from_gui(g)).grid(row=0, column=column, padx=4, pady=4)

        live = ttk.LabelFrame(content, text="Live Controller Inputs", style="Group.TLabelframe")
        live.pack(fill="x")
        ttk.Label(live, text="Selected actuator:").pack(anchor="w", padx=3, pady=(3, 0))
        ttk.Label(live, textvariable=self.selected_actuator_var, foreground="#06c").pack(
            anchor="w", padx=3, pady=(0, 5)
        )
        self.telemetry_var = tk.StringVar(value="Connect a controller to see live input here.")
        ttk.Label(live, textvariable=self.telemetry_var, wraplength=850, justify="left").pack(
            anchor="w", padx=3, pady=3
        )

        self._refresh_mapping_labels()

    def _refresh_mapping_labels(self) -> None:
        for key, variable in self.map_value_vars.items():
            variable.set(controls.format_binding(controls.get_binding(self.mapping, key)))
        if hasattr(self, "mapping_status_var"):
            self.mapping_status_var.set("; ".join(controls.conflicts(self.mapping)) or "No conflicting mappings.")

    def _clear_mapping(self, key: str) -> None:
        self._stop_controller()
        self._cancel_learn()
        controls.set_binding(self.mapping, key, None)
        self._save_controller_mapping()
        self._refresh_mapping_labels()

    def _cancel_learn(self) -> None:
        self.awaiting_learn = None
        self._learner = None

    def _save_controller_mapping(self) -> None:
        try:
            save_mapping(self.mapping)
        except RuntimeError as exc:
            self.output_queue.put(f"[controller] {exc}")
            if hasattr(self, "mapping_status_var"):
                self.mapping_status_var.set(str(exc))

    def _reset_mapping(self) -> None:
        self._stop_controller()
        self.awaiting_learn = None
        self.mapping = copy.deepcopy(DEFAULT_MAPPING)
        self._save_controller_mapping()
        self._refresh_mapping_labels()
        self.output_queue.put("[controller] Mapping reset.")

    def _connect_controller(self) -> None:
        self._stop_controller()
        self.awaiting_learn = None
        self.prev_buttons = []
        self._previous_actions = {}
        self._input_snapshot = ([], [], [])
        ok, info = self.controller.connect()
        if ok:
            self.controller_status_var.set(f"Connected: {info}")
            self.output_queue.put(f"[controller] Connected: {info}")
        else:
            self.controller_status_var.set("Not connected")
            messagebox.showerror("Controller", info)

    def _disconnect_controller(self) -> None:
        self._stop_controller()
        self.controller.disconnect()
        self.awaiting_learn = None
        self.prev_buttons = []
        self.controller_status_var.set("Not connected")
        self.selected_actuator = None
        self.selected_actuators = ()
        self._previous_actions = {}
        self.selected_actuator_var.set("None selected")
        self.output_queue.put("[controller] Disconnected.")

    def _controller_notice(self, text: str) -> None:
        variable = getattr(self, "control_notice_var", None)
        if variable is not None:
            variable.set(text)

    def _stop_controller(self, *, send_stop=True) -> None:
        pending = getattr(self, "_act_recovery", None)
        if pending is not None:
            pending.cancel()
        self.controller_enabled.set(False)
        self._controller_notice("Controller disabled — enable control when ready.")
        self._controller_wait_for_release = True
        self._active_actuator_pulse = None
        self.last_wheel_rpm_sent = 0.0
        if send_stop and self.units and not self._run_silent("stop"):
            self.output_queue.put("[controller] Stop delivery failed. Use the physical cutoff.")

    def _controller_enable_changed(self) -> None:
        pending = getattr(self, "_act_recovery", None)
        if pending is not None:
            self.controller_enabled.set(False)
            self._controller_notice("Actuator recovery in progress; controller remains disabled.")
            return
        if not self.controller_enabled.get():
            self._stop_controller()
        elif self.controller.joystick is None or self.awaiting_learn is not None:
            self._stop_controller()
            self.output_queue.put("[controller] Connect a controller and finish mapping before enabling.")
        else:
            errors = self._mapping_errors()
            if errors:
                self._stop_controller()
                self.output_queue.put("[controller] Fix mapping before enabling: " + "; ".join(errors))
                self._controller_notice("Controller disabled: " + "; ".join(errors))
                return
            self._controller_wait_for_release = True
            self.output_queue.put("[controller] Release buttons, triggers and D-pad before moving.")
            self._controller_notice("Controller enabled — release all buttons, triggers and D-pad first.")

    def _send_controller_command(self, command: str) -> None:
        if not self._run_silent(command):
            raise ControllerCommandError("Robot command could not be sent; controller control disabled.")

    def _mapping_errors(self) -> list:
        errors = controls.conflicts(self.mapping)
        axes, buttons, hats = self._input_snapshot
        errors.extend(controls.available_errors(self.mapping, axes, buttons, hats))
        if controls.get_binding(self.mapping, "stop") is None:
            errors.append("Learn a dedicated Stop control")
        return errors

    def _start_learn(self, key: str) -> None:
        if self.controller.joystick is None:
            messagebox.showwarning("Controller", "Connect the controller first.")
            return
        self._stop_controller()
        self.awaiting_learn = key
        self._learner = controls.BindingLearner(*self._input_snapshot)
        self.output_queue.put(f"[controller] Learning '{controls.LABELS[key]}': fully press and release one button, trigger or D-pad direction...")

    def _finish_learn(self, binding) -> None:
        key = self.awaiting_learn
        observed = copy.deepcopy(self.mapping)
        controls.register_aliases(observed, self._learner.aliases)
        proposed = copy.deepcopy(observed)
        controls.set_binding(proposed, key, binding)
        errors = controls.conflicts(proposed)
        # Existing unrelated conflicts must not prevent fixing one row.
        old_errors = set(controls.conflicts(self.mapping))
        introduced = [e for e in errors if e not in old_errors]
        if introduced:
            # Keep discovered physical aliases even when rejecting a binding.
            self.mapping = observed
            self._save_controller_mapping()
            self.output_queue.put("[controller] Mapping not changed: " + "; ".join(introduced) + ". Clear the conflicting row first.")
        else:
            self.mapping = proposed
            self._save_controller_mapping()
            self.output_queue.put(f"[controller] Learned {controls.LABELS[key]}: {controls.format_binding(binding)}")
            self._log_event("controller_mapping", action=key, binding=binding)
        self._cancel_learn()
        self._refresh_mapping_labels()

    def _controller_tick(self) -> None:
        if self._closing:
            return
        try:
            self._poll_controller()
        except ControllerCommandError as exc:
            self._stop_controller()
            self._controller_notice("Robot command failed — controller remains connected; control disabled. See console.")
            self.output_queue.put(f"[controller] {exc}")
        except Exception as exc:
            self._stop_controller()
            self.controller.disconnect()
            self.awaiting_learn = None
            self.prev_buttons = []
            self.controller_status_var.set("Disconnected (input error)")
            self._controller_notice(f"Controller disconnected: {exc}")
            self.output_queue.put(f"[controller] {exc}")
        finally:
            if not self._closing:
                self._controller_after_id = self.root.after(30, self._controller_tick)

    def _poll_controller(self) -> None:
        if not PYGAME_AVAILABLE or self.controller.joystick is None:
            return
        axes, buttons, hats = self.controller.poll()
        self._input_snapshot = (axes, buttons, hats)
        previous = self._previous_actions
        states = controls.evaluate(self.mapping, axes, buttons, hats, previous)
        if self.awaiting_learn is not None and self._learner is not None:
            binding = self._learner.sample(axes, buttons, hats)
            if binding is not None:
                self._finish_learn(binding)

        if hasattr(self, "telemetry_var"):
            axes_txt = ", ".join(f"A{i}:{v:+.2f}" for i, v in enumerate(axes))
            down = [i for i, p in enumerate(buttons) if p]
            hats_txt = ", ".join(f"H{i}:{v}" for i, v in enumerate(hats)) or "none"
            actions = ", ".join(controls.LABELS[k] for k, v in states.items() if v) or "none"
            self.telemetry_var.set(f"Axes: {axes_txt}\nButtons down: {down}  D-pad hats: {hats_txt}\nActive mappings: {actions}")

        if self.controller_enabled.get() and self.units and self.awaiting_learn is None:
            if states.get("stop"):
                self._stop_controller()
                self.output_queue.put("[controller] Stopped; re-enable controller control to resume.")
            elif self._mapping_errors():
                self._stop_controller()
                self.output_queue.put("[controller] Mapping invalid; stopped. Check the Controller tab.")
            elif self._controller_wait_for_release:
                if not any(states.values()) and not any(buttons) and not any(x or y for x, y in hats):
                    self._controller_wait_for_release = False
                    self._controller_notice("Controller enabled — choose an available arm/pair, then hold extend or retract.")
            else:
                self._apply_speed_adjust(states)
                self._stop_released_motion(states)
                if not self._controller_wait_for_release:
                    self._apply_actuator_selection(states)
                if not self._controller_wait_for_release:
                    self._apply_actuator_pulses(states)
                if not self._controller_wait_for_release:
                    self._apply_wheel_buttons(states)
        self.prev_buttons = buttons
        self._previous_actions = states

    @staticmethod
    def _button_down(idx: int | None, buttons_list: list) -> bool:
        return bool(type(idx) is int and 0 <= idx < len(buttons_list) and buttons_list[idx])

    def _pressed(self, action, states) -> bool:
        return bool(states.get(action) and not self._previous_actions.get(action))

    def _motion_stop(self, reason) -> None:
        # Global Stop must not be followed by another held command in this tick.
        self._active_actuator_pulse = None
        self.last_wheel_rpm_sent = 0.0
        self._controller_wait_for_release = True
        self._send_controller_command("stop")
        self.output_queue.put(f"[controller] Stop ({reason}); release controls before moving again.")
        self._controller_notice(f"Stopped: {reason}. Release controls; then select an available arm/pair.")

    def _stop_released_motion(self, states) -> None:
        # Stop existing motion before considering any new movement this frame.
        if self.last_wheel_rpm_sent:
            fwd, rev = states.get("wheel_fwd"), states.get("wheel_rev")
            sign = (1 if fwd else -1) if bool(fwd) != bool(rev) else 0
            old_sign = 1 if self.last_wheel_rpm_sent > 0 else -1
            if sign != old_sign or self.current_wheel_speed == 0:
                self._motion_stop("wheel control released or direction changed")
                return
        if self._active_actuator_pulse is not None:
            fwd, rev = states.get("actuator_pulse_fwd"), states.get("actuator_pulse_rev")
            direction = ("fwd" if fwd else "rev") if bool(fwd) != bool(rev) else None
            if direction != self._active_actuator_pulse[1]:
                self._motion_stop("actuator control released or direction changed")

    def _choose_actuators_from_gui(self, group) -> None:
        try:
            self._choose_actuators(group)
        except Exception as exc:
            self._stop_controller()
            self.output_queue.put(f"[controller] Selection failed: {exc}")

    def _choose_actuators(self, group) -> None:
        group = tuple(group)
        if group == self.selected_actuators:
            return
        if self._active_actuator_pulse is not None or self.last_wheel_rpm_sent:
            self._motion_stop("actuator selection changed")
        # Selection never starts an already-held motion control.
        self._controller_wait_for_release = True
        self.selected_actuators = group
        self.selected_actuator = group[0] if len(group) == 1 else None
        label = "Actuator " if len(group) == 1 else "Actuators "
        label += " + ".join(map(str, group))
        self.selected_actuator_var.set(label)
        self.output_queue.put(f"[controller] Selected {label.lower()}")
        self._log_event("controller_selection", actuators=list(group))

    def _apply_actuator_selection(self, states) -> None:
        choices = [(i,) for i in range(4) if self._pressed(f"actuator_select_{i}", states)]
        choices += [g for action, g in (("actuator_pair_02", (0, 2)), ("actuator_pair_13", (1, 3)))
                    if self._pressed(action, states)]
        if len(choices) == 1:
            self._choose_actuators(choices[0])
        elif len(choices) > 1:
            self._motion_stop("multiple actuator selections")

    def _apply_actuator_pulses(self, states) -> None:
        fwd = states.get("actuator_pulse_fwd", False)
        rev = states.get("actuator_pulse_rev", False)
        group = self.selected_actuators
        active = self._active_actuator_pulse
        if fwd and rev:
            self._motion_stop("opposing actuator controls")
            return
        desired = (group, "fwd" if fwd else "rev") if group and (fwd or rev) else None
        if not group and (fwd or rev):
            self._motion_stop("no actuator selected")
            self.output_queue.put("[controller] Select an actuator or pair first, then press extend/retract.")
            return
        if desired == active:
            return
        if active is not None:
            self._motion_stop("actuator control released or direction changed")
            return
        if desired is None:
            return
        connected = {u.index for u in self.units if self._unit_available(u)}
        missing = set(group) - connected
        if missing:
            detail = "Selected actuator(s) unavailable: " + ", ".join(map(str, sorted(missing)))
            self._motion_stop(detail)
            self.output_queue.put("[controller] Pair not started. Gamepad remains connected; select an available arm after releasing controls, or reconnect missing USB units.")
            return
        _, direction = desired
        # Both destinations are validated before either is sent. If any write
        # fails, _controller_tick disables control and attempts Stop on all units.
        for unit in group:
            self._send_controller_command(f"act{unit} {direction}")
        self._active_actuator_pulse = desired
        self.output_queue.put(f"[controller] Actuators {' + '.join(map(str, group))} {direction}")

    def _apply_speed_adjust(self, states) -> None:
        increment = float(self.speed_increment_var.get())
        max_speed = float(self.max_wheel_speed_var.get())
        if not math.isfinite(increment) or increment <= 0:
            raise ValueError("Speed increment must be a positive, finite number.")
        if not math.isfinite(max_speed) or max_speed < 0:
            raise ValueError("Max wheel RPM must be a nonnegative, finite number.")
        previous_speed = self.current_wheel_speed
        self.current_wheel_speed = min(max_speed, max(0.0, self.current_wheel_speed))
        if self._pressed("speed_increase", states):
            self.current_wheel_speed = min(max_speed, self.current_wheel_speed + increment)
        if self._pressed("speed_decrease", states):
            self.current_wheel_speed = max(0.0, self.current_wheel_speed - increment)
        if self.current_wheel_speed != previous_speed:
            self.current_speed_var.set(f"{self.current_wheel_speed:g} RPM")
            self.output_queue.put(f"[controller] Wheel speed set to {self.current_wheel_speed:g} RPM")

    def _apply_wheel_buttons(self, states) -> None:
        fwd = states.get("wheel_fwd", False)
        rev = states.get("wheel_rev", False)
        if fwd and rev:
            self._motion_stop("opposing wheel controls")
            return
        rpm = self.current_wheel_speed * (1 if fwd else -1) if (fwd or rev) else 0.0
        if rpm != self.last_wheel_rpm_sent:
            if rpm == 0.0:
                self._motion_stop("wheel control released")
            else:
                self._send_controller_command(f"all {rpm}")
                self.last_wheel_rpm_sent = rpm

    # ------------------------------------------------------------------ #
    # Connection handling
    # ------------------------------------------------------------------ #
    def _auto_detect(self) -> None:
        ports = sup.likely_usb_serial_ports()
        if not ports:
            messagebox.showinfo("Auto-detect", "No likely ESP32/Arduino USB serial ports found.")
            return
        self.ports_var.set(", ".join(ports))

    def _list_ports(self) -> None:
        for p in sup.list_ports.comports():
            self.output_queue.put(f"{p.device}\t{p.description}")
        if not list(sup.list_ports.comports()):
            self.output_queue.put("No serial ports found.")

    def _connect(self) -> None:
        self.live_store.clear()
        self.act_store.clear()
        raw_ports = [p.strip() for p in self.ports_var.get().split(",") if p.strip()]
        if not raw_ports:
            messagebox.showwarning("No ports", "Enter ports manually or click Auto-detect first.")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Invalid baud", "Baud rate must be an integer.")
            return

        self.connect_btn.config(state="disabled")
        self.status_var.set("Connecting...")

        def worker():
            old_stdout = sys.stdout
            sys.stdout = QueueWriter(self.output_queue)
            try:
                units = sup.open_units(
                    raw_ports,
                    baud,
                    self.output_queue,
                    discover=self.discover_var.get(),
                    allow_unknown=self.allow_unknown_var.get(),
                    line_handler=self._feed_live,
                )
            except Exception as exc:  # surfaces duplicate-ID errors etc.
                self.output_queue.put(f"[error] {exc}")
                units = []
            finally:
                sys.stdout = old_stdout
            self.root.after(0, self._on_connected, units)

        threading.Thread(target=worker, daemon=True).start()

    def _on_connected(self, units: list) -> None:
        self.units = units
        try:
            usb_ports = {os.path.realpath(p.device): p.serial_number for p in sup.list_ports.comports()}
        except Exception:
            usb_ports = {}
        mapping = [{"unit": u.index, "port": u.port, "identified": u.identified,
                    "odrive_node": u.odrive_node_id,
                    "usb_serial": usb_ports.get(os.path.realpath(u.port))} for u in units]
        self._log_event("unit_mapping", units=mapping)
        self.output_queue.put("[logging] Unit/USB mapping: " + json.dumps(mapping, sort_keys=True))

        if units:
            labels = ", ".join(f"u{u.index}={u.port}" for u in units)
            missing = sorted(set(range(4)) - {u.index for u in units if u.identified})
            suffix = " | MISSING: " + ", ".join(f"u{i}" for i in missing) if missing else ""
            self.status_var.set(f"Connected: {labels}{suffix}")
            if missing:
                self._controller_notice("USB units missing: " + ", ".join(map(str, missing)) + ". Pairs containing them cannot start.")
            self.disconnect_btn.config(state="normal")
        else:
            self.status_var.set("Not connected (no units opened)")
            self.connect_btn.config(state="normal")

    def _disconnect(self) -> None:
        self._stop_controller()
        self._log_event("disconnect", units=[getattr(u, "index", None) for u in self.units])
        for u in self.units:
            try:
                u.close()
            except Exception:
                pass
        self.units = []
        self.live_store.clear()
        self.act_store.clear()
        self.status_var.set("Not connected")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")

    # ------------------------------------------------------------------ #
    # Command dispatch — routes through your existing parser untouched
    # ------------------------------------------------------------------ #
    def _run(self, command: str) -> None:
        if command.strip().lower() in ("stop", "s", "estop"):
            self._stop_controller()
            return
        if not self.units:
            messagebox.showwarning("Not connected", "Connect to units first.")
            return
        self.output_queue.put(f"> {command}")
        self._run_silent(command)

    def _run_silent(self, command: str) -> bool:
        """Same dispatch as _run, but no popup and no '> ' echo — used by
        the controller loop which may fire many times per second."""
        if not self.units:
            return False
        normalized = command.strip().lower()
        pending = getattr(self, "_act_recovery", None)
        if pending is not None:
            if normalized in ("stop", "s", "estop"):
                pending.cancel()
            else:
                self.output_queue.put("[actuator recovery] Wait for recovery to finish, or press Stop to cancel.")
                return False
        self._log_event("command_request", command=command, units=[getattr(u, "index", None) for u in self.units])
        old_stdout = sys.stdout
        sys.stdout = QueueWriter(self.output_queue)
        succeeded = True
        try:
            # A disconnected unit must not prevent a stop reaching the rest.
            groups = [[unit] for unit in self.units] if normalized in ("stop", "s", "estop", "act stop", "act all stop", "wheel clear", "wheel tune", "wheel current", "all 0", "wall 0", "wheel 0", "wheel idle", "wstop") else [self.units]
            for group in groups:
                try:
                    sup.handle_supervisor_command(group, command)
                except Exception as exc:
                    succeeded = False
                    self.output_queue.put(f"[error] {exc}")
        finally:
            sys.stdout = old_stdout
        self._log_event("command_dispatch_result", command=command, succeeded=succeeded)
        return succeeded

    # ------------------------------------------------------------------ #
    # Console polling
    # ------------------------------------------------------------------ #
    def _log_event(self, event: str, **fields) -> None:
        logger = getattr(self, "session_log", None)
        if logger is None:
            return
        try:
            logger.record(event, **fields)
        except Exception:
            # Even an unexpected logger failure cannot prevent command dispatch.
            if not getattr(self, "_logging_warning_shown", False):
                self._logging_warning_shown = True
                self.output_queue.put("[logging] Diagnostic logging unavailable; controls remain available.")

    def _feed_live(self, unit: int, text: str) -> bool:
        # Save raw fragments before the live parsers hide telemetry from console.
        # Non-telemetry fault lines pass through this same per-unit hook.
        self._log_event("serial_receive", unit=unit, text=text)
        pending = getattr(self, "_act_recovery", None)
        if pending is not None:
            pending.feed(unit, text)
        return self.act_store.feed(unit, text) or self.live_store.feed(unit, text)

    @staticmethod
    def _unit_available(unit):
        failed = getattr(unit, "stop_reader", None)
        return bool(unit.identified and unit.serial_port.is_open and
                    (failed is None or not failed.is_set()))

    def _clear_actuator_errors(self):
        pending = getattr(self, "_act_recovery", None)
        if pending is not None and not pending.done.is_set():
            self.output_queue.put("[actuator recovery] Already in progress.")
            return
        # State changes stay on Tk's thread; serial Stop and verification run
        # in the worker so a failed board cannot freeze the GUI.
        self._stop_controller(send_stop=False)
        units = [u for u in self.units if self._unit_available(u)]
        missing = sorted(set(range(4)) - {u.index for u in units})
        for index in missing:
            self.output_queue.put(f"[actuator recovery u{index}] Unavailable over USB; not recovered. Reconnect this unit.")
        if not units:
            return
        def report(index, result):
            self.output_queue.put(f"[actuator recovery u{index}] {result}")
            self._log_event("actuator_recovery_result", unit=index, result=result)
        self._act_recovery = recovery.Recovery(units, self.act_store.sample, report)
        self.output_queue.put("[actuator recovery] Sending Stop, then verifying each actuator. Controller stays disabled; select/re-enable before another move.")
        self._controller_notice("Actuator recovery in progress; controller disabled.")
        self._log_event("actuator_recovery_start", units=[u.index for u in units], missing=missing)
        self._act_recovery.start()
        self._recovery_after_id = self.root.after(100, self._poll_actuator_recovery)

    def _poll_actuator_recovery(self):
        pending = getattr(self, "_act_recovery", None)
        if getattr(self, "_closing", False) or pending is None:
            return
        if not pending.done.is_set():
            self._recovery_after_id = self.root.after(100, self._poll_actuator_recovery)
            return
        self._recovery_after_id = None
        self._act_recovery = None
        self._controller_notice("Actuator recovery ended. Read per-arm results below; controller remains disabled.")
        self.output_queue.put("[actuator recovery] Finished. Check each arm's result above. Controller remains disabled; no movement requested.")

    def _set_actuator_power(self, unit: int, text: str) -> None:
        try:
            power = int(text)
            if not 1 <= power <= 60:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid actuator power", "Enter an integer from 1 to 60.")
            return
        self._run(f"u{unit} act power {power}")

    def _confirm_wheel_action(self, unit: int, title: str, message: str, command: str) -> bool:
        if not self.units:
            messagebox.showwarning("Not connected", "Connect to units first.")
            return False
        if not messagebox.askyesno(title, message):
            return False
        self._stop_controller(send_stop=False)
        self._run(command)
        return True

    def _set_wheel_tune_param(self, unit: int, param: str, text: str) -> bool:
        if not self.units:
            messagebox.showwarning("Not connected", "Connect to units first.")
            return False
        try:
            name, value = validate_wheel_tune_param(param, text)
        except ValueError as exc:
            messagebox.showerror("Invalid wheel tuning value", str(exc))
            return False
        self._stop_controller(send_stop=False)
        self._run(f"w{unit} tune set {name} {value:g}")
        return True

    def _refresh_live(self) -> None:
        if self._closing:
            return
        connected = {unit.index for unit in self.units if self._unit_available(unit)}
        for unit, fields in self.live_store.rows(connected).items():
            for variable, text in zip(self.live_vars[unit], (fields[0], fields[2], fields[4], fields[5], fields[6])):
                variable.set(text)
        for unit, fields in self.act_store.rows(connected).items():
            self.act_global_vars[unit].set(f"{fields[1]} A")
            for variable, text in zip(self.act_vars[unit], fields):
                variable.set(text)
        self._live_after_id = self.root.after(live.DISPLAY_MS, self._refresh_live)

    def _poll_queue(self) -> None:
        if self._closing:
            return
        logger = getattr(self, "session_log", None)
        if logger is not None:
            try:
                warning = logger.pop_warning()
                if warning:
                    self.output_queue.put(warning)
            except Exception:
                pass
        try:
            for _ in range(200):
                line = self.output_queue.get_nowait()
                self._log_event("console", text=line)
                if any(word in line.lower() for word in ("blocked", "fault:", "violation", "no unit ", "skipped ", "stop not confirmed")):
                    self._controller_notice("Robot: " + line[:220])
                self.console.config(state="normal")
                self.console.insert("end", line + "\n")
                self.console.see("end")
                self.console.config(state="disabled")
        except queue.Empty:
            pass
        self._queue_after_id = self.root.after(80, self._poll_queue)

    def _on_close(self) -> None:
        self._closing = True
        for callback in (self._controller_after_id, self._queue_after_id, getattr(self, "_live_after_id", None), getattr(self, "_recovery_after_id", None)):
            if callback is not None:
                self.root.after_cancel(callback)
        if self.units:
            self._disconnect()
        if PYGAME_AVAILABLE:
            self.controller.disconnect()
        logger = getattr(self, "session_log", None)
        if logger is not None:
            try:
                logger.close(timeout=.3)
            except Exception:
                pass
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    ClimberGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
