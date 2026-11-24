# All Working

`All Working.py` is the computer-side serial supervisor for arms 0, 1, 2, and 3.
`climber_gui5.py` loads that exact file from the same folder, even when started
from another directory. Both front-ends use the same command routing.

The supervisor's control behavior is unchanged from the working
`climber_serial_supervisor.py` in this folder. The original remains available.
The GUI retains the controller crash fix and existing `controller_mapping.json`.
This change does not flash firmware, change motor settings, or enable ToF.

## Run the GUI

```bash
python3 "/Users/stem/dev/Gui/climber_gui5.py"
```

Use Auto-detect and Connect. Confirm all intended units appear by their saved
IDs: u0, u1, u2, u3. Keep unit-ID discovery enabled.

## Run the Terminal Supervisor Instead

```bash
python3 "/Users/stem/dev/Gui/All Working.py" --auto
```

Do not run the GUI and terminal supervisor against the same ports simultaneously.
The quotes around `All Working.py` are required because its name contains a space.
Dependencies are `pyserial`, and optionally `pygame` for the GUI controller tab;
the GUI also requires Python with Tk support.

At `climber>`, use one command at a time:

| Command | Action |
| --- | --- |
| `unit` | Read connected unit identities |
| `act diag` | Read RoboClaw diagnostics on connected units |
| `wheel diag` | Read ODrive diagnostics on connected units |
| `act center` | Home and center all connected actuators |
| `start` | Run actuator homing/centering and wheel startup tests on all connected arms |
| `start 1` | Run startup only on arm 1 |
| `wheel test` | Jog all connected wheels at 5 RPM for 2 seconds |
| `act1 fwd` | Extend only arm 1 toward its forward limit |
| `stop` | Request stop on all connected units |
| `quit` | Request stop and exit the supervisor |

`start` includes wheel motion at 600 RPM in both directions. Secure the assembly,
keep wheels free and people clear, and have the power cutoff available. Commands
are broadcast to the connected units; they are not precisely synchronized.

## Firmware Notes

The motor-control routines run on the ESPs, not in these Python scripts.
Arm 1's latest firmware uses CAN TX D9 / GPIO18 and RX D10 / GPIO21. Its heartbeat
was verified after that change. Other arms' firmware was not changed here.
Arm 1's earlier RoboClaw acknowledgement problem is separate from CAN and must
be resolved before its full startup can pass. Confirm controller power and
diagnostics before movement. The requested filename is not certification that
every arm has passed a powered hardware test.

## Software Tests

From this folder:

```bash
python3 -B -m unittest -v test_climber_gui5 test_all_working
```

These use fake arms to check discovery, four-arm broadcast, per-arm routing,
GUI import paths, stop behavior, and controller regression cases. They do not
open robot serial connections or run motors.

All 43 automated tests passed on 2026-09-18. The previous GUI and test helpers
were preserved in `backup_before_all_working_20260918` before installation.
