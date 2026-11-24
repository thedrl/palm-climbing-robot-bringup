# Controller crash fix, September 18, 2026

## What failed

Three macOS Python crash reports at 14:10:17, 14:11:18, and 14:14:32
show the same native SIGABRT path: the Tk timer called pygame's SDL Cocoa
event pump, which re-entered Tk text redraw callbacks and failed inside
`PyEval_RestoreThread`. This is a window/event-loop conflict, not an ESP
firmware failure. Python exception handling alone cannot catch this abort.

## Changes in climber_gui5.py

- Tk remains the only native window/event-loop owner. Pygame initializes
  its headless `dummy` display and joystick subsystem, not all subsystems.
- Background joystick updates are enabled so input works with Tk focused.
- Joystick removal and polling errors disable control, request a stop on
  every connected unit, and leave the GUI timer running for recovery.
- Disabling/disconnecting controller control or learning/resetting mappings
  requests a stop. A stop button takes precedence over motion and disables
  controller control until explicitly re-enabled.
- Release every controller button after enabling, then press to move.
- Bad button indices and invalid/non-finite speed settings cannot crash
  or issue invalid motion commands. Console work per tick is bounded.
- Ordinary actuator/wheel commands still use the existing supervisor parser.

Firmware, serial supervisor, and the saved mapping JSON were not changed.
The previous GUI is saved beside the updated file as
`climber_gui5.before_controller_fix_<timestamp>.py.bak`.

## Verification

- Python 3.13.9, pygame 2.6.1, SDL 2.28.4 on this Mac.
- 29 automated regression tests passed, including 1,000 simulated
  enable/move/stop cycles, unplug errors, malformed mappings, stop priority,
  command failures, and controller mapping capture.
- Real connected PS4 controller: 30.04 seconds, 894 input polls, 895 console
  redraw batches, one input-state change, zero Tk callback errors.
- A short five-second baseline did not reproduce the intermittent abort;
  diagnosis is based on the three existing native crash stacks.
- No robot serial connection was opened and no motor command was sent.
  This is not an endurance or physical stopping-distance certification.

Run regression tests from this folder:

```sh
python3 test_climber_gui5.py
```

The optional disconnected GUI test blocks the real supervisor entirely:

```sh
python3 gui_controller_smoke.py ./climber_gui5.py --seconds 30
```

## Retry

Close any existing GUI, then run:

```sh
cd /Users/stem/dev/Gui
python3 climber_gui5.py
```

First check controller telemetry with robot connections closed. The saved
mapping currently assigns button 11 to both selecting actuator 0 and
actuator forward. Use Learn to assign distinct buttons before motor testing.
After a stop or input error, re-enable control and release all buttons
before issuing new movement. A software stop request is not proof that
the hardware stopped; keep the physical cutoff available.

## Implementation references

- [SDL video-driver selection](https://wiki.libsdl.org/SDL2/SDL_HINT_VIDEODRIVER)
- [Pygame joystick events and background input](https://www.pygame.org/docs/ref/joystick.html)
