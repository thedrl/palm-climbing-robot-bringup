"""Bounded, input-only Tk/SDL stress test. GUI robot commands use a stub."""

import argparse
import importlib.util
import sys
import time
import types
from unittest.mock import patch


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("module")
    parser.add_argument("--seconds", type=float, default=20)
    args = parser.parse_args()

    def forbidden(*args, **kwargs):
        raise AssertionError("Robot/serial access is forbidden in this test")

    stub = types.ModuleType("climber_all_working_stub")
    stub.BAUD_DEFAULT = 115200
    stub.open_units = forbidden
    stub.handle_supervisor_command = forbidden
    stub.likely_usb_serial_ports = lambda: []
    stub.list_ports = types.SimpleNamespace(comports=lambda: [])
    spec = importlib.util.spec_from_file_location("gui_under_test", args.module)
    gui = importlib.util.module_from_spec(spec)
    with patch("serial.Serial", side_effect=forbidden):
        spec.loader.exec_module(gui)
    gui.sup = stub
    root = gui.tk.Tk()
    app = gui.ClimberGUI(root)
    root.title("Controller input test - ROBOT DISCONNECTED")
    errors = []
    root.report_callback_exception = lambda *exc: errors.append(str(exc))
    start = time.monotonic()
    samples = 0
    changes = 0
    last_state = None
    batches = 0
    app.notebook.select(app.notebook.tabs()[-1])

    def connect():
        nonlocal last_state
        ok, info = app.controller.connect()
        print(f"CONTROLLER ok={ok} {info}", flush=True)
        if not ok:
            errors.append(info)
            app._on_close()
            return
        print(f"SDL driver={gui.pygame.display.get_driver()}", flush=True)
        original_poll = app.controller.poll

        def counted_poll():
            nonlocal samples, changes, last_state
            axes, buttons, hats = original_poll()
            samples += 1
            state = (tuple(round(a, 2) for a in axes), tuple(buttons), tuple(hats))
            if last_state is not None and state != last_state:
                changes += 1
            last_state = state
            return axes, buttons, hats

        app.controller.poll = counted_poll
        redraw()

    def redraw():
        nonlocal batches
        batches += 1
        for i in range(30):
            app.output_queue.put(f"[simulated] console redraw batch={batches} line={i}")
        app.console.yview_moveto(1.0)
        if batches % 20 == 0:
            root.geometry("1050x760" if batches % 40 else "1060x770")
        if time.monotonic() - start >= args.seconds:
            app._on_close()
        else:
            root.after(30, redraw)

    root.after(100, connect)
    root.mainloop()
    gui.pygame.quit()
    print(f"RESULT elapsed={time.monotonic()-start:.2f}s polls={samples} "
          f"input_changes={changes} redraw_batches={batches} callback_errors={errors} "
          "robot_connections=0", flush=True)
    if errors or samples < 10:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
