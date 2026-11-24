# V16 persistent diagnostic logs

The GUI automatically records diagnostics under the revision folder's `logs/`.
Each run has a unique session ID, shown in the existing console. Files are
newline-delimited JSON named `climber-v16-<UTC>-<PID>-<session>-<part>.jsonl`.
No extra command is needed to start recording.

Records contain UTC and monotonic timestamps, session/revision information,
GUI command requests and dispatch results, unit/USB mapping, console messages,
and every received serial line—including live telemetry fragments normally
hidden from the console. Raw telemetry preserves values, age fields, controller
state and fault bits for later reconstruction. A successful dispatch records
that the GUI dispatch returned without exception; it does not prove movement
or controller acceptance. The unit list on command requests is connection
context; the command text identifies its routing.

Recent readings are written continuously before a fault, so they remain
available after a GUI restart. Each rotated segment repeats revision/session
and the latest unit mapping. Keep the relevant files before conducting further
long tests if you want to preserve a specific event.

A background writer handles file I/O. Producers use a bounded 4096-record
queue and never wait for disk writes. Files flush every 250 ms and request an
OS sync every second, on fault text, and on orderly shutdown. At most 16 files
of 8 MiB each are retained across sessions (128 MiB total). Rotation removes
only regular files matching this logger's exact filename pattern; it does not
remove unrelated files or symlinks. Run one GUI instance for the connected
robot, as required for exclusive serial ownership.

A slow/full/unwritable disk does not prevent Stop or command dispatch. Queue
overflow keeps newer records and reports loss once in the console. Disk errors
disable further writes and report once. Shutdown waits at most 300 ms after
the existing stop/disconnect sequence; the writer is a daemon. A power failure,
forced application exit, stalled disk or full queue can lose recent data. The
logger does not guarantee crash-proof storage or verify physical stopping.

Offline tests use temporary files and fake GUI units only:

```bash
cd /path/to/revision/Gui
python3 -B -m unittest discover -v
```

Tests cover persisted pre-fault history, a second session, periodic flush,
bounded rotation, preserving unrelated files/symlinks, slow-disk queue and
shutdown bounds, disk creation/write errors, oversized records, raw telemetry
capture, unit mapping, and Stop reaching remaining units when logging fails.
