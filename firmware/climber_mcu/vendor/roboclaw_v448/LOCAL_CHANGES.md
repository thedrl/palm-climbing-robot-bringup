# Local Receive-Buffer Synchronization Fix

Base: https://github.com/basicmicro/roboclaw_arduino_library
Commit: 83c031b6476599045ddfdf32fd4f4efa4ee5a79f.

The inherited library change is explicit RX clearing at the beginning of each
packet transaction/retry. On this ESP32 core, flush() waits for TX completion
but does not clear RX. Buffered reply bytes can otherwise shift a new response
or supply a stale 0xFF acknowledgment. CRC validation is unchanged.

This follows the receive-buffer preparation used by the newer vendor library:
https://github.com/basicmicro/basicmicro_arduino/blob/main/src/Basicmicro.cpp
(write_n and write_address_cmd).

tools/test_roboclaw_link.py tests this actual source using a synthetic UART
whose flush semantics match the installed ESP32 core. It covers leftover ACKs,
retry garbage, absent controllers, bad CRCs and preservation of controller
fault bits. A fake UART is not proof of actual wiring or electrical integrity.

Only bringup_unit0 selects this local library. Other MCU environments retain
their existing dependency. Timeouts, CRC format, command payloads, current
limits and fault masks are not changed.

This candidate additionally corrects command 59 and 75 reply lengths to the
five-byte payloads captured over direct USB from the v4.4.8 controller. New
overloads expose the auto-maximum byte and CTRL1/CTRL2 modes; old signatures
delegate to them. Short legacy formats are not supported by this scoped
unit 0 library. GetPinFunctions now clears RX before every attempt. A new
read-only command 15 method reads the serial timeout without changing it.
Captured and malformed-reply tests exercise the actual library. The initial
controller-status discrepancy and other vendor warnings remain unresolved.
