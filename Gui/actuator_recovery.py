"""Response-driven V17 actuator recovery. Opens no ports; never requests motion."""
from collections import deque
import threading
import time


class Recovery:
    def __init__(self, units, sample, report, *, timeout=3.0):
        self.units = list(units)
        self.sample = sample
        self.report = report
        self.timeout = timeout
        self.cancelled = threading.Event()
        self.done = threading.Event()
        self.condition = threading.Condition()
        self.lines = deque(maxlen=512)
        self.thread = None

    def feed(self, unit, text):
        if 'PART,' in text:
            if 'bringup>' not in text:
                return
            text = text.split('bringup>', 1)[0] + 'bringup>'
        with self.condition:
            self.lines.append((time.monotonic(), unit, text))
            self.condition.notify_all()

    def cancel(self):
        self.cancelled.set()
        with self.condition:
            self.condition.notify_all()

    def start(self):
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    @staticmethod
    def available(unit):
        if unit.stop_reader.is_set() or not unit.serial_port.is_open:
            raise RuntimeError('USB disconnected; reconnect this unit')

    def send(self, unit, command):
        if self.cancelled.is_set():
            raise RuntimeError('cancelled')
        self.available(unit)
        began = time.monotonic()
        # The serial write timeout bounds this write. Avoid tcdrain/flush,
        # which can wait indefinitely on a failed USB device.
        unit.send(command, flush=False)
        return began

    def wait_prompt(self, unit, began, *, acknowledgement=False):
        deadline = began + self.timeout
        ack = None
        with self.condition:
            while not self.cancelled.is_set():
                self.available(unit)
                ack = None
                for at, index, text in self.lines:
                    if index != unit.index or at < began or at > deadline:
                        continue
                    if 'Actuator stop NOT confirmed' in text:
                        raise RuntimeError('zero commands were not acknowledged')
                    if 'Actuator zero commands acknowledged.' in text:
                        ack = at
                    # The ACK must precede a prompt, not follow an old prompt.
                    prompt_after_ack = ack is not None and (at > ack or
                        ('Actuator zero commands acknowledged.' in text and
                         text.find('bringup>') > text.find('Actuator zero commands acknowledged.')))
                    if 'bringup>' in text and (not acknowledgement or prompt_after_ack):
                        return at
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError('no completed zero acknowledgement' if acknowledgement else 'no prompt after Stop')
                self.condition.wait(min(remaining, .05))
        raise RuntimeError('cancelled')

    def verify(self, unit, completed):
        deadline = time.monotonic() + self.timeout
        while not self.cancelled.is_set():
            self.available(unit)
            now = time.monotonic()
            sample = self.sample(unit.index)
            if sample is not None and sample.started >= completed and now - sample.received <= .6:
                # V17 explicit Stop invalidates readings AND pending telemetry
                # fragments before its prompt. Require a complete frame begun
                # after that prompt, so old snapshots cannot count as recovery.
                # The limit polls every 5 s; its post-Stop generation is stronger
                # evidence than comparing its source time with a delayed prompt.
                fresh = all(sample.ages[i] >= 0 and
                            sample.ages[i] + (now - sample.received) * 1000 <= (5000 if i == 3 else 600)
                            for i in (1, 2, 3, 4))
                if fresh:
                    pwm1, pwm2 = sample.values[3:5]
                    status = sample.values[6]
                    if sample.state != 1 or sample.values[7] != 0 or pwm1 or pwm2:
                        raise RuntimeError('fresh readings do not confirm idle/zero PWM')
                    if sample.channel != 1 or abs(sample.values[1]) > 50:
                        raise RuntimeError('fresh M1 current/channel does not confirm the stopped actuator')
                    if not 670 <= sample.values[5] <= 675:
                        raise RuntimeError('fresh actuator current limit is outside 6.70–6.75 A')
                    remaining = status & ~0x03000000
                    if remaining:
                        self.report(unit.index, f'Stop acknowledged; zero PWM. Controller status remains 0x{status:08X}; inspect Unit diagnostics.')
                    else:
                        self.report(unit.index, f'Software latch reset; fresh zero PWM, M1 current at most 0.50 A, verified current ceiling and controller status 0x{status:08X}. No movement requested.')
                    return
            if now >= deadline:
                raise RuntimeError('Stop acknowledged, but fresh PWM/status verification unavailable; inspect Unit diagnostics')
            self.cancelled.wait(.05)
        raise RuntimeError('cancelled')

    def run(self):
        try:
            sent = {}
            # Stop every board before waiting on any one failed board.
            for unit in self.units:
                try:
                    sent[unit.index] = self.send(unit, 'stop')
                except Exception as exc:
                    self.report(unit.index, f'Recovery not verified: {exc}')
            for unit in self.units:
                if self.cancelled.is_set():
                    break
                if unit.index not in sent:
                    continue
                try:
                    self.wait_prompt(unit, sent[unit.index])
                    began = self.send(unit, 'act stop')
                    completed = self.wait_prompt(unit, began, acknowledgement=True)
                    self.verify(unit, completed)
                except Exception as exc:
                    self.report(unit.index, f'Recovery not verified: {exc}')
        finally:
            self.done.set()
