"""Serial link to the ESP32 base firmware (firmware/src/base). No ROS here.

The firmware speaks ASCII lines: "v <l> <r>" (wheel speeds in encoder counts/s) answers
"e <left counts> <right counts> <left cps> <right cps> <fault>". Lines starting with '#' are
comments (start-up banner, fault notices). See the protocol comment in firmware/src/base/main.cpp.
"""
import time
from dataclasses import dataclass

COUNTS_PER_TURN = 49200.0         # left wheel turned once by hand
DEFAULT_COUNTS_PER_M = 178200.0   # floor test: 44,560 counts over 0.25 m (to refine with a longer run)
MAX_CPS = 50000                   # the firmware's own limit, about 0.28 m/s

FAULTS = {1: "left stalled", 2: "right stalled", 4: "left turning the wrong way",
          8: "right turning the wrong way", 16: "left overspeed", 32: "right overspeed"}


class LinkError(Exception):
    pass


@dataclass
class State:
    left: int          # encoder counts, forward positive
    right: int
    left_cps: float    # measured speed, counts/s
    right_cps: float
    fault: int

    @property
    def fault_text(self):
        return ", ".join(t for bit, t in FAULTS.items() if self.fault & bit) or "none"


def parse_state(line):
    parts = line.split()
    if len(parts) != 6 or parts[0] != "e":
        raise LinkError(f"unexpected reply: {line!r}")
    try:
        return State(int(parts[1]), int(parts[2]), float(parts[3]), float(parts[4]), int(parts[5]))
    except ValueError as e:
        raise LinkError(f"unexpected reply: {line!r}") from e


class BaseLink:
    """Request/response over a pyserial-like port (needs write, readline, reset_input_buffer)."""

    def __init__(self, port, notices=None):
        self.port = port
        self.notices = notices if notices is not None else []   # '#' lines seen

    @classmethod
    def open(cls, device="/dev/esp32", baud=115200, boot_wait_s=2.0):
        import serial    # python3-serial; imported here so the tests need no serial port
        try:
            port = serial.Serial(device, baud, timeout=0.3)
        except serial.SerialException as e:
            raise LinkError(f"cannot open {device}: {e} (is the ESP32 plugged in? are you in the "
                            "dialout group?)") from e
        time.sleep(boot_wait_s)          # opening the port resets the ESP32
        port.reset_input_buffer()
        link = cls(port)
        version = link.request("ver")
        if not version.startswith("ver slam-robot-base"):
            raise LinkError(f"no base firmware on {device} (got {version!r})")
        return link

    def request(self, cmd):
        self.port.write((cmd + "\n").encode())
        for _ in range(5):
            raw = self.port.readline()
            if not raw:
                raise LinkError(f"no reply to {cmd!r}")
            line = raw.decode(errors="replace").strip()
            if line.startswith("#"):
                self.notices.append(line)
                continue
            if line:
                return line
        raise LinkError(f"no reply to {cmd!r}")

    def velocity(self, left_cps, right_cps):
        return parse_state(self.request(f"v {round(left_cps)} {round(right_cps)}"))

    def read(self):
        return parse_state(self.request("e"))

    def stop(self):
        """Stop and clear a latched fault."""
        reply = self.request("x")
        if not reply.startswith("ok"):
            raise LinkError(f"stop not acknowledged: {reply!r}")
