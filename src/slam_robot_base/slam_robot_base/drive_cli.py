"""base_drive: drive the robot over SSH without ROS, for floor tests.

    base_drive status                       firmware version and encoder counts
    base_drive straight --distance 2.0      drive straight, then enter the tape-measured distance
                                            to get counts per metre
    base_drive teleop                       keyboard driving (w/s speed, a/d turn, space stop, q quit)

The firmware limits speed to about 0.28 m/s and stops the motors 0.5 s after the last command, so
the robot stops by itself if this tool or the SSH session dies.
"""
import argparse
import select
import sys
import termios
import time
import tty

from .link import DEFAULT_COUNTS_PER_M, MAX_CPS, BaseLink, LinkError

PERIOD_S = 0.1   # command rate, well inside the firmware's 500 ms timeout
FIRMWARE_ACCEL_CPS2 = 90000   # the firmware's speed ramp (ACCEL_CPS2)
HOLD_GAIN = 1.0               # counts/s of correction per count of left-right difference


def cmd_status(link, a):
    s = link.read()
    print(f"firmware: {link.request('ver')[4:]}")
    print(f"encoders: left {s.left}, right {s.right}; fault: {s.fault_text}")


def settle(link, timeout_s=2.0):
    """Wait until both wheels have stopped; returns the final state."""
    t0 = time.monotonic()
    s = link.read()
    while time.monotonic() - t0 < timeout_s and (abs(s.left_cps) > 20 or abs(s.right_cps) > 20):
        time.sleep(PERIOD_S)
        s = link.read()
    return s


def cmd_straight(link, a):
    cps = a.speed * a.counts_per_m
    if cps > MAX_CPS:
        sys.exit(f"--speed {a.speed} m/s is above the firmware limit "
                 f"({MAX_CPS / a.counts_per_m:.2f} m/s)")
    target = a.distance * a.counts_per_m
    sign = 1 if a.distance > 0 else -1
    print(f"driving {a.distance:+.2f} m at {a.speed:.2f} m/s ({target:+.0f} counts); Ctrl+C stops")
    link.stop()
    start = link.read()
    t0 = time.monotonic()
    braking = cps * cps / (2 * FIRMWARE_ACCEL_CPS2)     # counts the firmware needs to stop
    s = start
    try:
        while True:
            dl, dr = s.left - start.left, s.right - start.right
            done = (dl + dr) / 2
            # straight-line hold: the wheel that is behind in counts gets a little more speed
            corr = max(-0.2 * cps, min(0.2 * cps, HOLD_GAIN * sign * (dl - dr)))
            s = link.velocity(sign * cps - sign * corr, sign * cps + sign * corr)
            if s.fault:
                print(f"\nFAULT: {s.fault_text}")
                break
            print(f"\r  {done:+8.0f} / {target:+.0f} counts  speed L {s.left_cps:6.0f} "
                  f"R {s.right_cps:6.0f}  L-R {dl - dr:+5.0f}", end="", flush=True)
            if abs(done) >= abs(target) - braking:
                break
            time.sleep(PERIOD_S)
    except KeyboardInterrupt:
        print("\nstopped by Ctrl+C")
    link.velocity(0, 0)
    end = settle(link)
    link.stop()
    dl, dr = end.left - start.left, end.right - start.right
    mean = (dl + dr) / 2
    print(f"\nfinal: left {dl:+d}, right {dr:+d} counts (difference {dl - dr:+d}, "
          f"{100 * (dl - dr) / max(abs(mean), 1):+.1f}%), {time.monotonic() - t0:.1f} s")
    print(f"estimated distance with {a.counts_per_m:.0f} counts/m: {mean / a.counts_per_m:.3f} m")
    try:
        measured = input("tape-measured distance in m (Enter to skip): ").strip()
    except EOFError:
        measured = ""
    if measured:
        m = float(measured)
        print(f"counts per metre: {abs(mean / m):.0f} (left {abs(dl / m):.0f}, right {abs(dr / m):.0f})")
        print("a robot that drifted sideways means the wheel diameters differ; tell me which way")


HELP = ("w/s: faster/slower (negative = backward)   a/d: turn left/right   "
        "space: stop   q: quit")


def cmd_teleop(link, a):
    step, turn_step = a.step, a.step
    speed = turn = 0.0
    limit = MAX_CPS / a.counts_per_m
    print(HELP)
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    link.stop()
    try:
        while True:
            if select.select([sys.stdin], [], [], PERIOD_S)[0]:
                k = sys.stdin.read(1)
                if k == "q":
                    break
                if k == "w": speed += step
                elif k == "s": speed -= step
                elif k == "a": turn += turn_step
                elif k == "d": turn -= turn_step
                elif k == " ": speed = turn = 0.0
                speed = max(-limit, min(limit, speed))
                turn = max(-limit, min(limit, turn))
            # turn > 0 = counterclockwise: right wheel faster
            left = (speed - turn) * a.counts_per_m
            right = (speed + turn) * a.counts_per_m
            s = link.velocity(left, right)
            if s.fault:
                print(f"\nFAULT: {s.fault_text}; press space, then keep driving (fault cleared)")
                link.stop()
                speed = turn = 0.0
            print(f"\rspeed {speed:+.2f} m/s  turn {turn:+.2f}  | counts L {s.left:9d} R {s.right:9d} ",
                  end="", flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        link.velocity(0, 0)
        link.stop()
        print()


def listen(link, seconds=3.0):
    """After a lost reply, show what the ESP32 sends: a boot banner means it restarted."""
    print(f"listening {seconds:.0f} s for ESP32 messages ...")
    t0 = time.monotonic()
    heard = False
    while time.monotonic() - t0 < seconds:
        try:
            raw = link.port.readline()
        except Exception as e:      # the USB device itself went away
            print(f"  serial port lost: {e} (USB disconnected?)")
            return
        line = raw.decode(errors="replace").strip()
        if line:
            heard = True
            print(f"  {line}")
    if not heard:
        print("  nothing: the ESP32 is silent (USB link dropped, or the chip is stuck)")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--port", default="/dev/esp32", help="default /dev/esp32 (udev link)")
    ap.add_argument("--counts-per-m", type=float, default=DEFAULT_COUNTS_PER_M,
                    help=f"default {DEFAULT_COUNTS_PER_M:.0f} (floor test)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    p = sub.add_parser("straight")
    p.add_argument("--distance", type=float, default=2.0, help="metres; negative = backward")
    p.add_argument("--speed", type=float, default=0.10, help="m/s")
    p = sub.add_parser("teleop")
    p.add_argument("--step", type=float, default=0.05, help="m/s per key press")
    a = ap.parse_args(argv)
    link = None
    try:
        link = BaseLink.open(a.port)
        {"status": cmd_status, "straight": cmd_straight, "teleop": cmd_teleop}[a.cmd](link, a)
    except LinkError as e:
        print(f"\nerror: {e}")
        if link is not None:
            listen(link)
        sys.exit(1)
    for n in link.notices:
        if "fault" in n:
            print(f"firmware: {n[2:]}")


if __name__ == "__main__":
    main()
