"""Live capture: channel hopper + scapy sniffer feeding a thread-safe collector."""
import subprocess
import threading
import time
from typing import List, Sequence

from .beacon import Beacon, parse_beacon


class Collector:
    """Stores beacons only while recording is on."""

    def __init__(self):
        self._lock = threading.Lock()
        self._items: List[Beacon] = []
        self._recording = False

    def start(self):
        with self._lock:
            self._items = []
            self._recording = True

    def stop(self) -> List[Beacon]:
        with self._lock:
            self._recording = False
            items, self._items = self._items, []
        return items

    def count(self) -> int:
        with self._lock:
            return len(self._items)

    def on_packet(self, pkt):
        b = parse_beacon(pkt)
        if b is None:
            return
        with self._lock:
            if self._recording:
                self._items.append(b)


class ChannelHopper(threading.Thread):
    """Cycles the adapter through channels with `iw dev <if> set channel N` (needs root)."""

    def __init__(self, iface: str, channels: Sequence[int], dwell_s: float = 0.2):
        super().__init__(daemon=True)
        self.iface, self.channels, self.dwell_s = iface, list(channels), dwell_s
        self._stop_evt = threading.Event()

    def run(self):
        while not self._stop_evt.is_set():
            for ch in self.channels:
                if self._stop_evt.is_set():
                    return
                subprocess.run(["iw", "dev", self.iface, "set", "channel", str(ch)],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self._stop_evt.wait(self.dwell_s)

    def stop(self):
        self._stop_evt.set()


def set_monitor_mode(iface: str, channel: int = 6):
    """Put iface in monitor mode. Never call this on the interface carrying SSH."""
    for cmd in (["ip", "link", "set", iface, "down"],
                ["iw", "dev", iface, "set", "type", "monitor"],
                ["ip", "link", "set", iface, "up"],
                ["iw", "dev", iface, "set", "channel", str(channel)]):
        subprocess.run(cmd, check=True)


class LiveScanner:
    """Owns the sniffer and hopper for the lifetime of the node."""

    def __init__(self, iface: str, channels: Sequence[int], channel_dwell_s: float):
        from scapy.sendrecv import AsyncSniffer  # imported lazily: needs root + scapy
        self.collector = Collector()
        self.hopper = ChannelHopper(iface, channels, channel_dwell_s)
        self._sniffer = AsyncSniffer(iface=iface, prn=self.collector.on_packet, store=False)

    def start(self):
        self._sniffer.start()
        self.hopper.start()

    def stop(self):
        self.hopper.stop()
        try:
            self._sniffer.stop()
        except Exception:
            pass

    def scan(self, dwell_s: float) -> List[Beacon]:
        self.collector.start()
        time.sleep(dwell_s)
        return self.collector.stop()
