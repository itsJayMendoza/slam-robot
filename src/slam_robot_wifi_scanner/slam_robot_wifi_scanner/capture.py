"""Live capture: channel hopper + scapy sniffer feeding a thread-safe collector."""
import subprocess
import threading
import time
from typing import Callable, Dict, List, Optional, Sequence, Set

from .beacon import Beacon, parse_beacon


class Collector:
    """Stores beacons only while recording is on."""

    def __init__(self):
        self._lock = threading.Lock()
        self._items: List[Beacon] = []
        self._recording = False
        self._seen: Dict[int, float] = {}

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
            if b.channel:                       # remember where APs live, even when not recording
                self._seen[b.channel] = time.monotonic()
            if self._recording:
                self._items.append(b)

    def channels_seen(self, max_age_s: float) -> Set[int]:
        """Channels on which an AP has announced itself within the last max_age_s seconds."""
        cutoff = time.monotonic() - max_age_s
        with self._lock:
            return {c for c, t in self._seen.items() if t >= cutoff}


def plan_channels(all_channels: Sequence[int], seen: Set[int], full_sweep: bool) -> List[int]:
    """Channels to visit this round: all of them for a discovery sweep, otherwise only those
    where an AP was recently heard (falling back to all if none were)."""
    if full_sweep:
        return list(all_channels)
    active = [c for c in all_channels if c in seen]
    return active or list(all_channels)


class ChannelHopper(threading.Thread):
    """Cycles the adapter through channels with `iw dev <if> set channel N` (needs root).

    With active_fn (adaptive mode) it does a full discovery sweep every rediscover_s seconds and
    otherwise visits only channels that active_fn() reports, so each real AP gets several
    times more on-channel beacons than a blind sweep of all 13 channels would give."""

    def __init__(self, iface: str, channels: Sequence[int], dwell_s: float = 0.2,
                 active_fn: Optional[Callable[[], Set[int]]] = None, rediscover_s: float = 10.0):
        super().__init__(daemon=True)
        self.iface, self.channels, self.dwell_s = iface, list(channels), dwell_s
        self.active_fn, self.rediscover_s = active_fn, rediscover_s
        self._stop_evt = threading.Event()

    def run(self):
        last_full = float("-inf")
        while not self._stop_evt.is_set():
            now = time.monotonic()
            full = self.active_fn is None or now - last_full >= self.rediscover_s
            if full:
                last_full = now
            seen = self.active_fn() if self.active_fn else set()
            for ch in plan_channels(self.channels, seen, full):
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

    def __init__(self, iface: str, channels: Sequence[int], channel_dwell_s: float,
                 adaptive: bool = True, rediscover_s: float = 10.0, max_age_s: float = 30.0):
        from scapy.sendrecv import AsyncSniffer  # imported lazily: needs root + scapy
        self.collector = Collector()
        active = (lambda: self.collector.channels_seen(max_age_s)) if adaptive else None
        self.hopper = ChannelHopper(iface, channels, channel_dwell_s, active, rediscover_s)
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
