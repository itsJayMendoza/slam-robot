"""Parse 802.11 beacon frames (radiotap + Dot11) into plain Python objects."""
import time
from dataclasses import dataclass
from typing import Iterator, Optional, Tuple

from scapy.layers.dot11 import Dot11, Dot11Beacon, Dot11Elt, RadioTap

IE_SSID = 0
IE_DS_PARAMS = 3
IE_BSS_LOAD = 11
IE_RSN = 48
IE_VENDOR = 221
WPA_OUI_TYPE = b"\x00\x50\xf2\x01"   # Microsoft OUI, type 1 = WPA (pre-RSN)
RSN_OUI = b"\x00\x0f\xac"


@dataclass(frozen=True)
class Beacon:
    t_unix: float
    bssid: str
    ssid: str
    channel: Optional[int]
    rssi_dbm: Optional[float]
    noise_dbm: Optional[float]      # None when the adapter reports no noise floor
    chan_util_pct: Optional[float]  # None when the AP sends no BSS Load IE
    tuned_channel: Optional[int] = None  # channel the adapter was on when it heard the frame
    encryption: Optional[str] = None     # OPEN, WEP, WPA, WPA2, WPA3, WPA2/WPA3, WPA2-Enterprise, OWE


def freq_to_channel(mhz: Optional[int]) -> Optional[int]:
    if mhz is None:
        return None
    if mhz == 2484:
        return 14
    if 2412 <= mhz <= 2472:
        return (mhz - 2407) // 5
    if 5000 <= mhz <= 5900:
        return (mhz - 5000) // 5
    return None


def rsn_akms(info: bytes):
    """AKM suite types listed in an RSN IE (2 = PSK, 8 = SAE, 1 = 802.1X, 18 = OWE, ...)."""
    try:
        pos = 2 + 4                                   # version, group cipher
        n = int.from_bytes(info[pos:pos + 2], "little")
        pos += 2 + 4 * n                              # pairwise cipher list
        m = int.from_bytes(info[pos:pos + 2], "little")
        pos += 2
        return [info[pos + 4 * i + 3] for i in range(m)
                if info[pos + 4 * i:pos + 4 * i + 3] == RSN_OUI]
    except IndexError:
        return []


def classify_encryption(privacy: bool, akms: Optional[list], has_wpa_ie: bool) -> str:
    """akms is None when the beacon carries no RSN IE."""
    if akms is not None:
        s = set(akms)
        if 18 in s:
            return "OWE"
        sae, psk = bool(s & {8, 24}), bool(s & {2, 6})
        if sae and psk:
            return "WPA2/WPA3"
        if sae:
            return "WPA3"
        if s & {1, 3, 5, 11, 12, 13}:
            return "WPA2-Enterprise"
        return "WPA2"
    if has_wpa_ie:
        return "WPA"
    return "WEP" if privacy else "OPEN"


def _iter_ies(pkt) -> Iterator[Tuple[int, bytes]]:
    elt = pkt.getlayer(Dot11Elt)
    while elt is not None and isinstance(elt, Dot11Elt):
        yield elt.ID, bytes(elt.info)
        elt = elt.payload.getlayer(Dot11Elt)


def parse_beacon(pkt, t_unix: Optional[float] = None) -> Optional[Beacon]:
    """Return a Beacon, or None if pkt is not a (parseable) beacon frame."""
    try:
        if not pkt.haslayer(Dot11Beacon):
            return None
        dot11 = pkt.getlayer(Dot11)
        bssid = (dot11.addr3 or "").lower()
        if not bssid:
            return None

        ssid = ""
        ds_channel = None
        chan_util = None
        akms = None
        has_wpa_ie = False
        for ie_id, info in _iter_ies(pkt):
            if ie_id == IE_SSID:
                ssid = info.decode("utf-8", errors="replace")
            elif ie_id == IE_DS_PARAMS and len(info) >= 1:
                ds_channel = info[0]
            elif ie_id == IE_BSS_LOAD and len(info) >= 3:
                # station count (2 bytes), channel utilization (1 byte, 0-255)
                chan_util = info[2] * 100.0 / 255.0
            elif ie_id == IE_RSN:
                akms = rsn_akms(info)
            elif ie_id == IE_VENDOR and info[:4] == WPA_OUI_TYPE:
                has_wpa_ie = True
        # Named flag, not a bit mask: scapy stores the capability field byte-swapped.
        privacy = bool(pkt.getlayer(Dot11Beacon).cap.privacy)

        rt = pkt.getlayer(RadioTap)
        rssi = noise = freq = None
        if rt is not None:
            rssi = getattr(rt, "dBm_AntSignal", None)
            noise = getattr(rt, "dBm_AntNoise", None)
            freq = getattr(rt, "ChannelFrequency", None)

        # The DS Parameter Set IE is the AP's own channel; the radiotap frequency is
        # where the adapter was tuned, which can differ (adjacent-channel leakage).
        channel = ds_channel if ds_channel else freq_to_channel(freq)

        return Beacon(
            t_unix=time.time() if t_unix is None else t_unix,
            bssid=bssid,
            ssid=ssid,
            channel=channel,
            rssi_dbm=None if rssi is None else float(rssi),
            noise_dbm=None if noise is None else float(noise),
            chan_util_pct=chan_util,
            tuned_channel=freq_to_channel(freq),
            encryption=classify_encryption(privacy, akms, has_wpa_ie),
        )
    except Exception:  # malformed frame: skip it, never crash the capture thread
        return None
