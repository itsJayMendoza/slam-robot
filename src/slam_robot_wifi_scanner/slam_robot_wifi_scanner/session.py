"""Write a survey session in the format agreed with the post-processing (laptop) side.

One session folder holds:
  records.json       list of raw records, ONE PER BEACON heard (nothing filtered on the Pi)
  waypoints.json     waypoint definitions for this session (intended positions)
  notification.json  session summary: status, times, record count, SHA-256 of records.json
While the session runs, records are appended to records.jsonl (flushed after every waypoint),
so an interrupted session still keeps everything recorded up to that point.
"""
import hashlib
import json
import os
import time
from pathlib import Path
from typing import Iterable, List, Optional

from .beacon import Beacon


def beacon_record(b: Beacon, session_id: str, floor: str, seq: int, waypoint_id: int,
                  x: float, y: float, heading_deg: float) -> dict:
    return {
        "session_id": session_id,
        "floor": floor,
        "seq": seq,
        "timestamp": round(b.t_unix, 3),
        "waypoint_id": waypoint_id,
        "x": x,
        "y": y,
        "heading": heading_deg,             # degrees, counter-clockwise from the map's +x axis
        "bssid": b.bssid.upper(),
        "ssid": b.ssid,
        "channel": b.channel,                # the AP's own channel (DS Parameter Set IE)
        "tuned_channel": b.tuned_channel,    # channel the adapter was on when it heard the frame
        "rssi": b.rssi_dbm,
        "noise_floor": b.noise_dbm,          # null: this adapter reports no noise floor
        "chan_util": None if b.chan_util_pct is None else round(b.chan_util_pct, 1),
        "encryption": b.encryption,
    }


class SessionWriter:
    def __init__(self, out_dir, session_id: str, floor: str, spacing_m: Optional[float] = None):
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.session_id, self.floor, self.spacing_m = session_id, floor, spacing_m
        self.waypoints: List[dict] = []
        self.seq = 0
        self.start_ts = time.time()
        self._jsonl = open(self.dir / "records.jsonl", "w")

    def add_waypoint(self, waypoint_id: int, x: float, y: float):
        self.waypoints.append({"waypoint_id": waypoint_id, "x": x, "y": y})

    def add_beacons(self, waypoint_id: int, x: float, y: float, heading_deg: float,
                    beacons: Iterable[Beacon]) -> int:
        n = 0
        for b in beacons:
            self.seq += 1
            rec = beacon_record(b, self.session_id, self.floor, self.seq, waypoint_id, x, y, heading_deg)
            self._jsonl.write(json.dumps(rec) + "\n")
            n += 1
        self._jsonl.flush()
        os.fsync(self._jsonl.fileno())
        return n

    def finish(self, status: str = "complete") -> dict:
        """status is 'complete' or 'interrupted'. Returns the notification."""
        if status not in ("complete", "interrupted"):
            raise ValueError("status must be 'complete' or 'interrupted'")
        self._jsonl.close()
        records = [json.loads(line) for line in open(self.dir / "records.jsonl")]
        body = json.dumps(records, indent=1).encode()
        (self.dir / "records.json").write_bytes(body)
        (self.dir / "waypoints.json").write_text(json.dumps({
            "session_id": self.session_id, "floor": self.floor, "spacing_m": self.spacing_m,
            "waypoints": self.waypoints}, indent=1))
        note = {
            "session_id": self.session_id,
            "floor": self.floor,
            "status": status,
            "start_timestamp": round(self.start_ts, 3),
            "end_timestamp": round(time.time(), 3),
            "record_count": len(records),
            "records_sha256": hashlib.sha256(body).hexdigest(),
        }
        (self.dir / "notification.json").write_text(json.dumps(note, indent=1))
        return note
