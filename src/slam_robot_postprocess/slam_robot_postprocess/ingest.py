"""Phase A (steps 3-5): check a received session and store an untouched copy.

The HTTP push/pull between Pi and laptop is a transport detail; here a session arrives as a
folder (notification.json, records.json, waypoints.json), e.g. copied with scp.
"""
import hashlib
import json
import shutil
from pathlib import Path

SESSION_FILES = ("notification.json", "records.json", "waypoints.json")


class SessionError(Exception):
    """A session that must not be processed; the message says why."""


def file_sha256(*paths) -> str:
    h = hashlib.sha256()
    for p in paths:
        h.update(Path(p).read_bytes())
    return h.hexdigest()


def check_session(session_dir, accept_interrupted: bool = False):
    d = Path(session_dir)
    for name in SESSION_FILES:
        if not (d / name).is_file():
            raise SessionError(f"{d}: missing {name}")
    note = json.loads((d / "notification.json").read_text())
    if note.get("status") == "interrupted" and not accept_interrupted:
        raise SessionError(f"session {note.get('session_id')} is interrupted, awaiting decision: "
                           "rerun with --accept-interrupted to process it with a partial warning")
    if note.get("status") not in ("complete", "interrupted"):
        raise SessionError(f"unknown session status {note.get('status')!r}")
    raw = (d / "records.json").read_bytes()
    records = json.loads(raw)
    if len(records) != note.get("record_count"):
        raise SessionError(f"record count mismatch: notified {note.get('record_count')}, "
                           f"received {len(records)} (retry the transfer)")
    if "records_sha256" in note and hashlib.sha256(raw).hexdigest() != note["records_sha256"]:
        raise SessionError("records.json checksum does not match the notification (retry the transfer)")
    seqs = [r.get("seq") for r in records]
    if all(s is not None for s in seqs) and sorted(seqs) != list(range(1, len(records) + 1)):
        raise SessionError("record sequence numbers have gaps or duplicates (retry the transfer)")
    waypoints = json.loads((d / "waypoints.json").read_text())
    if str(waypoints.get("session_id")) != str(note.get("session_id")):
        raise SessionError("waypoints.json belongs to a different session")
    return note, records, waypoints


def store_raw_copy(session_dir, store_dir, note) -> Path:
    """Copy the received files untouched. Refuses to overwrite a different earlier copy."""
    raw = Path(store_dir) / "sessions" / str(note["session_id"]) / "raw"
    raw.mkdir(parents=True, exist_ok=True)
    for name in SESSION_FILES:
        src, dst = Path(session_dir) / name, raw / name
        if dst.exists():
            if dst.read_bytes() != src.read_bytes():
                raise SessionError(f"{dst} already holds a different copy of this session")
        else:
            shutil.copy2(src, dst)
    return raw


def ensure_map(store_dir, floor: str, map_yaml=None) -> Path:
    """The floor's map lives in store/maps/<floor>/. Import it from map_yaml if missing."""
    import yaml
    folder = Path(store_dir) / "maps" / str(floor)
    target = folder / "map.yaml"
    if target.exists():
        return target
    if map_yaml is None:
        raise SessionError(f"no map for floor {floor}: expected {target}. Fetch the floor's "
                           "map.yaml and image from the robot (or pass --map).")
    src = Path(map_yaml)
    meta = yaml.safe_load(src.read_text())
    image = src.parent / meta["image"]
    folder.mkdir(parents=True, exist_ok=True)
    shutil.copy2(image, folder / image.name)
    shutil.copy2(src, target)
    return target
