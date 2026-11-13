"""All tunable settings in one place. Every processed session stores the config it used."""
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import List, Optional

import yaml


@dataclass
class Config:
    # Step 6: cleaning
    rssi_min_dbm: float = -120.0          # below this is not a real reading
    rssi_max_dbm: float = -10.0           # above this is not a real indoor reading
    placeholder_values: List[float] = field(default_factory=lambda: [0.0, -127.0, -239.0])
    near_sensitivity_dbm: float = -100.0  # kept, but counted: this adapter's scale reads low
    reject_off_channel: bool = True       # frames heard while tuned elsewhere read low
    outlier_min_readings: int = 3         # the median test needs at least this many readings
    outlier_mad_k: float = 3.0            # reject beyond max(min_dev, k * 1.4826 * MAD) ...
    outlier_min_dev_db: float = 10.0      # ... but never within this many dB of the median
    # Step 7
    merge: str = "median"                 # or "mean"
    # Phase C
    cell_size_m: float = 0.25
    # Phase D
    idw_power: float = 2.0
    idw_min_points: int = 3
    kriging_min_points: int = 20
    kriging_model: str = "spherical"
    kriging_nlags: int = 6
    kriging_clip: Optional[List[float]] = None   # e.g. [-120, -10]; None = store unclipped
    target_ssids: List[str] = field(default_factory=list)  # empty = all networks (as in T1)

    def to_dict(self):
        return asdict(self)

    @classmethod
    def load(cls, path=None, **overrides):
        data = {}
        if path:
            data = yaml.safe_load(Path(path).read_text()) or {}
        data.update({k: v for k, v in overrides.items() if v is not None})
        known = {f.name for f in fields(cls)}
        unknown = set(data) - known
        if unknown:
            raise ValueError(f"unknown config keys: {sorted(unknown)}")
        cfg = cls(**data)
        if cfg.merge not in ("median", "mean"):
            raise ValueError("merge must be 'median' or 'mean'")
        return cfg
