# One individual beacon, kept so per-dwell standard deviation can be computed offline.
float64 t_unix
string bssid
string ssid
uint8 channel            # 0 = unknown
float32 rssi_dbm
bool noise_valid
float32 noise_dbm        # NaN unless noise_valid
bool chan_util_valid
float32 chan_util_pct    # NaN unless chan_util_valid
bool rejected            # true if the outlier filter dropped it
