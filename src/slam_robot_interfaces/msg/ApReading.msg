# One access point, averaged over a scan dwell (outliers already removed).
string bssid
string ssid
uint8 channel            # 0 = unknown
uint32 n_samples         # beacons kept
uint32 n_rejected        # outliers dropped (frames heard off-channel are excluded before this count)
bool low_confidence      # too few samples for the mean and std to be trusted
float32 rssi_mean
float32 rssi_std
float32 rssi_median
float32 rssi_min
float32 rssi_max
bool noise_valid         # false when the adapter reports no noise floor
float32 noise_mean       # NaN unless noise_valid
float32 noise_std        # NaN unless noise_valid
bool chan_util_valid     # false when the AP sends no BSS Load IE
float32 chan_util_mean   # percent, NaN unless chan_util_valid
