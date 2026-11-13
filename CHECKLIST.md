# slam-robot project checklist

Legend: `[x]` done and verified, `[~]` written and tested here but not yet proven on the robot or Pi, `[ ]` not started.
Last updated: 2026-11-06.

## 1. Pi and environment
- [x] Ubuntu Server 22.04 arm64 on the Pi 4B, headless, SSH works (`robot@slam.local`)
- [x] ROS 2 Humble installed natively (ros-base, slam_toolbox, nav2, robot_localization, rosbridge, laser_filters)
- [x] Docker build environment for Humble on the laptop (`docker/run.sh`)
- [x] Workspace pushed to private GitHub repo `itsJayMendoza/slam-robot`
- [ ] Deploy key on the Pi, clone the repo, `colcon build` the robot packages natively
- [ ] `pip3 install pykrige==1.7.2` on the Pi
- [ ] Router: move 5 GHz off DFS channels (36-48), reserve the Pi's IP (MAC D8:3A:DD:1A:C2:C8)
- [ ] Decide: Pi regulatory domain (currently GB, intended PH)
- [ ] Laptop RViz against the Pi: `ros:humble` container with `--net=host`, `ROS_DOMAIN_ID=42` (laptop is Jazzy)
- [ ] Decide: move the Gazebo sim from Jazzy to Humble + Fortress, or keep it as a Jazzy sandbox
- [ ] Hold kernel upgrades once everything works (`apt-mark hold linux-image-raspi`)
- [ ] Always `sudo shutdown now` before cutting power (several hard cuts so far)

## 2. WiFi scanning (highest risk)
- [x] T3U Plus detected (USB 2357:0138), driver built via DKMS (morrownr/88x2bu-20210702), monitor mode works
- [x] Beacons received with sensible RSSI; adapter on the USB 2.0 side
- [x] Noise floor: radiotap has no noise field, so SNR is not measurable (noise_valid=false, SNR left empty)
- [x] Wireshark tools fail on this Pi (dumpcap permission error); scapy and tcpdump work instead
- [~] `slam_robot_wifi_scanner` (parser, outlier filter, averaging, channel hopper, action server, CLI): 18 tests pass
- [x] Scanner CLI run on the Pi with the real adapter: 8 APs found, Yumi 2.4G on channel 11, BSS Load present on all APs (strong routers report 0%)
- [x] Found: frames heard while tuned to another channel bias the mean low (up to 4 dB) and inflate std; on-channel filter fixes it but cuts samples about 5x
- [~] Adaptive hopping + on-channel filter default + low-confidence flag: written, 87 tests pass, action server verified with a fake adapter; not yet run on the Pi
- [x] Channels 12 and 13 are disabled by the T3U driver (the kernel lists them as `(disabled)`). Tried `rtw_country_code` PH and GB and `iw reg set GB`; none helped. The scanner covers channels 1-11, skips 12/13 and reports them as unavailable in each scan result
- [ ] Document the 1-11 limit; check once (phone app or another adapter) whether the surveyed building has any AP on channel 12 or 13
- [~] Hand-carried walk test: `walk_survey` (guided stop-and-scan, writes records.json / waypoints.json / notification.json per the client's T1 format, one record per beacon, with encryption), `make_room_map` (room size -> map.yaml/.pgm); 104 tests pass; not yet run on the Pi
- [ ] Repeatability: 3 back-to-back scans without moving anything; on-channel means should agree within a couple of dB
- [ ] Check which APs broadcast the BSS Load IE (channel utilization)
- [ ] Noise-floor test in several rooms (confirm the field is always absent)
- [ ] Monitor-mode setup at boot (systemd unit; never touch `wlan0`)
- [ ] udev rule: no USB autosuspend for the T3U (and later the LiDAR and ESP32)
- [ ] Antenna mast at 0.7-1.2 m, USB 2.0 extension cable, antenna orientation fixed
- [ ] Optional, client's choice: RTL-SDR as a separate noise-floor sensor
- [ ] Tell the client: noise floor and SNR are a documented limitation

## 3. Chassis, power, electronics
- [ ] Assemble chassis, JGA25-360 motors, ball caster, second deck
- [ ] Wiring per the plan (TB6612 and ESP32 pins; BIN1 on GPIO 17, not 12); common ground
- [ ] Verify encoder outputs are true 90-degree quadrature; measure real ticks per wheel revolution
- [ ] Power: 3S LiPo, DC-rated switch, inline fuse, XL4016 set to about 5.1 V, separate Pi rail
- [ ] Power ESP32 and LiDAR from the buck rail if the Pi's USB (about 1.2 A total) is too tight; test with all devices under load
- [ ] Measure battery runtime (Pi + LiDAR + motors); plan sessions for the long 0.5 m and 0.25 m surveys
- [ ] Mast stability, low ballast, doorway clearance of at least 80 cm
- [ ] Measure track width on the built chassis (wheel radius 36 mm)

## 4. ESP32 firmware and base bridge
- [ ] Firmware: PWM, direction, encoder ISRs, MPU6050, wheel-speed PID, 500 ms watchdog
- [ ] Framed USB serial protocol (setpoints in, ticks and IMU out)
- [ ] `slam_robot_base_bridge`: `/odom`, `/imu/data`, `/cmd_vel`, TF odom to base_link
- [ ] Teleop from the laptop

## 5. LiDAR, fusion, mapping
- [ ] YDLidar-SDK and `ydlidar_ros2_driver` on the Pi (X3: verify port, baud, scan frequency)
- [ ] Verify X3 minimum range, blind spot, glass and dark-surface behavior
- [ ] Mask the mast angular slice (`ignore_array` or laser_filters)
- [ ] URDF and TF tree for the real robot
- [ ] EKF (`robot_localization`) fusing odometry and IMU
- [ ] `slam_toolbox` mapping run, map saved
- [ ] Experiment 1 targets: wall RMSE at most 0.10 m, overlap at least 85%
- [ ] Tell the client the X3 is the LiDAR used (the thesis names the LD06)

## 6. Navigation
- [ ] Nav2 tuned for a small differential robot at low speed
- [ ] Localization on the saved map (slam_toolbox localization or AMCL)
- [ ] Reliable goal reaching; stop and settle behavior

## 7. Survey logic
- [~] `slam_robot_waypoints` (lattice, clearance, connectivity, lawnmower, JSON/CSV, map loader, CLI): 23 tests pass
- [~] `slam_robot_interfaces` (ScanAtWaypoint action, ApReading, RawReading)
- [ ] `slam_robot_survey_manager`: state machine (IDLE to DONE), goToPose loop, settle check, scan action call, retry once then skip, inaccessible marking
- [ ] Run folder: `raw_readings.csv`, `waypoint_avg.csv`, `run_meta.json`, map, waypoint list
- [ ] `/survey_status` topic (waypoint index, total, last RSSI, ETA)
- [ ] Total survey completion time recorded per run
- [ ] Repeat-run support (reload the same waypoint set)
- [ ] Test the manager with a mock scanner and mock Nav2 before the real robot

## 8. Post-processing
- [~] `slam_robot_interpolation` (IDW, Ordinary Kriging, variance, holdout, k-fold, thinning, metrics, per-AP table, gap finder): 34 tests pass
- [ ] Heatmap PNG rendering overlaid on the floor plan (RSSI, SNR where valid, noise where valid, Kriging variance)
- [~] `slam_robot_postprocess`: the client's T1 pipeline, phases A-D (session checks, untouched raw copy, cleaning with per-reason counts incl. off-channel and the adapter's -239 placeholder, median merge, per-SSID sets, grid from map.yaml, IDW, Kriging with min-points/fail handling, all-networks view, T1 JSON outputs); 26 tests incl. the client's worked example reproduced exactly; run on the first real walk (session 001). Not yet: HTTP transfer + 3-attempt retry
- [ ] Coverage gap report as a deliverable (document or CSV), with measured vs interpolated
- [ ] Threshold values configurable (RSSI -67 dBm, SNR 20 dB; the thesis lit review's 25 dB noted)
- [ ] Run on real data

## 9. Web interface
- [ ] rosbridge and a Flask or FastAPI backend
- [ ] Configuration tab (dwell 5/10/15 s, spacing, run label, start/stop)
- [ ] Live Monitoring tab (robot on floor plan, waypoint N of total, live RSSI, ETA)
- [ ] Results tab (heatmaps, IDW/Kriging toggle, variance map, CSV and PNG export)
- [ ] Usable by a non-technical person (SUS evaluation)

## 10. Experiments
- [ ] E1 SLAM mapping fidelity
- [ ] E2 Dwell time 5/10/15 s (std at most 3 dBm)
- [ ] E3 Repeatability, 5 runs, same waypoints (std at most 3 dBm)
- [ ] E4 Robot vs walk test (RMSE at most 6 dBm)
- [ ] E5 Sampling interval 0.5/1.0/1.5/2.0 m plus the 0.25 m ground truth (RMSE at most 3.5 dBm)
- [ ] E6 Inaccessible zones, thinning 100/75/50/25% (boundary error at most 4 dBm)
- [ ] E7 Interpolation performance, 20-25% holdout, IDW vs Kriging
- [ ] E8 Usability

## 11. Repo hygiene
- [x] Commit the Dockerfile change, `slam_robot_interpolation` and this checklist
- [ ] README: robot packages, build steps, Pi setup notes
