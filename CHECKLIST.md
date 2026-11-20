# slam-robot project checklist

Legend: `[x]` done and verified, `[~]` written and tested here but not yet proven on the robot or Pi, `[ ]` not started.
Last updated: 2026-11-18.

## 1. Pi and environment
- [x] Ubuntu Server 22.04 arm64 on the Pi 4B, headless, SSH works (`robot@slam.local`)
- [x] ROS 2 Humble installed natively (ros-base, slam_toolbox, nav2, robot_localization, rosbridge, laser_filters)
- [x] Docker build environment for Humble on the laptop (`docker/run.sh`)
- [x] Workspace pushed to private GitHub repo `itsJayMendoza/slam-robot`
- [x] Deploy key on the Pi, repo cloned at `~/slam-robot`, robot packages built natively
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
- [x] Hand-carried walk test (session 001, 5 spots, data and heatmap in `data/sessions/001`): `walk_survey` (guided stop-and-scan, writes records.json / waypoints.json / notification.json per the T1 format, one record per beacon, with encryption), `make_room_map` (room size -> map.yaml/.pgm); run on the Pi on 2026-11-13
- [ ] Repeatability: 3 back-to-back scans without moving anything; on-channel means should agree within a couple of dB
- [ ] Check which APs broadcast the BSS Load IE (channel utilization)
- [ ] Noise-floor test in several rooms (confirm the field is always absent)
- [ ] Monitor-mode setup at boot (systemd unit; never touch `wlan0`)
- [ ] udev rule: no USB autosuspend for the T3U (and later the LiDAR and ESP32)
- [ ] Antenna mast at 0.7-1.2 m, USB 2.0 extension cable, antenna orientation fixed
- [ ] Optional: RTL-SDR as a separate noise-floor sensor
- [ ] Document noise floor and SNR as a limitation

## 3. Chassis, power, electronics
- [ ] Assemble chassis, JGA25-360 motors, ball caster, second deck
- [x] Wiring per the plan, done on the robot: TB6612 PWMA/AIN1/AIN2 on GPIO 25/26/27, PWMB/BIN1/BIN2 on 14/17/13 (not 12), STBY on 33; encoders on 34/35 and 32/39 (blue VCC to 3.3 V); common ground. Motor labels: red M+, black M-, blue encoder VCC, green GND, yellow A, white B (the product description's colours are wrong)
- [~] MPU6050 on the Pi header (VCC pin 1, SDA pin 3, SCL pin 5, GND pin 6): wired and answering at 0x68 (chip is an MPU6500); not yet mounted: rigid, flat, near the axle centre, away from the motor wires
- [x] Encoders are clean quadrature (0 invalid steps, both wires equal edge counts); about 49,200 counts per wheel turn at 4x decoding (left wheel turned once by hand while reading the base firmware; the earlier figure of 5,000 was a misread one-turn test that really turned about 1/8 turn); 2 m floor test (firmware 0.4, 0.10 m/s, 20.6 s): 217,200 counts per metre, which matches 72 mm wheels at 49,100 counts per turn; left and right within 0.1%; right wheel turn count still to measure
- [x] Motor test (`firmware/src/motor_test`, 12 V bench supply, wheels in the air): left motor direction and right encoder sign are reversed (fix in firmware); speed rises evenly with duty, same both ways, left about 5-10% faster; wheels start at duty 8/255; speeds noted here in m/s were about 8x too high (wrong counts per turn), the motors are in line with the listed 201 rpm. A screw rubbing the left wheel was found and fixed
- [~] Floor test: 2 m straight at 0.10 m/s done over SSH (1.65 m measured, 217,200 counts per metre, firmware ran on 12 V supply); still to do: sideways drift, lowest controllable speed under load, gain tuning
- [ ] Power: 3S LiPo, DC-rated switch, inline fuse, XL4016 set to about 5.1 V, separate Pi rail
- [ ] Power ESP32 and LiDAR from the buck rail if the Pi's USB (about 1.2 A total) is too tight; test with all devices under load
- [ ] Measure battery runtime (Pi + LiDAR + motors); plan sessions for the long 0.5 m and 0.25 m surveys
- [ ] Mast stability, low ballast, doorway clearance of at least 80 cm
- [ ] Measure track width on the built chassis (wheel radius 36 mm)

## 4. ESP32 firmware and base bridge
- [~] Firmware `firmware/src/base`: 11-bit PWM at 20 kHz, direction fixes (both wheels confirmed forward), encoders on the pulse-counter hardware, wheel-speed PI with feedforward, 0.23 m/s limit (50,000 counts/s; was 7,000 counts/s, only about 4 cm/s, before the rescale), 500 ms command timeout (verified), stall / wrong-direction / overspeed faults (overspeed verified when a sign error made a wheel run away); bench-tested in the air: holds 5,000 counts/s within ±200, jerky below about 2,000 counts/s (wheels at the start-up threshold); gains to tune on the floor
- [x] USB serial protocol: ASCII lines, `v <l> <r>` in counts/s answered by encoder counts, speeds and fault bits (documented in `firmware/src/base/main.cpp`)
- [~] Base on the Pi: ros2_control hardware interface + `diff_drive_controller` (as in andino and bumperbot) giving `/odom`, `/cmd_vel`, TF odom to base_link. `slam_robot_hardware` (C++ SystemInterface, firmware protocol, 5 gtests) and `slam_robot_bringup` (`base.launch.py`, `controllers.yaml`: separation 0.180 m, radius 0.036 m, 0.20 m/s and 1.5 rad/s limits) written; verified with mock hardware in Docker (controllers active, /odom and TF move with /cmd_vel); runs on the robot: connects to firmware 0.4, both controllers active, teleop_twist_keyboard over SSH drives every direction correctly (wheels in the air); floor run through ROS still to do
- [~] `slam_robot_imu`: MPU6050 on the Pi's I2C bus, `imu/data_raw` (gyro + accel, covariances from a stationary calibration, `~/calibrate` service for re-measuring the bias at a waypoint), `imu_check` CLI; 21 tests pass; bench-tested on the Pi: 100 Hz, gyro z noise about 0.12 deg/s, stable bias. Still to do: 90-degree turn check (sign and scale), `imu_node` rate check, test on the robot
- [~] `slam_robot_base`: serial link + `base_drive` (status, `straight` with counts-per-metre calibration and straight-line hold, keyboard `teleop`) for floor tests over SSH; 12 tests pass; drove 2 m on the floor over SSH from the Pi; keyboard teleop on the floor works (forward, reverse, turns on the spot)
- [~] Teleop through ROS (`/cmd_vel`): works from an SSH session on the Pi; from the laptop over `ROS_DOMAIN_ID=42` still to try

## 5. LiDAR, fusion, mapping
- [x] YDLidar-SDK and `ydlidar_ros2_driver` on the Pi (`scripts/setup_ydlidar_pi.sh`; `lidar.launch.py`; X3 on /dev/ydlidar, 115200 baud, `isSingleChannel: true`, /scan about 6.5 Hz, 620 points, even coverage over 360 degrees)
- [~] Verify X3 minimum range, blind spot, glass and dark-surface behavior: scan-plane check done (T3U antenna folded below the scan plane; it had blanked 15-20 and 40-50 deg). A fixed blind slice remains at about -157 deg (back right, 70-80% invalid in two robot orientations), probably a screw head next to the lidar; it gives invalid readings, not false points, so it is left unmasked. Corners, openings and far clutter cause the other invalid sectors. Glass and dark surfaces still to check
- [ ] Mask the mast angular slice (`ignore_array` or laser_filters)
- [~] URDF and TF tree for the real robot (`slam_robot_description`): the 3D-printed chassis meshes from the reference repo (Apache-2.0, credited in meshes/README.md) with 72 mm wheels 0.180 m apart on the motor mounts, the front caster only, IMU at the chassis mount, lidar 40 mm ahead of the axle at the chassis mount height; X3 direction verified by the box test (box 50 cm ahead at 0 deg / 0.53 m, on the left at +90 deg), so `laser_yaw` stays 0
- [~] EKF (`robot_localization`) fusing odometry and IMU: `ekf.yaml` (wheel vx/vy/yaw rate + gyro yaw rate, 2D) and `robot.launch.py` (base + IMU + lidar + EKF; diff_drive_controller leaves the odom TF to the EKF). Docker test with mock wheels and a fake gyro: EKF follows a 0.1 m/s, 0.5 rad/s arc (yaw 119 deg vs 115 commanded), only the EKF publishes odom -> base_footprint. imu_node now publishes reliable QoS so robot_localization receives it. Runs on the robot (`robot.launch.py`: IMU 95 Hz, /odom 49 Hz, EKF 29 Hz, /scan; still-standing heading creep about 2 deg/min). Spin test with the lidar as reference (start/end scans matched): one turn left was 366.5 deg, right 370.0 deg; wheels counted 2.7-2.8% short both ways, so `wheel_separation_multiplier: 0.973`; gyro read 1-4% high, about 2% after separating bias and scale, so `gyro_z_scale: 0.980`. Check spin after the corrections (left, true 363.5 deg from the lidar): wheels 363.3, gyro 364.4, EKF 364.4, all within 0.3%
- [~] `slam_toolbox` mapping run, map saved: `slam.yaml` (5 cm cells, 8 m range, a scan every 15 cm or 8.6 deg), `mapping.launch.py`, laptop viewer `scripts/view_mapping.sh` + `rviz/mapping.rviz`. Docker test with a simulated 5 x 3.5 m room and a 0.4 m box: walls within 1 cm, box found, robot pose on the map within 1 cm of the truth, map saved with map_saver_cli. Real-room run still to do
- [ ] Experiment 1 targets: wall RMSE at most 0.10 m, overlap at least 85%
- [ ] Document that the X3 is the LiDAR used (the thesis text names the LD06)

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
- [~] `slam_robot_postprocess`: the T1 post-processing pipeline, phases A-D (session checks, untouched raw copy, cleaning with per-reason counts incl. off-channel and the adapter's -239 placeholder, median merge, per-SSID sets, grid from map.yaml, IDW, Kriging with min-points/fail handling, all-networks view, T1 JSON outputs); 26 tests incl. the T1 worked example reproduced exactly; run on the first real walk (session 001). Not yet: HTTP transfer + 3-attempt retry
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
