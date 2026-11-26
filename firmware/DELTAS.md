# Our firmware vs. Yahboom's

Compared with the stock firmware this board ran until 2026-11-26: Yahboom
microROS_Robot **V1.1.3** (May 2024, ESP-IDF 5.1.2, micro-ROS humble). Its
topics, rates and messages were recorded before it was replaced; its settings
were read from its NVS: domain 20, serial agent at 921600 baud, servo offsets
0/0, motor PID 1.0/0.2/0.2, IMU yaw PID 1.0/0/0.2.

## Kept the same

- Node `YB_Car_Node`, domain 20, serial agent at 921600 baud, all topics reliable.
- Topic names, types, frame ids (`odom_frame`, `base_footprint`, `imu_frame`,
  `laser_frame`), odometry covariances, scan layout (360 x 1 deg from -pi,
  0.12-8 m) and its angle mapping, battery as volts x 10.
- Kinematics: wheel speed = v -/+ w x 0.115 m (stock `ROBOT_APB`), 1 m/s wheel
  limit, wheel circumference 150.8 mm (a 48 mm wheel; ours are 45 mm),
  1040 encoder counts per revolution. The ROS side (velocity watchdog
  `angular_gain`, EKF) was tuned against these, so they are kept; all are
  build options now.
- Motor control: 25 kHz PWM, 400-tick period, 200-tick dead zone, incremental
  PID on encoder counts per 10 ms with the stock gains 1.0/0.2/0.2.
- Servo mapping (-90..90 deg -> 500..2500 us, whole degrees) and limits.
- Buzzer semantics. Partition layout (so stock and ours swap cleanly).

## Changed

| What | Stock | Ours | Why |
|---|---|---|---|
| Agent restart | connects once; after an agent restart the board needs a reset | pings the agent every second; after 3 misses tears down, stops the motors and reconnects (~10 s, tested) | restarting `rainbow_microros` no longer needs a board reset |
| `cmd_vel` silent | wheels keep the last command | stop after 0.5 s | safety if the Pi side stops; the velocity watchdog republishes at 10 Hz |
| Zero wheel target | PID keeps running, hunts around zero through the dead zone | that wheel brakes, its PID resets | no jitter at standstill |
| Servos at boot | jump to the firmware's default pose | no pulse until the first command | the head does not swing on every reset; the head node resends its pose every second |
| `odom_raw` rate | 11 Hz | 20 Hz | more samples for the EKF |
| Odometry speed | instantaneous 10 ms encoder speed (1 count = 14.5 mm/s steps) | from count changes over the 50 ms period; pose integrated at the mid heading | less quantization noise |
| `/scan` without lidar | 12.5 Hz of zeros | not published | no fake data |
| `/scan` missing points | keep their old value forever | 0 (invalid) for that revolution | no ghost points |
| `/scan` timing | timer, 12.5 Hz, `scan_time` 0 | once per lidar revolution, `scan_time` = its measured duration | one scan per revolution, not repeats; the speed can change now |
| `/imu` | always, 25 Hz; orientation identity with covariance 0 (claims a perfect orientation), its own RPY filter | off by default (build option `RAINBOW_PUBLISH_IMU`; the chip is then never configured and sleeps). On: raw accel/gyro, `orientation_covariance[0] = -1` (no orientation, REP 145), +-4 g, +-500 deg/s | nothing uses it: saves ~8 % of the serial link and ~1 % of a Pi core in the agent |
| Battery | per sample | averaged over 1 s, rounded | steadier |
| IMU yaw PID | yes (heading hold with the board's IMU, gains in NVS) | no | the board IMU is mounted wrongly; heading belongs on the Pi with the BNO055 |
| WiFi, BLE, WiFi agent | built in | not built | serial only; less power and heat |
| Serial config tool (`config_robot.py`: domain, baud, PID, offsets, namespace, WiFi into NVS) | yes | no: settings are build options | we rebuild and reflash instead; one source of truth in git |
| New: `/esp_firmware` | | latched version string | see what is flashed |
| New: `lidar/enable` | | SetBool service: MS200 standby / run | lidar on/off (see below) |
| New: `lidar_speed_hz` | | node parameter, 5-15 Hz | lidar rotation speed |
| Lidar at boot | spinning (never sends it anything) | standby; the Pi's `lidar_power` turns it on when needed | frugal: no hum, no wear while the robot rests |
| Logs on the serial port | ESP-IDF logs at boot | none after the ROM bootloader | the port carries only micro-ROS |
| Toolchain | ESP-IDF 5.1.2, micro-ROS humble | ESP-IDF 5.2.5, micro-ROS jazzy (matches the agent) | |

## Lidar on/off and speed

The stock firmware never sends the MS200 anything (its TX pin, GPIO17, is set up
but unused). Ours uses the MS200's commands from Oradar's SDK (MIT; the same
frames are in the MS200k manual), at 230400 baud:

| Command | Bytes |
|---|---|
| standby (no ranging, no data) | `A5 F5 A2 C1 01 80 B2 31 F2` |
| run | `A5 F5 A2 C1 01 81 B3 31 F2` |
| speed, e.g. 10 Hz (5-15, sent in standby) | `A5 F5 A1 C1 02 0A 00 38 31 F2` |

As in the SDK, a command is confirmed by the data stream (stopped, started, or
the packets' speed within 10 %), commands are >= 400 ms apart, and a speed change
goes standby -> speed -> run. While off, scans are dropped even if the sensor
keeps sending. Tested on the robot 2026-11-26: standby stops the motor (no hum)
in ~1.5 s, run restarts it (~4 s to "lidar running"), and 5, 7, 10 and 15 Hz all
came out within 1 % (`/scan` rate and `scan_time`). Not known yet: whether the
speed survives a lidar power cycle.

## Yahboom V2.0.0

The newer factory binary (`microROS_Robot_V2.0.0.bin`, April 2024, ESP-IDF
5.1.2-3) has the same topics, the same config protocol and the same settings as
V1.1.3; its strings differ only in build details (logs stripped, no PSRAM use).
Nothing in it to adopt.

## Not yet checked on the robot

Built and run on the board 2026-11-26 with the robot stripped (no lidar,
cameras, ToF; not driven): connection, reconnect after an agent restart, `odom_raw`, `imu`,
`battery`, `esp_firmware` and `lidar/enable` work, `beep` is accepted (not heard). With the lidar
back: `/scan` at 10 Hz, 341 of 360 points valid; standby, run and speed work
(above). Still to check with
the owner watching: each motor's direction and the PID on the floor (teleop
against the gyro), the servos (head), the buzzer heard, and the scan's orientation (front at
index 180) once the lidar is on the chassis.
