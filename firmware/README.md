# ESP32 firmware

Our own firmware for the robot's Yahboom MicroROS board (ESP32-S3, 4 MB flash):
the motors and their PID, the encoders and odometry, the head servos, the lidar,
the board's IMU, the buzzer and the battery reading, talking micro-ROS to the Pi
over the CP2102 USB serial link. It replaces Yahboom's closed binary and keeps
its ROS contract; what differs is listed in [DELTAS.md](DELTAS.md).

## Build

On the Mac or the Pi host (Docker, arm64 or x86):

```
firmware/build.sh              # -> firmware/build/
firmware/build.sh menuconfig   # any idf.py command; settings: "Rainbow firmware"
```

The first build clones and compiles micro-ROS (~10 min); later builds take
seconds. Pins: the ESP-IDF image by digest and the pip packages in
`docker/Dockerfile`, the micro-ROS component commit in `build.sh`, the micro-ROS
sources in `micro_ros.lock`, `bdc_motor` in `dependencies.lock`.

Settings (wheel size, PID gains, kinematics, timeout, servo offsets, baud,
domain) are build options in `main/Kconfig.projbuild`; change the defaults there
(or `sdkconfig.defaults`) so they stay in git. `sdkconfig` itself is generated.

## Flash, back up, restore

On the Pi host, where the board is plugged in (`/dev/ttyUSB_ROS`):

```
firmware/esp.sh flash [build_dir]   # default firmware/build
firmware/esp.sh backup [file]       # whole 4 MB flash
firmware/esp.sh restore <file>      # whole-flash image, e.g. the stock backup
firmware/esp.sh info
```

`esp.sh` stops the micro-ROS agent (`rainbow_microros`) for the port and starts
it again; esptool resets the board after writing. It installs a pinned esptool
in `~/esptool-venv` on first use. Flashing does not touch the NVS partition.

A build made on the Mac is copied to the Pi with its `flash_args`
(`bootloader/bootloader.bin`, `partition_table/partition-table.bin`,
`rainbow_esp.bin`), e.g. into `~/esp-builds/<version>/`, then
`esp.sh flash ~/esp-builds/<version>`.

The stock firmware's whole-flash backup (Yahboom V1.1.3, read from this board
2026-11-26, sha256 `39a031317a0c1fbd798c4c4c7cedde33091452ed160fad960ce7714fc0ddb347`)
is kept outside git (it holds Yahboom's WiFi/BLE defaults): in `~/esp-backup/`
on the Pi host and on the owner's Mac. Restoring it was tested.

Check what is running: `ros2 topic echo --once /esp_firmware` (domain 20).

## ROS interface (node `YB_Car_Node`, domain 20, all reliable)

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `cmd_vel` (sub) | geometry_msgs/Twist | | skid-steer: `linear.x`, `angular.z`; `linear.y` ignored. Silent for 0.5 s -> stop |
| `servo_s1`, `servo_s2` (sub) | std_msgs/Int32 | | degrees; S1 pan [-90, 90], S2 tilt [-90, 20] |
| `beep` (sub) | std_msgs/UInt16 | | 0 off, 1 on, >= 10 on for that many ms |
| `odom_raw` | nav_msgs/Odometry | 20 Hz | `odom_frame` -> `base_footprint`, wheel odometry |
| `imu` | sensor_msgs/Imu | 25 Hz | `imu_frame`, the board's ICM-42670-P (mounted wrongly, unused) |
| `scan` | sensor_msgs/LaserScan | per revolution | `laser_frame`, 360 x 1 deg from -pi, 0.12-8 m; only while the lidar sends |
| `battery` | std_msgs/UInt16 | 1 Hz | volts x 10 |
| `esp_firmware` | std_msgs/String | latched | version, build date, IDF version |
| `lidar/enable` (service) | std_srvs/SetBool | | stop/start reading the lidar (software only) |

## Code

`main/`: `uros.c` (node, topics, agent connection), `motors.c` (PWM, encoders,
PID), `lidar.c` (MS200), `imu.c`, `servos.c`, `beep.c`, `battery.c`,
`transport.c` (UART0). The board's pin map is at the top of `rainbow.h`. All
written against ESP-IDF's drivers; the pin map and constants were taken from
Yahboom's sample code, none of their code is used.
