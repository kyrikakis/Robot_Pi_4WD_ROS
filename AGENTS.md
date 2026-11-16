# AGENTS.md

Guidance for AI coding agents (and people) working on this repo. Read the
README first for hardware setup and day-to-day commands; this file covers what
the code does not tell you.

## What this is

Rainbow is a hobby robot: a small 4WD skid-steer chassis (19 x 14 cm) with a
pan/tilt head, on a Raspberry Pi 5 running ROS 2 Jazzy in Docker. Everything
ROS-side is in the `rainbow_*` packages under `src/` (see the README table).

The hardware is split across the ESP board and two containers. The Pi host
itself only runs Docker, the Hailo driver/firmware/runtime service
(`hailort.service`) and system config; it runs no ROS. The Hailo container's
compose setup lives in `~/hailo_tappas_ros2` on the host.

| Where | What | In this repo? |
|---|---|---|
| ESP32 board (micro-ROS over `/dev/ttyUSB_ROS`) | Motor PID, head servos, lidar, battery, an IMU | No. Firmware is out of scope; treat its topics as a fixed contract |
| `robot` container (this repo) | Everything in `src/`, run by supervisor | Yes |
| Hailo container ([hailo_tappas_ros2](https://github.com/kyrikakis/hailo_tappas_ros2)) | Face recognition on the Hailo, capturing the RGB camera itself | No. Only `hailo_msgs` is built here, to read its messages |

Contracts with the ESP firmware (node `YB_Car_Node`, do not rename): it
subscribes to `cmd_vel`, `servo_s1` (pan), `servo_s2` (tilt) and `/beep`, and
publishes `/odom_raw`, `/scan` in `laser_frame`, `/imu` in `imu_frame`, and
`/battery` (UInt16, volts x 10, 1 Hz; the owner's pack runs ~6.9-8.0 V). The ESP's IMU is mounted wrongly and
unused; the EKF fuses `/odom_raw` with the BNO055 (`/bno055/imu`, `imu2_frame`).

## How it runs

- Appliance style: the Dockerfile copies the repo into the image and builds it,
  so a power-on is a clean deployment. This is deliberate; do not bind-mount
  the repo. The image is built from the **Pi host's** checkout, not from inside
  the container.
- Supervisor runs five programs from `supervisor/rainbow.conf`, each a
  `rainbow_bringup` launch file via `scripts/rainbow_launch.sh`.
- Nav2 runs composed in one process (`nav2_container`); as separate processes
  its 11 servers cost ~66% of a core idle, composed ~38%.
- `ROS_DOMAIN_ID=20` for the robot.
- All measurements live in `src/rainbow_description/urdf/dimensions.xacro`
  (measured values are marked). Nav2's footprint in `rainbow_bringup/config/nav2.yaml`
  must be kept in sync with it by hand.

## Working on the live robot

The code you edit is the code the robot runs. Lessons learned:

- **Test new nodes on another ROS domain** (`ROS_DOMAIN_ID=77`) so they cannot
  command the real robot or clash with its TF.
- **Never stop processes by name pattern** (`pkill -f joint_state_publisher`
  also kills the robot's own). Stop test processes by PID. `setsid` forks, so
  `$!` after it is not the node's PID.
- **Hardware is single-owner**: the I2C eyes, BNO055 and the ToF camera can only
  be opened by one process; test their nodes by restarting the supervisor
  program, not alongside it.
- `supervisorctl tail` can fail on non-UTF-8 log output; read
  `/var/log/supervisor/<program>-stdout*.log` directly.
- **ROS params files** (`config/*.yaml`) must only have node names at the top
  level: YAML anchors defined as extra top-level keys make every node fail to
  start. Validate a params file by starting one real node with it on domain 77.
- The ToF publishes with sensor-data QoS (best effort); subscribers must match.
  The sensor has no frame-rate control (always ~25-30 fps; the SDK rejects
  FRAME_RATE), so `frame_rate` is applied by skipping frames. Its range is in
  mm (2000 or 4000; we use 4000) and only accepted after `start()`. Nearly all
  of the node's CPU (~18%) is the SDK's own depth computation, which no setting
  reduces; the node's own processing is a percent or two.
- With the ESP board off there is no odometry or lidar, so SLAM and Nav2 start
  but cannot do anything useful. Do not read that as a failure.
- The Hailo container's supervisor socket is reachable from here at
  `/proc/<its supervisord pid>/root/run/supervisor.sock` (this container shares
  the host PID namespace).

## Conventions

- Work happens on the `refactor` branch until the refactor is finished; do not
  merge to `main` or suggest merging until the owner says so.
- Python nodes: ament_python, one node per module, `ament_flake8 --linelength 100`
  clean. New services go in `rainbow_interfaces`. Keep the old topics
  (`/head/*_rotation`, `/eyes`, `/speech/*`) working.
- Pin anything cloned or pip-installed in the Dockerfile (pins are `ARG`s at
  the top). Pin pip dependencies that can break things too, not only the
  packages you asked for: an unpinned Adafruit-Blinka upgrade once broke the
  eyes. Ubuntu/ROS apt packages are the exception, with `apt-get upgrade`
  kept on purpose.
- The README is curated: change it only to keep it correct, never pad it.
- The version before the refactor is tagged `v0-legacy`.

## Pushing from the container

The Pi host's checkout fetches over HTTPS (the host has no GitHub key) and
pushes over SSH; the image copies that `.git`, so the container inherits both.

The container has no SSH keys; VS Code forwards the laptop's agent, which holds
a work key and the owner's personal key. Pin this repo to the personal one
(lost on every container rebuild):

```
ssh-add -L | grep 'kirikakis@gmail.com' > ~/.ssh/id_ed25519_personal.pub
git config core.sshCommand "ssh -i ~/.ssh/id_ed25519_personal.pub -o IdentitiesOnly=yes"
```

## Open work

Keep this list current: when an item is done, remove it in the same commit;
when new work comes up, add it here.

- Drive test with the ESP on (teleop, head directions, lidar/odometry, Nav2
  in a small room). The image rebuild of the refactor is done.
- ToF is out of Nav2 (voxel_layer removed from both costmaps' plugin lists in
  nav2.yaml) because it added noise. Debug the noise (confidence threshold,
  floor points, frame orientation), then re-enable and check obstacles clear
  in `/local_costmap/voxel_grid`. The ToF node itself still runs.
- Battery node: warn around 7.0 V (log, eyes or speech) and cancel Nav2 goals /
  stop at the cutoff (~6.6-6.9 V, judged at rest). The firmware's own low-voltage
  behaviour is unknown.
- Calibrate the Camera Module 3 (the files in `calibration/` are for the IMX296).
- Wake word "Hey Rainbow" (placeholder in `rainbow_voice`); speech to text is
  planned with Whisper on the Hailo.
- Hailo face recognition is currently disabled inside its container to save CPU.
- Host: Hailo, kernel (6.6, contains the Hailo driver) and camera packages are
  held with apt-mark. Update them together later, with the Hailo container
  and a rebuild of the ALFA Wi-Fi driver (rtl8812au) for the new kernel.
  Debian 12 is in LTS until mid-2028; a move to Debian 13 is due eventually.
