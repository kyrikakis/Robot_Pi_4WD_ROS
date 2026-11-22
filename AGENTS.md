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
- Supervisor runs eight of its nine programs from `supervisor/rainbow.conf`, each a
  `rainbow_bringup` launch file via `scripts/rainbow_launch.sh`. The micro-ROS
  agent (`rainbow_microros`) and the ToF (`rainbow_tof`) are separate programs:
  restarting the base disconnects neither the ESP nor the ToF.
- Localization is map_server + AMCL (`rainbow_localization`) on a saved map in
  `maps/`, read-only. slam_toolbox (`rainbow_slam`, autostart off) is only for
  mapping a new place; never run both (both publish map -> odom). Its
  localization mode was dropped: it adds its scans to the map and damaged it
  whenever it lost track. AMCL starts with global localization (no start
  pose); the pose settles once the robot drives or turns a little.
- **Restarting `rainbow_base` restarts the EKF, which resets the odometry: AMCL
  then needs a pose estimate, and a SLAM mapping session gets scrambled
  (restart `rainbow_slam` right after). Ask the owner before restarting the
  base.** Restarting Nav2, the ToF or the brain is safe.
- Nav2 runs composed in one process (`nav2_container`); as separate processes
  its 11 servers cost ~66% of a core idle, composed ~38%.
- `ROS_DOMAIN_ID=20` for the robot.
- The voice node (`rainbow_voice`, program `rainbow_voice`) owns the ReSpeaker
  Lite's microphone (single owner: stop the program before recording from it).
  It reads it with `arecord`; its models are in `/opt/rainbow/voice` (pinned
  downloads in the Dockerfile). Test it without the mic by putting a stand-in
  `arecord` that streams a raw 16 kHz stereo file first on its PATH.
- The brain's link to the Mac agent (`agent/`) is a WebSocket on port 8766;
  its token is `/data/rainbow/link_token` (`rainbow-agent pair` copies it).
  `agent/tests/test_link.py` runs both ends against each other on the Mac.
- Data the robot writes at runtime (places, the link token) lives on the
  `rainbow_data` Docker volume at `/data/rainbow`, never in the image or `maps/`.
  Places are per map (`places/<map>.yaml`, map name taken from
  `localization.yaml`): a new map needs new places.
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
- **ESP connection**: if `/scan`, `/odom_raw` and `/battery` stop, the ESP's CP2102
  USB serial chip may have locked up (`dmesg`: `cp210x ... failed set request`).
  It is powered by the Pi, so a battery reset does not clear it; re-enumerate it:
  `echo 0 > /sys/bus/usb/devices/1-2/authorized; sleep 2; echo 1 > .../authorized`.
  The agent reconnects by itself. Avoid restarting `rainbow_microros`: after the
  agent stops, the ESP itself needs a board reset (with the agent running).
- The ToF is in both costmaps (STVL). The node does the floor filtering and
  publishes `/tof/obstacles` (2-16 cm above the floor within 0.7 m, 2.5-16 cm
  beyond; Nav2 marks only from this), after dropping flying pixels and the image
  edges. The floor test is levelled with the BNO055's roll/pitch (relative to a
  5 s average) and measured from a floor plane fitted in each frame (the robot
  resting tilted on a carpet edge made the floor read ~1.5 cm low, hiding a
  3-4 cm desk foot), and obstacle pixels need half their 5x5 neighbourhood to be
  obstacles too (floor specks were marked as ghosts). Frames have no SDK
  timestamp: they are stamped on arrival minus `latency` (0.04 s, estimated);
  late stamps smeared obstacles into ghosts while turning. Clearing uses
  `/point_cloud`, 0.06-1.5 m in the central ~90% of the view (matching the edge
  crop), where voxels fade within ~1-2 s (`decay_acceleration` 5); out-of-view
  voxels fade after 30 s.
- Narrow passages: footprint padding 0; the controller checks collisions with
  the footprint (it collides only where the footprint touches an obstacle
  cell). Planner inflation 0.35 m / factor 6 keeps paths off walls in
  open space; controller 0.20 m / factor 10 does not fill gaps. A 28 cm gap
  passes. Centre-only checking made it scrape the bed.
- Planner: Theta* (any-angle). Smac 2D's 8-direction grid turned straight
  corridors at odd angles into a bend that smoothing made a curve. Theta*
  checks its lines along the centre only, so paths can pass close to corners;
  the controller's footprint check and the collision monitor catch that. Its
  path poses have no frame_id, which Foxglove rejects: view `/plan_viz`
  (republished by `plan_viz`), not `/plan`.
- Turning: commands reach the gyro ~0.3 s late (recorded) and the robot turns
  at ~80-87% of the commanded rate. The velocity watchdog multiplies turn rate
  by `angular_gain` 1.2. The controller is Nav2's graceful controller: it turns
  in place with speed proportional to the angle left
  (`rotation_scaling_factor` 0.6), so it slows into the heading. `beta` 0.1
  (default 0.4 crawled in every curve) and in-place turns only above ~46 deg
  (`initial_rotation_tolerance` 0.8; 0.5 stopped at every corner). MPPI was
  dropped: with the rotation shim it ping-ponged around every heading, alone it
  drove in circles. Regulated Pure Pursuit was tried and dropped: steady, but its
  in-place turn never slows near the heading and overshot a lot.
- Tight arcs (inner wheels below `min_inner_ratio` 0.15 of the outer) become
  turns in place in the velocity watchdog: pivoting on one side scrubs both
  sides and the wheel odometry reports motion that did not happen. At 0.3 slow
  curves stopped too often.
- Nav2's collision monitor (approach mode, footprint, lidar + `/tof/obstacles`)
  slows Nav2's commands so the footprint stays >= 1.2 s from a collision, then
  stops. It acts after the velocity smoother, so the soft normal braking (0.8
  m/s^2) cannot delay an emergency stop. Teleop does not pass through it.
- Nav2 does not drive backwards (`allow_backward: false`): nothing sees low
  obstacles behind. The only exception is the recovery in
  `rainbow_bringup/behavior_trees/`: spin left, spin right, then back up 10 cm
  at 5 cm/s. With its nose within ~2 cm of an obstacle the 19 x 14 cm footprint's
  front corners (12.2 cm from the centre) hit it turning either way, and the
  ToF's 30 s memory keeps it after a costmap clear (blind zone).
- Frames and IMU are verified: base_footprint -> base_link is a fixed joint, so
  mixing them across configs is fine; the BNO055 remap (P2) is right on all axes.
  Wheel odometry over-reports rotation ~16% (skid-steer slip); the EKF takes
  only forward/sideways speed from the wheels, yaw and yaw rate from the BNO055.
- Wheel slip (rugs grip the robot, wheels keep turning) fuses or rotates rooms
  in the SLAM map. A lidar-vs-wheels "slip guard" was tried and dropped: its
  crude lidar estimate zeroed real motion and broke the map in straight lines.
  Do not loosen SLAM's trust in odometry (`distance_variance_penalty` 0.3,
  search 0.8 m) either: tried twice, it made wrong matches and worse maps.
- The lidar has no per-point timing; scans taken while turning are bent, so
  SLAM adds one only every ~29 deg of turning (`minimum_travel_heading` 0.5).
  Do not hold scans back from SLAM: its map->odom TF is stamped with the last
  scan and goes stale, and Nav2 then aborts every goal.
- After an ESP reset its servos go to the firmware's default position; the head
  node resends the pose every second so the head (and the calibrated ToF) return.
  Do not move the head without the owner watching.
- ToF calibration: `scripts/calibrate_tof.py` (robot facing clear floor, head
  centred) gives the FOV and the mount pitch/roll. Current values: FOV 48.5 x
  39.3 deg (in base.launch.py; the spec's 64.3 x 50.4 is wrong), pitch +2.6 deg,
  roll -2.2 deg (dimensions.xacro). Floor then reads within ~1-2 cm to 1 m.
  Against the lidar (the reference) the ToF placed things ~8.5 cm too far and
  4 cm left: fixed with `tof_dx` -0.085 m and `tof_yaw` -1.5 deg (dimensions.xacro),
  now within ~2 cm. Not with a range offset (`range_offset`, kept at 0): that
  also lifted the floor ~3-4 cm and filled the costmaps with ghosts.
- With the ESP board off there is no odometry or lidar, so SLAM and Nav2 start
  but cannot do anything useful. Do not read that as a failure.
- The Hailo container's supervisor socket is reachable from here at
  `/proc/<its supervisord pid>/root/run/supervisor.sock` (this container shares
  the host PID namespace).

## Conventions

- `main` is the finished robot. New work happens on a branch per feature
  (currently `brain`, for `docs/brain.md`); do not merge to `main` or suggest
  merging until the owner says so.
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
- ToF blind zone: ~20-25 cm in front of the nose and outside +-24 deg to the
  sides (head level); obstacles there are only known from memory (30 s). A
  head tilt down would shrink it; try step by step with the owner watching.
- Brain and voice (`docs/brain.md`, branch `brain`): steps 1-3 built and tested
  on the Pi on domain 77 (no driving yet). Still to do: the image rebuild and a
  test with the owner watching (driving through the link, speaking); train
  "Rainbow" and "stop" wake word models (the wake word is the placeholder
  "hey Jarvis" until then, and "stop" only works after it); re-enable face
  recognition in the Hailo container (photos, faces and startup localization
  depend on it); the head controller. Memory and photos/faces (steps 5-6) are
  built.
- Battery: warn around 7.0 V (log, eyes or speech) and cancel Nav2 goals /
  stop at the cutoff (~6.6-6.9 V, judged at rest). The firmware's own low-voltage
  behaviour is unknown. Not part of the brain work for now; later it becomes a
  brain mode that refuses motion.
- Calibrate the Camera Module 3 (the files in `calibration/` are for the IMX296).
- Whisper on the Hailo (speech to text without the Mac): later, after the brain.
- Hailo face recognition is disabled inside its container to save CPU; it is to
  be re-enabled for the brain (stills and face names, see `docs/brain.md`).
- Host: Hailo, kernel (6.6, contains the Hailo driver) and camera packages are
  held with apt-mark. Update them together later, with the Hailo container
  and a rebuild of the ALFA Wi-Fi driver (rtl8812au) for the new kernel.
  Debian 12 is in LTS until mid-2028; a move to Debian 13 is due eventually.
