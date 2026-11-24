# The brain's view of the robot, with the Mac agent's status on top (/brain/status, for
# Foxglove). Published every second and whenever something changes.
builtin_interfaces/Time stamp
# local (no agent) or remote (agent connected), and the map
string mode
string map
# What the brain is doing, e.g. "idle", "going to kitchen", "turning", "looking around"
string activity
# Localization (AMCL): certainty word (sure, roughly, lost, unknown), pose in the map,
# standard deviations (m, rad; negative when unknown). localized: only when sure
bool localized
string certainty
float64 x
float64 y
float64 yaw
float64 position_std
float64 yaw_std
# Volts; 0 when no /battery message yet
float32 battery_volts
# Seconds since the last message from the ESP board (battery 1 Hz, lidar, wheel odometry);
# negative when none arrived yet. All three growing: the ESP link is down (AGENTS.md)
float32 battery_age
float32 scan_age
float32 odom_age
# Nav2's navigator, the speech node and face recognition are up
bool nav2_ready
bool speech_ready
bool faces_ready
uint32 places
# The last event for the agent (e.g. "arrived kitchen") and the last local voice command
string last_event
string last_command
# Seconds the brain has been running
float32 uptime
# The Mac agent (agent.connected false: last known values)
AgentStatus agent
