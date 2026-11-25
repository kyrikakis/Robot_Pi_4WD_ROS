#!/bin/bash
# Run a rainbow_bringup launch file with the full ROS environment.
# Usage: rainbow_launch.sh <launch file> [launch arguments...]
#   e.g. rainbow_launch.sh base.launch.py tof:=false
#
# Supervisor stops a program by signalling its process group. ros2 launch then
# exited at once while the Nav2 container ignored the signal, and supervisor saw
# its program gone and never escalated: every restart of rainbow_navigation left
# the old Nav2 running beside the new one (2026-11-25). So this script stays in
# front: it hands the stop to launch as SIGINT (launch's clean shutdown, which
# escalates to its children itself), waits for it, then stops whatever is still
# in its own process group (by group id, never by name), SIGKILL after a grace
# period.

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"

source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
for ws in /uros_ws /colcon_ws /workspaces/Robot_Pi_4WD_ROS; do
    if [ -f "$ws/install/local_setup.bash" ]; then
        source "$ws/install/local_setup.bash"
    fi
done

GRACE="${RAINBOW_STOP_GRACE:-5}"   # s for leftovers after launch exits, then SIGKILL

ros2 launch rainbow_bringup "$@" &
launch=$!

stopping=0
stop() {
    stopping=1
    kill -INT "$launch" 2>/dev/null
}
trap stop TERM INT

# wait returns early when a trapped signal arrives; keep waiting for launch
while kill -0 "$launch" 2>/dev/null; do
    wait "$launch"
    status=$?
done

# Leftovers in this program's process group (this script leads it under supervisor)
group=$(ps -o pgid= -p $$ | tr -d ' ')
# (not this script, nor the subshell running this and its ps/awk, nor zombies:
# those are dead already and only wait to be reaped)
leftovers() {
    local helper=$BASHPID   # here: inside the pipeline it would be awk's own pid
    ps -eo pid=,ppid=,pgid=,stat= | awk -v g="$group" -v me="$$" -v helper="$helper" \
        '$3 == g && $1 != me && $1 != helper && $2 != helper && $4 !~ /^Z/ { print $1 }'
}
# Only when this script leads its group (as under supervisor): run from a shell,
# the group is the shell's, and its other processes are not ours to stop
if [ "$group" = "$$" ] && [ -n "$(leftovers)" ]; then
    kill -TERM $(leftovers) 2>/dev/null
    for _ in $(seq $((GRACE * 10))); do
        [ -z "$(leftovers)" ] && break
        sleep 0.1
    done
    [ -n "$(leftovers)" ] && kill -KILL $(leftovers) 2>/dev/null
fi

[ "$stopping" = 1 ] && exit 0
exit "${status:-1}"
