#!/usr/bin/env bash
set -Eeuo pipefail

# ROS Humble's generated setup scripts probe optional variables that may be
# unset, so suspend nounset only while sourcing them.
set +u
source /opt/ros/humble/setup.bash
source /opt/hexapod_gait/ros2_ws/install/setup.bash
set -u

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "${XDG_RUNTIME_DIR}"
chmod 0700 "${XDG_RUNTIME_DIR}"

launch_pid=""

stop_simulation() {
    local status=$?
    trap - EXIT INT TERM HUP

    if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
        kill -INT "${launch_pid}" 2>/dev/null || true
        wait "${launch_pid}" 2>/dev/null || true
    fi

    exit "${status}"
}

trap stop_simulation EXIT INT TERM HUP

ros2 launch hexapod_sim robot.launch.py \
    sim:=true \
    rviz:="${HEXAPOD_RVIZ:-true}" \
    use_sim_time:=true &
launch_pid=$!

# Keep the keyboard adapter in the foreground so it owns the interactive TTY.
# Its action-server wait also gives Gazebo and ros2_control time to initialize.
ros2 run hexapod_sim sim_interface.py --ros-args \
    -p wait_timeout:="${HEXAPOD_ACTION_WAIT_TIMEOUT:-60.0}" \
    "$@"
