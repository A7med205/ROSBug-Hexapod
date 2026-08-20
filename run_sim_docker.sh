#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
image_name="${HEXAPOD_DOCKER_IMAGE:-hexapod-gait:humble}"
container_name="${HEXAPOD_DOCKER_CONTAINER:-hexapod-gait-sim}"
build_image=true
rviz=true

usage() {
    cat <<'EOF'
Usage: ./run_sim_docker.sh [options]

Build and run the ROS 2 Humble hexapod simulation with an interactive keyboard.

Options:
  --no-build       Run the existing image without rebuilding it
  --no-rviz        Run Gazebo without RViz
  --image NAME     Override the image name/tag
  -h, --help       Show this help

Environment:
  HEXAPOD_DOCKER_IMAGE       Default image name/tag
  HEXAPOD_DOCKER_CONTAINER   Default container name
  HEXAPOD_ACTION_WAIT_TIMEOUT  Seconds to wait for the trajectory action server
EOF
}

while (($#)); do
    case "$1" in
        --no-build)
            build_image=false
            ;;
        --no-rviz)
            rviz=false
            ;;
        --image)
            if (($# < 2)); then
                echo "--image requires a value" >&2
                exit 2
            fi
            image_name=$2
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY is not set; Gazebo and RViz cannot connect to the host display." >&2
    exit 1
fi

command -v docker >/dev/null || {
    echo "Docker is not installed or is not on PATH." >&2
    exit 1
}
command -v xhost >/dev/null || {
    echo "xhost is required for local X11 display access." >&2
    exit 1
}

if [[ "${build_image}" == true ]]; then
    docker build --tag "${image_name}" "${script_dir}"
fi

xhost +SI:localuser:root >/dev/null
restore_xhost() {
    xhost -SI:localuser:root >/dev/null 2>&1 || true
}
trap restore_xhost EXIT INT TERM HUP

docker_args=(
    run
    --rm
    --interactive
    --tty
    --name "${container_name}"
    --network host
    --ipc host
    --env "DISPLAY=${DISPLAY}"
    --env "QT_X11_NO_MITSHM=1"
    --env "HEXAPOD_RVIZ=${rviz}"
    --env "HEXAPOD_ACTION_WAIT_TIMEOUT=${HEXAPOD_ACTION_WAIT_TIMEOUT:-60.0}"
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
)

if [[ -d /dev/dri ]]; then
    docker_args+=(--device /dev/dri:/dev/dri)
fi

docker "${docker_args[@]}" "${image_name}"
