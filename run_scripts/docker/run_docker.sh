#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# run_docker.sh  –  Start the spray-paint simulation container
#
# Usage:
#   ./run_scripts/docker/run_docker.sh [container_name=<name>] [headless] [detach] [empty_container]
#
# Arguments (all optional):
#   container_name=<name>   Override the default container name
#   headless                Run gz sim server-only (no GUI), default is GUI mode
#   detach                  Start container in background with sleep infinity
#                           (used by startScript.sh; launch files run via docker exec)
#   empty_container         Open an interactive bash shell in the container
#
# Examples:
#   ./run_scripts/docker/run_docker.sh
#   ./run_scripts/docker/run_docker.sh headless
#   ./run_scripts/docker/run_docker.sh empty_container
#   ./run_scripts/docker/run_docker.sh container_name=my_sim detach
# ─────────────────────────────────────────────────────────────────────────────

set -e

# ── Defaults ──────────────────────────────────────────────────────────────────
CONTAINER_NAME="spray_paint_stack"
HEADLESS=false
MODE="standalone"   # standalone | detach | empty_container

# ── Argument parsing ──────────────────────────────────────────────────────────
for arg in "$@"; do
    case $arg in
        container_name=*)
            CONTAINER_NAME="${arg#*=}"
            ;;
        headless)
            HEADLESS=true
            ;;
        detach)
            MODE="detach"
            ;;
        empty_container)
            MODE="empty_container"
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Allowed arguments:"
            echo "  container_name=<name>"
            echo "  headless"
            echo "  detach"
            echo "  empty_container"
            exit 1
            ;;
    esac
done

# ── Paths ─────────────────────────────────────────────────────────────────────
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." >/dev/null 2>&1 && pwd)"
PLATFORM="$(uname -m)"
IMAGE_NAME="spray_paint_plugin"

# ── Host user identity ────────────────────────────────────────────────────────
HOST_UID=$(id -u)
HOST_GID=$(id -g)
HOST_USER=$(id -un)

PLUGIN_PATH_INSIDE="/ws/install/gz_sim_spray_painting_plugin/lib/gz_sim_spray_painting_plugin"

echo ""
echo "  Container : $CONTAINER_NAME"
echo "  Image     : $IMAGE_NAME"
echo "  Platform  : $PLATFORM"
echo "  Headless  : $HEADLESS"
echo "  Mode      : $MODE"
echo ""

# ── Stop and remove any existing container with the same name ─────────────────
if [ "$(docker ps -q --filter "name=^/${CONTAINER_NAME}$")" ]; then
    echo "-> Stopping existing container '$CONTAINER_NAME'..."
    docker stop "$CONTAINER_NAME" > /dev/null
    sleep 1
fi
if [ "$(docker ps -a -q --filter "name=^/${CONTAINER_NAME}$")" ]; then
    echo "-> Removing existing container '$CONTAINER_NAME'..."
    docker rm "$CONTAINER_NAME" > /dev/null
fi

# ── Base docker args ──────────────────────────────────────────────────────────
DOCKER_ARGS=()

# ── X11 / display ─────────────────────────────────────────────────────────────
if [ "$HEADLESS" = false ]; then
    xhost +local:docker > /dev/null 2>&1 || true
    DOCKER_ARGS+=("-e" "DISPLAY=$DISPLAY")
    DOCKER_ARGS+=("-v" "/tmp/.X11-unix:/tmp/.X11-unix")
    DOCKER_ARGS+=("-v" "$HOME/.Xauthority:$HOME/.Xauthority:rw")
fi

# ── GPU (NVIDIA) ──────────────────────────────────────────────────────────────
if [ ${#GPU_FLAGS[@]} -gt 0 ]; then
    DOCKER_ARGS+=("-e" "NVIDIA_VISIBLE_DEVICES=all")
    DOCKER_ARGS+=("-e" "NVIDIA_DRIVER_CAPABILITIES=all")
fi

# ── gz-sim environment ────────────────────────────────────────────────────────
# Plugin paths and resource paths are managed by gz_sim.launch.py via
# SetEnvironmentVariable. Only GZ_VERSION is set here unconditionally.
DOCKER_ARGS+=("-e" "GZ_VERSION=harmonic")
DOCKER_ARGS+=("-e" "TERM=${TERM:-xterm-256color}")

# ── Source code / install volume mounts ───────────────────────────────────────
DOCKER_ARGS+=("-v" "$ROOT:/ws")

# If a host-side install/ exists (produced by build_code.py / colcon build),
# overlay it into the container so the freshly-built plugin takes priority.
if [ -d "$ROOT/install" ]; then
    echo "-> Host install/ found – mounting freshly-built plugin."
    DOCKER_ARGS+=("-v" "$ROOT/install:/ws/install")
else
    echo "-> No host install/ found – using plugin baked into Docker image."
    echo "   Run 'Code Build' (option 2) to rebuild after source changes."
fi

# ── Plugin log files ───────────────────────────────────────────────────────────
# Mount file_logs/ read-write so the plugin can write session logs even though
# the rest of /ws is read-only.  Logs appear on the host at <project>/file_logs/.
mkdir -p "$ROOT/file_logs"
DOCKER_ARGS+=("-v" "$ROOT/file_logs:/ws/file_logs")
echo "-> Log output : $ROOT/file_logs/"

# ── ROS / Gazebo home directories ────────────────────────────────────────────
# ROS2 writes logs to $HOME/.ros/  and Gazebo to $HOME/.gz/
# Mount both so the container user (same UID) has write access.
mkdir -p "$HOME/.ros"
mkdir -p "$HOME/.gz"
DOCKER_ARGS+=("-v" "$HOME/.ros:$HOME/.ros")
DOCKER_ARGS+=("-v" "$HOME/.gz:$HOME/.gz")

# ── Jetson / aarch64 extras ───────────────────────────────────────────────────
if [ "$PLATFORM" = "aarch64" ]; then
    DOCKER_ARGS+=("-v" "/usr/bin/tegrastats:/usr/bin/tegrastats")
    DOCKER_ARGS+=("-v" "/usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
    DOCKER_ARGS+=("-v" "/usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
    DOCKER_ARGS+=("--pid=host")
    DOCKER_ARGS+=("-v" "/usr/share/vpi3:/usr/share/vpi3")
    DOCKER_ARGS+=("-v" "/dev/input:/dev/input")

    if [[ $(getent group jtop) ]]; then
        DOCKER_ARGS+=("-v" "/run/jtop.sock:/run/jtop.sock:ro")
        JETSON_STATS_GID="$(getent group jtop | cut -d: -f3)"
        DOCKER_ARGS+=("--group-add" "$JETSON_STATS_GID")
    fi
fi

# ── Load extra docker args from optional override file ────────────────────────
EXTRA_ARGS_FILE="$ROOT/.spray_paint_dockerargs"
if [ -f "$EXTRA_ARGS_FILE" ]; then
    echo "-> Loading extra docker args from $EXTRA_ARGS_FILE"
    readarray -t EXTRA_LINES < "$EXTRA_ARGS_FILE"
    for line in "${EXTRA_LINES[@]}"; do
        # shellcheck disable=SC2046
        DOCKER_ARGS+=($(eval "echo $line | envsubst"))
    done
fi

# ── GPU availability check ────────────────────────────────────────────────────
# Use --runtime nvidia (legacy mode) rather than --gpus all (CDI mode).
# --gpus all requires nvidia-container-toolkit CDI configuration; --runtime nvidia
# works with the classic daemon.json "runtimes" entry and is sufficient here.
GPU_FLAGS=()
if docker run --rm --runtime nvidia ubuntu:22.04 true 2>/dev/null; then
    echo "-> NVIDIA runtime available – enabling GPU passthrough."
    GPU_FLAGS=(--runtime nvidia)
else
    echo "-> NVIDIA runtime not available – running without GPU (CPU rendering)."
fi

# ── Common docker run flags ───────────────────────────────────────────────────
COMMON_FLAGS=(
    --privileged
    --network host
    "${GPU_FLAGS[@]}"
    --user "${HOST_UID}:${HOST_GID}"
    -e "HOME=$HOME"
    -e "USER=$HOST_USER"
    -v /dev:/dev
    -v /tmp:/tmp
    -v /etc/localtime:/etc/localtime:ro
    -v /etc/passwd:/etc/passwd:ro
    -v /etc/group:/etc/group:ro
    --name "$CONTAINER_NAME"
    --workdir /ws
)

# ── Run container ─────────────────────────────────────────────────────────────
echo "-> Starting container..."
echo ""

if [ "$MODE" = "empty_container" ]; then
    echo "-> Mode: empty_container (interactive bash, running as root)"
    docker run -it --rm \
        --privileged \
        --network host \
        "${GPU_FLAGS[@]}" \
        -e "HOME=/root" \
        -v /dev:/dev \
        -v /tmp:/tmp \
        -v /etc/localtime:/etc/localtime:ro \
        --name "$CONTAINER_NAME" \
        --workdir /ws \
        "${DOCKER_ARGS[@]}" \
        "$IMAGE_NAME" \
        bash
elif [ "$MODE" = "detach" ]; then
    echo "-> Mode: detach (container starts in background; exec into it to run workload)"
    docker run -d \
        "${COMMON_FLAGS[@]}" \
        "${DOCKER_ARGS[@]}" \
        "$IMAGE_NAME" \
        sleep infinity
    echo "-> Container '$CONTAINER_NAME' is running in the background."
else
    # Standalone: run the full demo stack directly (manual / CI use).
    if [ "$HEADLESS" = true ]; then
        LAUNCH_CMD="ros2 launch gz_spray_painting_plugin_demo ur_spray_demo.launch.py headless:=true"
    else
        LAUNCH_CMD="ros2 launch gz_spray_painting_plugin_demo ur_spray_demo.launch.py"
    fi
    echo "-> Launch: $LAUNCH_CMD"
    docker run -it --rm \
        "${COMMON_FLAGS[@]}" \
        "${DOCKER_ARGS[@]}" \
        "$IMAGE_NAME" \
        bash -c ". /opt/ros/humble/setup.bash && . /ws/install/setup.bash && $LAUNCH_CMD"
fi
