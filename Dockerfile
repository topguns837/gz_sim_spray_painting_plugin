# ─────────────────────────────────────────────────────────────────────────────
# Spray Paint Plugin – Gazebo Harmonic (gz-sim 8) + ROS 2 Humble
# Base: Ubuntu 22.04 (Jammy)
#
# Build image:
#   docker build -t spray_paint_plugin .
#
# Run (headless, trigger via gz topic):
#   docker run --rm -it spray_paint_plugin gz sim -s $SPRAY_WORLD
#
# Run with GUI (requires X11 forwarding on host: xhost +local:docker):
#   docker run --rm -it \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     spray_paint_plugin gz sim $SPRAY_WORLD
# ─────────────────────────────────────────────────────────────────────────────

FROM ubuntu:22.04

# Disable apt release-date validity check (guards against host/container clock skew)
RUN echo 'Acquire::Check-Valid-Until "false";' > /etc/apt/apt.conf.d/99ignore-release-date

# ── Non-interactive apt ────────────────────────────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# ── Locale (required by ROS 2) ─────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ── Base utilities ─────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl \
      gnupg2 \
      lsb-release \
      ca-certificates \
      software-properties-common \
      wget \
      git \
      build-essential \
      cmake \
      python3-pip \
      tmux \
    && rm -rf /var/lib/apt/lists/*

# ── Add ROS 2 Humble apt repository ───────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# ── Add Gazebo Harmonic (OSRF) apt repository ─────────────────────────────────
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# ── Install Gazebo Harmonic runtime + dev libraries ────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      # Runtime executables
      gz-harmonic \
      # Core dev headers needed to build the plugin
      libgz-sim8-dev \
      libgz-rendering8-dev \
      libgz-rendering8-ogre2-dev \
      libgz-transport13-dev \
      libgz-math7-dev \
      libgz-common5-dev \
      libgz-common5-graphics-dev \
      libgz-plugin2-dev \
      libgz-msgs10-dev \
      libgz-cmake3-dev \
    && rm -rf /var/lib/apt/lists/*

# ── Install ROS 2 Humble (ament_cmake + colcon + ros_gz bridge) ────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros-base \
      ros-humble-ament-cmake \
      ros-humble-robot-state-publisher \
      ros-humble-ros-gz-sim \
      ros-humble-ros-gz-bridge \
      ros-humble-xacro \
      python3-colcon-common-extensions \
      python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ── ros2_control (controller_manager, spawner, JTC, joint_state_broadcaster) ──
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-ros2-control-cmake \
    && rm -rf /var/lib/apt/lists/*

# ── UR robot description + MoveIt ─────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-moveit \
      ros-humble-ur-moveit-config \
    && rm -rf /var/lib/apt/lists/*

# gz_spray_painting_plugin_demo and gz_ros2_control are cloned into src/ and built by the
# main colcon step below (after COPY . .).

# ── rosdep init (best-effort; ignore if already done) ─────────────────────────
RUN rosdep init 2>/dev/null || true && rosdep update 2>/dev/null || true

# ── Refresh GPG keys (guards against stale Docker cache invalidating signatures)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# ── X11 / display libraries for gz-sim GUI ────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgl1-mesa-glx \
      libgl1-mesa-dri \
      libgles2-mesa \
      libx11-xcb1 \
      libxcb-icccm4 \
      libxcb-image0 \
      libxcb-keysyms1 \
      libxcb-render-util0 \
      libxcb-xinerama0 \
      libxkbcommon-x11-0 \
      libfontconfig1 \
    && rm -rf /var/lib/apt/lists/*

# ── Set up colcon workspace ───────────────────────────────────────────────────
# The project root IS the colcon workspace; the package lives in src/.
WORKDIR /ws
COPY . .

# ── Build with colcon ─────────────────────────────────────────────────────────
# GZ_VERSION=harmonic is required so gz_ros2_control links against gz-sim8.
RUN . /opt/ros/humble/setup.sh \
    && GZ_VERSION=harmonic colcon build --symlink-install \
         --packages-select \
           gz_ros2_control \
           gz_spray_painting_plugin_demo \
           gz_sim_spray_painting_plugin \
         --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# ── Runtime environment ───────────────────────────────────────────────────────
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/ws/install/gz_sim_spray_painting_plugin/lib/gz_sim_spray_painting_plugin:/ws/install/gz_ros2_control/lib
ENV GZ_SIM_RESOURCE_PATH=/ws/install/gz_spray_painting_plugin_demo/share
ENV GZ_VERSION=harmonic
# Source ROS and the colcon workspace in every shell session
RUN echo ". /opt/ros/humble/setup.bash" >> /etc/bash.bashrc \
    && echo ". /ws/install/setup.bash 2>/dev/null || true" >> /etc/bash.bashrc

# Convenience: world file path
ENV SPRAY_WORLD=/ws/install/gz_spray_painting_plugin_demo/share/gz_spray_painting_plugin_demo/worlds/spray_painting.sdf

# ── Entrypoint ────────────────────────────────────────────────────────────────
# entrypoint.sh was already copied via COPY . . above (project root → /ws)
RUN chmod +x /ws/docker/entrypoint.sh \
    && cp /ws/docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default: launch gz sim server only (headless). Use shell form so $SPRAY_WORLD expands.
CMD gz sim -s $SPRAY_WORLD
