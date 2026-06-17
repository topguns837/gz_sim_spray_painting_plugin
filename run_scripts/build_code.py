#!/usr/bin/env python3
"""
build_code.py
─────────────
Starts the Docker container and runs colcon build inside it.

The project source is bind-mounted into the container so you can edit code on
the host and rebuild without re-building the Docker image.  The colcon output
(install/, build/, log/) is written back to the host project root via mounts.
"""

import os
import subprocess
import sys

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)   # repo root
IMAGE_NAME  = "spray_paint_plugin"
CONTAINER   = "spray_paint_build"

# Directories that colcon writes to – created on the host if they don't exist.
COLCON_DIRS = ["install", "build", "log"]

# ── Colours ───────────────────────────────────────────────────────────────────
R = "\033[0;31m"; G = "\033[0;32m"; Y = "\033[1;33m"
C = "\033[0;36m"; B = "\033[1m";    X = "\033[0m"


def check_docker_image() -> bool:
    result = subprocess.run(
        ["docker", "image", "inspect", IMAGE_NAME],
        capture_output=True
    )
    return result.returncode == 0


def ensure_colcon_dirs():
    """Create host-side colcon output directories so Docker can bind-mount them."""
    for d in COLCON_DIRS:
        path = os.path.join(PROJECT_DIR, d)
        os.makedirs(path, exist_ok=True)


def build():
    print(f"\n{Y}{B}▶ Code Build – running colcon inside Docker container{X}\n")

    # ── Pre-flight ────────────────────────────────────────────────────────────
    if not check_docker_image():
        print(f"  {R}Error:{X} Docker image '{IMAGE_NAME}' not found.")
        print(f"  Run Docker Build (option 3 in the start menu) first.\n")
        sys.exit(1)

    ensure_colcon_dirs()

    # ── Build the docker run command ──────────────────────────────────────────
    # Source is mounted read-only; output dirs are mounted read-write.
    mounts = [
        # project root → colcon workspace root (matches Dockerfile WORKDIR /ws)
        "-v", f"{PROJECT_DIR}:/ws:ro",
        # colcon output dirs (read-write so artifacts appear on the host)
        "-v", f"{PROJECT_DIR}/install:/ws/install",
        "-v", f"{PROJECT_DIR}/build:/ws/build",
        "-v", f"{PROJECT_DIR}/log:/ws/log",
    ]

    colcon_cmd = (
        ". /opt/ros/humble/setup.bash && "
        "cd /ws && "
        "GZ_VERSION=harmonic colcon build --symlink-install "
        "  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo "
        "  --event-handlers console_cohesion+"
    )

    cmd = [
        "docker", "run", "--rm",
        "--name", CONTAINER,
        *mounts,
        IMAGE_NAME,
        "bash", "-c", colcon_cmd,
    ]

    print(f"  Container : {C}{IMAGE_NAME}{X}")
    print(f"  Source    : {C}{PROJECT_DIR}{X}")
    print(f"  Output    : {C}{PROJECT_DIR}/install/{X}\n")
    print("─" * 60)

    # ── Run ───────────────────────────────────────────────────────────────────
    result = subprocess.run(cmd)

    print("─" * 60)

    if result.returncode == 0:
        plugin_lib = os.path.join(
            PROJECT_DIR,
            "install",
            "gz_sim_spray_painting_plugin",
            "lib",
            "gz_sim_spray_painting_plugin",
        )
        print(f"\n{G}{B}✔ Build complete.{X}")
        print(f"  Plugin library: {C}{plugin_lib}/{X}\n")
    else:
        print(f"\n{R}{B}✘ Build failed (exit code {result.returncode}).{X}\n")
        sys.exit(result.returncode)


if __name__ == "__main__":
    build()
