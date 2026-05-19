#!/usr/bin/env python3
"""
run_stack.py
────────────
Launches the full spray paint simulation stack inside a single Docker container.

The tmux session is created INSIDE the container by container_tmux_setup.sh.
Killing the tmux session (or closing all windows) stops the container.

tmux layout (inside the container)
───────────────────────────────────
  Window 0 – sim
      pane 0 : ur_spray_demo.launch.py – Gazebo + robot spawn + MoveIt (all-in-one)
  Window 1 – cartesian_spray
      pane 0 : cartesian_spray.launch.py (pre-typed, press Enter to run)
      pane 1 : ros2 topic echo /joint_states
  Window 2 – spray_control
      pane 0 : spray ON  command (press Enter to fire)
      pane 1 : spray OFF command (press Enter to fire)
"""

import os
import shutil
import subprocess
import sys

# ── Config ────────────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
RUN_DOCKER  = os.path.join(SCRIPT_DIR, "run_docker.sh")
IMAGE_NAME  = "spray_paint_plugin"
CONTAINER   = "spray_paint_stack"

# ── Colours ───────────────────────────────────────────────────────────────────
R = "\033[0;31m"; G = "\033[0;32m"; Y = "\033[1;33m"
C = "\033[0;36m"; B = "\033[1m";    X = "\033[0m"


def check_dependency(name: str):
    if shutil.which(name) is None:
        print(f"  {R}Error:{X} '{name}' is not installed or not in PATH.")
        sys.exit(1)


def check_docker_image() -> bool:
    return subprocess.run(
        ["docker", "image", "inspect", IMAGE_NAME],
        capture_output=True
    ).returncode == 0


def kill_existing():
    subprocess.run(["docker", "stop", CONTAINER], capture_output=True)
    subprocess.run(["docker", "rm",   CONTAINER], capture_output=True)


def launch():
    print(f"\n{G}{B}▶ Launching spray paint stack{X}\n")

    # ── Pre-flight ────────────────────────────────────────────────────────────
    check_dependency("docker")

    if not os.path.isfile(RUN_DOCKER):
        print(f"  {R}Error:{X} run_docker.sh not found at {RUN_DOCKER}")
        sys.exit(1)

    if not check_docker_image():
        print(f"  {R}Error:{X} Docker image '{IMAGE_NAME}' not found.")
        print(f"  Run Docker Build (option 3 in the start menu) first.\n")
        sys.exit(1)

    host_install = os.path.join(PROJECT_DIR, "install")
    if os.path.isdir(host_install):
        print(f"  {G}Host install/ found:{X} freshly-built plugin will be used.")
    else:
        print(f"  {Y}No host install/ found:{X} plugin baked into image will be used.")
        print(f"  Run Code Build (option 2) to rebuild after source changes.\n")

    # ── Clean up any stale container ──────────────────────────────────────────
    kill_existing()

    print(f"  Container : {C}{CONTAINER}{X}")
    print(f"  Image     : {C}{IMAGE_NAME}{X}")
    print(f"\n  {Y}Tips (inside tmux):{X} Ctrl-b n → next window  |  Ctrl-b d → detach")
    print(f"  Killing the tmux session stops the container.\n")

    # ── Hand off to run_docker.sh which starts the container and tmux ─────────
    os.execvp("bash", ["bash", RUN_DOCKER, f"container_name={CONTAINER}", "tmux_stack"])


if __name__ == "__main__":
    launch()
