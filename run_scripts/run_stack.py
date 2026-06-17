#!/usr/bin/env python3
"""
run_stack.py
────────────
Runs INSIDE the Docker container. Presents a world-selection menu built by
scanning the installed worlds directory, then creates the appropriate tmux
session and attaches.

World mode is determined solely by filename:
  car_painting.sdf  →  UR mode  (UR5e + MoveIt + cartesian spray)
  everything else   →  demo mode (Gazebo + spray nozzle only)

Adding or removing .sdf files from the worlds directory automatically updates
the menu — no code changes required.

tmux layouts
────────────
  UR mode:
    Window 0 – sim            : ur_spray_demo.launch.py (Gazebo + UR5e + MoveIt)
    Window 1 – cartesian_spray: cartesian_spray.launch.py (auto-runs after 20 s)
    Window 2 – spray_control  : spray ON/OFF pre-typed

  Demo mode:
    Window 0 – sim            : demo.launch.py world:=<stem>
    Window 1 – spray_control  : spray ON/OFF pre-typed
"""

import os
import subprocess
import sys

SESSION   = "spray_paint"
TMUX_CONF = "/tmp/.tmux_spray.conf"
ROS       = ". /opt/ros/humble/setup.bash && . /ws/install/setup.bash 2>/dev/null || true"

UR_WORLD  = "demo_car"

# ── Discover worlds dynamically ───────────────────────────────────────────────
# Prefer installed share directory; fall back to source tree.
_INSTALLED = "/ws/install/gz_spray_painting_plugin_demo/share/gz_spray_painting_plugin_demo/worlds"
_SOURCE    = "/ws/src/gz_spray_painting_plugin_demo/worlds"
WORLDS_DIR = _INSTALLED if os.path.isdir(_INSTALLED) else _SOURCE

if not os.path.isdir(WORLDS_DIR):
    print(f"  Error: worlds directory not found at:\n  {_INSTALLED}\n  {_SOURCE}")
    sys.exit(1)

def _label(stem: str) -> str:
    suffix = " (UR5e robot)" if stem == UR_WORLD else ""
    return stem.replace("_", " ").title() + suffix

# Build menu: car_painting first (if present), rest alphabetically.
_stems = sorted(
    os.path.splitext(f)[0]
    for f in os.listdir(WORLDS_DIR)
    if f.endswith(".sdf")
)
if UR_WORLD in _stems:
    _stems = [UR_WORLD] + [s for s in _stems if s != UR_WORLD]

WORLDS = [(_label(s), s, "ur" if s == UR_WORLD else "demo") for s in _stems]

if not WORLDS:
    print("  Error: no .sdf worlds found in", WORLDS_DIR)
    sys.exit(1)

# ── World selection menu ───────────────────────────────────────────────────────
print("\n  ╔══════════════════════════════════════════════════╗")
print("  ║          GZ SIM – SELECT A WORLD                 ║")
print("  ╚══════════════════════════════════════════════════╝\n")
for i, (label, _, _) in enumerate(WORLDS, 1):
    print(f"  [{i}] {label}")
print()

try:
    raw = input("  Choice: ").strip()
    idx = int(raw) - 1
    if not (0 <= idx < len(WORLDS)):
        raise ValueError
except (ValueError, EOFError):
    print("  Invalid choice — exiting.")
    sys.exit(1)

label, world_stem, mode = WORLDS[idx]
print(f"\n  Launching: {label}\n")

# ── tmux config: arrow keys, true-colour, bash as default shell ───────────────
with open(TMUX_CONF, "w") as f:
    f.write('set -g default-terminal "screen-256color"\n')
    f.write('set -g terminal-overrides ",xterm-256color:Tc"\n')
    f.write('set-option -g default-shell /bin/bash\n')


def tmux(*args):
    subprocess.run(["tmux", "-f", TMUX_CONF, *args], check=True)


def send(target, cmd, enter=True):
    tmux("send-keys", "-t", f"{SESSION}:{target}", cmd, *(["Enter"] if enter else [""]))


# ── Build tmux session ────────────────────────────────────────────────────────
if mode == "ur":
    # ── Window 0: sim (full UR5e stack) ──────────────────────────────────────
    tmux("new-session", "-d", "-s", SESSION, "-n", "sim", "-x", "220", "-y", "50")
    send("sim.0", f"{ROS} && ros2 launch gz_spray_painting_plugin_demo ur_spray_demo.launch.py")

    # ── Window 1: cartesian_spray ─────────────────────────────────────────────
    tmux("new-window", "-t", SESSION, "-n", "cartesian_spray")
    send("cartesian_spray.0",
         f"sleep 20 && {ROS} && ros2 launch gz_spray_painting_plugin_demo cartesian_spray.launch.py")

    # ── Window 2: spray_control ───────────────────────────────────────────────
    tmux("new-window", "-t", SESSION, "-n", "spray_control")
    send("spray_control.0",
         'gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: true"', enter=False)
    tmux("split-window", "-t", f"{SESSION}:spray_control", "-v", "-p", "50")
    send("spray_control.1",
         'gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: false"', enter=False)
    tmux("select-pane", "-t", f"{SESSION}:spray_control.0")

else:
    # ── Window 0: sim (nozzle-only demo) ─────────────────────────────────────
    tmux("new-session", "-d", "-s", SESSION, "-n", "sim", "-x", "220", "-y", "50")
    send("sim.0",
         f"{ROS} && ros2 launch gz_spray_painting_plugin_demo demo.launch.py world:={world_stem}")

    # ── Window 1: spray_control ───────────────────────────────────────────────
    tmux("new-window", "-t", SESSION, "-n", "spray_control")
    send("spray_control.0",
         'gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: true"', enter=False)
    tmux("split-window", "-t", f"{SESSION}:spray_control", "-v", "-p", "50")
    send("spray_control.1",
         'gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: false"', enter=False)
    tmux("select-pane", "-t", f"{SESSION}:spray_control.0")

# ── Focus sim window and attach ───────────────────────────────────────────────
tmux("select-window", "-t", f"{SESSION}:sim")
os.execvp("tmux", ["tmux", "-f", TMUX_CONF, "attach-session", "-t", SESSION])
