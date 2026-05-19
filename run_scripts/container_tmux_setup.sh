#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# container_tmux_setup.sh
# Runs INSIDE the Docker container. Creates the spray paint tmux session,
# populates all windows/panes, then attaches. The container lives exactly as
# long as this tmux session – killing the session stops the container.
# ─────────────────────────────────────────────────────────────────────────────

SESSION="spray_paint"

# Source ROS + colcon install so ros2 launch works in every pane.
. /opt/ros/humble/setup.bash
. /ws/install/setup.bash

# ── Window 0: sim ────────────────────────────────────────────────────────────
# ur_spray_demo.launch.py (now in ur_simulation_gz) starts Gazebo, spawns the
# robot, brings up MoveIt, and bridges /clock and /spray_paint/trigger.
tmux new-session -d -s "$SESSION" -n sim -x 220 -y 50

tmux send-keys -t "$SESSION:sim.0" \
    "ros2 launch ur_simulation_gz ur_spray_demo.launch.py" Enter

# ── Window 1: cartesian_spray ─────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n cartesian_spray

# Top pane: auto-execute after 70s delay — gives the stack enough time to
# fully start (Gazebo + robot spawn + MoveIt) before the executor connects.
tmux send-keys -t "$SESSION:cartesian_spray.0" \
    "sleep 20 && ros2 launch ur_simulation_gz cartesian_spray.launch.py" Enter

# Bottom pane: log monitor (split horizontally)
tmux split-window -t "$SESSION:cartesian_spray" -h
tmux send-keys -t "$SESSION:cartesian_spray.1" \
    "ros2 topic echo /joint_states --once" Enter

tmux select-pane -t "$SESSION:cartesian_spray.0"

# ── Window 2: spray_control ──────────────────────────────────────────────────
tmux new-window -t "$SESSION" -n spray_control

SPRAY_ON='gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: true"'
SPRAY_OFF='gz topic -t /spray_paint/trigger -m gz.msgs.Boolean -p "data: false"'

tmux send-keys -t "$SESSION:spray_control" \
    "echo '─────────────────────────────────────────────────────'" Enter
tmux send-keys -t "$SESSION:spray_control" \
    "echo '  Spray Paint Control – press Enter on a command to run'" Enter
tmux send-keys -t "$SESSION:spray_control" \
    "echo '─────────────────────────────────────────────────────'" Enter

# Top pane: spray ON pre-typed, not executed.
tmux send-keys -t "$SESSION:spray_control.0" "$SPRAY_ON"

# Bottom pane: spray OFF pre-typed, not executed.
tmux split-window -t "$SESSION:spray_control" -v -l 3
tmux send-keys -t "$SESSION:spray_control.1" "$SPRAY_OFF"

tmux select-pane -t "$SESSION:spray_control.0"

# ── Focus gz_sim and attach ───────────────────────────────────────────────────
tmux select-window -t "$SESSION:sim"

# Attach – keeps the container alive. When this session ends the container stops.
tmux attach-session -t "$SESSION"
