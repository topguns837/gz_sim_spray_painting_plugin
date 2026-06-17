#!/usr/bin/env python3
"""
cartesian_path_executor.py
==========================
Drives the UR5e spray gun through a boustrophedon (zigzag) pattern using
pre-computed joint configurations loaded from a YAML file.

Each waypoint in the YAML has a `joint_configs` section with the 6 joint
angles that put tool0 at the desired TCP pose (position + orientation).  The
script moves through the waypoints by publishing directly to the
joint_trajectory_controller, bypassing MoveIt's planner and collision
checker (MoveIt's world frame incorrectly places the ground at the robot
base level rather than 0.80 m below, causing false self-collision rejects).

Spray pattern:
  home  →  row1_start (spray OFF, approach)
         →  row1_end  (spray ON, continuous sweep, no stops)
  spray OFF  →  home (return)

Parameters (--ros-args -p):
  poses_file        YAML file with joint_configs section (required)
  planning_group    unused (kept for compatibility)
  velocity_scaling  0–1, used to scale move duration (default: 0.35)
  spray_topic       Topic to trigger spray (default: /spray_paint/trigger)
  spray_enabled     Publish spray trigger (default: true)

Usage:
  ros2 launch gz_spray_painting_plugin_demo cartesian_spray.launch.py
"""

import subprocess
import sys
import time
import threading
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

HOME = {
    "shoulder_pan_joint":  0.0,
    "shoulder_lift_joint": -1.5708,
    "elbow_joint":          1.5708,
    "wrist_1_joint":       -1.5708,
    "wrist_2_joint":       -1.5708,
    "wrist_3_joint":        0.0,
}

MAX_JOINT_SPEED = 1.0  # rad/s


def _move_duration(q_from: list, q_to: list, velocity_scaling: float) -> float:
    """Return the time (s) to move from q_from to q_to at scaled speed."""
    max_delta = max(abs(a - b) for a, b in zip(q_from, q_to))
    if max_delta < 1e-6:
        return 0.5
    speed = MAX_JOINT_SPEED * max(0.05, min(1.0, velocity_scaling))
    return max(1.5, max_delta / speed)


class JointSprayPainter(Node):

    def __init__(self):
        super().__init__("cartesian_path_executor")

        self.declare_parameter("poses_file",       "")
        self.declare_parameter("planning_group",   "ur_manipulator")
        self.declare_parameter("velocity_scaling", 0.35)
        self.declare_parameter("spray_topic",      "/spray_paint/trigger")
        self.declare_parameter("spray_enabled",    True)

        self._traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )
        self._current_joints: list[float] | None = None
        self._js_sub = self.create_subscription(
            JointState, "/joint_states",
            self._js_cb, 10,
        )

    def _js_cb(self, msg: JointState):
        try:
            pos = [msg.position[msg.name.index(j)] for j in JOINT_NAMES]
            self._current_joints = pos
        except (ValueError, IndexError):
            pass

    def _wait_for_joints(self, timeout: float = 15.0) -> list[float] | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._current_joints is not None:
                return list(self._current_joints)
            time.sleep(0.05)
        return None

    def _wait_for_controller(self, timeout: float = 30.0) -> bool:
        self.get_logger().info("Waiting for joint_trajectory_controller...")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._traj_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.5)
        self.get_logger().error("joint_trajectory_controller not ready after timeout")
        return False

    def _send_trajectory(self, q_list: list[list[float]], vel_scale: float,
                         label: str, zero_end_vel: bool = True) -> bool:
        """
        Send a multi-point JointTrajectory, wait for completion.

        Intermediate velocities are computed by finite differences so the
        controller does not decelerate to zero between waypoints, giving
        smooth continuous motion.  Only the first and last points get
        zero velocity.
        """
        n = len(q_list)
        if n == 0:
            return True

        # Cumulative times
        times = [0.0]
        for i in range(1, n):
            times.append(times[-1] + _move_duration(q_list[i - 1], q_list[i], vel_scale))

        # Finite-difference velocities; zero at endpoints
        vels: list[list[float]] = [[0.0] * 6]
        for i in range(1, n - 1):
            dt = times[i + 1] - times[i - 1]
            v = [(q_list[i + 1][j] - q_list[i - 1][j]) / dt for j in range(6)]
            vels.append(v)
        vels.append([0.0] * 6 if zero_end_vel else vels[-1])

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        for q, v, t in zip(q_list, vels, times):
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.velocities = v
            pt.accelerations = [0.0] * 6
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            msg.points.append(pt)

        self._traj_pub.publish(msg)
        total = times[-1]
        self.get_logger().info(f"  {label}: {n} pts, {total:.1f}s — waiting...")
        time.sleep(total + 0.5)
        return True

    def _move_to(self, target: dict, vel_scale: float) -> bool:
        """Single-waypoint move (used for home and approach)."""
        current = self._wait_for_joints(timeout=10.0)
        if current is None:
            self.get_logger().warn("No joint state received — sending move anyway")
            current = [target[j] for j in JOINT_NAMES]
        q_to = [target[j] for j in JOINT_NAMES]
        return self._send_trajectory([current, q_to], vel_scale, "move")

    # ── public entry point ─────────────────────────────────────────────────────

    def run(self) -> bool:
        poses_file    = self.get_parameter("poses_file").value
        vel_scale     = self.get_parameter("velocity_scaling").value
        spray_topic   = self.get_parameter("spray_topic").value
        spray_enabled = self.get_parameter("spray_enabled").value

        if not poses_file:
            self.get_logger().error("poses_file parameter is required.")
            return False

        try:
            with open(poses_file, "r") as f:
                data = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Poses file not found: {poses_file}")
            return False

        joint_configs: list[dict] = []
        for entry in data.get("joint_configs", []):
            cfg = {jn: float(entry[jn]) for jn in JOINT_NAMES}
            name = entry.get("name", f"wp{len(joint_configs)}")
            self.get_logger().info(
                f"  Loaded {name}: pan={cfg['shoulder_pan_joint']:.3f} "
                f"lift={cfg['shoulder_lift_joint']:.3f} "
                f"elbow={cfg['elbow_joint']:.3f}"
            )
            joint_configs.append(cfg)

        if len(joint_configs) < 2:
            self.get_logger().error("Need at least 2 joint configs in YAML.")
            return False

        self.get_logger().info(f"Loaded {len(joint_configs)} joint configs.")

        def set_spray(state: bool):
            if not spray_enabled:
                return
            data_str = "data: true" if state else "data: false"
            try:
                subprocess.run(
                    [
                        "gz", "topic",
                        "-t", spray_topic,
                        "-m", "gz.msgs.Boolean",
                        "-p", data_str,
                    ],
                    check=True, timeout=5.0,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                )
            except Exception as exc:
                self.get_logger().warn(f"gz topic spray command failed: {exc}")
            self.get_logger().info(f"Spray {'ON' if state else 'OFF'}")

        if not self._wait_for_controller(timeout=60.0):
            return False
        time.sleep(1.0)

        self.get_logger().info("Moving to home configuration...")
        self._move_to(HOME, vel_scale)
        time.sleep(1.0)

        self.get_logger().info("Approaching start position (spray OFF)...")
        self._move_to(joint_configs[0], vel_scale)
        time.sleep(0.5)

        # Continuous sweep: all waypoints in a single trajectory so the
        # controller never decelerates to zero between intermediate points.
        set_spray(True)
        q_sweep = [[cfg[j] for j in JOINT_NAMES] for cfg in joint_configs]
        self.get_logger().info(
            f"Starting continuous sweep ({len(q_sweep)} waypoints)..."
        )
        self._send_trajectory(q_sweep, vel_scale, "sweep")
        set_spray(False)

        self.get_logger().info("Returning to home configuration...")
        self._move_to(HOME, vel_scale)

        self.get_logger().info("Spray painting complete.")
        return True


def main():
    rclpy.init(args=sys.argv)
    node = JointSprayPainter()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()
