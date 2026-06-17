#!/usr/bin/env python3
"""
generate_spray_poses.py
=======================
Reads the current UR5e TCP pose from /joint_states, generates a linear
sweep to the RIGHT (robot base -Y direction), solves IK for each waypoint,
and writes joint_configs to:
  src/gz_spray_painting_plugin_demo/config/cartesian_poses.yaml

Usage (with robot running):
  ros2 run gz_spray_painting_plugin_demo generate_spray_poses.py
  ros2 run gz_spray_painting_plugin_demo generate_spray_poses.py --ros-args \
      -p sweep_distance:=0.7 -p n_waypoints:=10

Parameters:
  sweep_distance   Total rightward distance in metres (default: 0.7)
  n_waypoints      Number of waypoints including start and end (default: 8)
  output_file      Absolute path to output YAML (default: auto-detected)
"""

import os
import sys
import threading
import time

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# ─────────────────────────────────────────────────────────────────────────────
# UR5e kinematics  (standard DH convention, from default_kinematics.yaml)
# ─────────────────────────────────────────────────────────────────────────────

# Modified DH parameters derived from ur5e/default_kinematics.yaml:
#   shoulder:  z=0.1625               → d1
#   upper_arm: roll=π/2               → alpha1
#   forearm:   x=-0.425               → a2
#   wrist_1:   x=-0.3922, z=0.1333   → a3, d4
#   wrist_2:   y=-0.0997, roll=π/2   → d5, alpha4
#   wrist_3:   y=0.09959, roll=π/2   → d6, alpha5 (net: -π/2 after yaw=π)
D  = [0.1625,  0.0,    0.0,     0.1333, 0.0997,  0.0996]
A  = [0.0,    -0.425, -0.3922,  0.0,    0.0,     0.0   ]
AL = [np.pi/2, 0.0,   0.0,     np.pi/2, -np.pi/2, 0.0  ]

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _dh(d, a, alpha, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def fk(q):
    """Forward kinematics: joint angles → 4×4 base→tool0 transform."""
    T = np.eye(4)
    for i in range(6):
        T = T @ _dh(D[i], A[i], AL[i], q[i])
    return T


def _pose_error(q, target_pos, target_rot):
    """IK cost: squared position + rotation error."""
    T = fk(q)
    dp = T[:3, 3] - target_pos
    dR = T[:3, :3] - target_rot
    return float(np.dot(dp, dp) * 100.0 + np.sum(dR ** 2))


def ik(target_pos, target_rot, q0, tol=1e-8, maxiter=2000):
    """
    Numerical IK via L-BFGS-B.
    Returns (q, success) where q is the joint solution.
    """
    # Joint limits for UR5e (generous ±2π for wrists)
    bounds = [
        (-2 * np.pi, 2 * np.pi),
        (-2 * np.pi, 2 * np.pi),
        (-np.pi,     np.pi),
        (-2 * np.pi, 2 * np.pi),
        (-2 * np.pi, 2 * np.pi),
        (-2 * np.pi, 2 * np.pi),
    ]
    res = minimize(
        _pose_error,
        q0,
        args=(target_pos, target_rot),
        method="L-BFGS-B",
        bounds=bounds,
        options={"maxiter": maxiter, "ftol": tol, "gtol": 1e-7},
    )
    T = fk(res.x)
    pos_err = np.linalg.norm(T[:3, 3] - target_pos)
    return res.x, pos_err < 2e-3  # success if < 2 mm


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 node
# ─────────────────────────────────────────────────────────────────────────────

class PoseGenerator(Node):

    def __init__(self):
        super().__init__("generate_spray_poses")

        self.declare_parameter("sweep_distance", 0.7)
        self.declare_parameter("n_waypoints",    8)
        self.declare_parameter("output_file",    "")

        self._joints: dict | None = None
        self._lock = threading.Lock()

        self._sub = self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10
        )

    def _js_cb(self, msg: JointState):
        try:
            q = {n: msg.position[msg.name.index(n)] for n in JOINT_NAMES}
            with self._lock:
                self._joints = q
        except (ValueError, IndexError):
            pass

    def wait_for_joints(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._joints is not None:
                    return dict(self._joints)
            time.sleep(0.05)
        return None

    def run(self):
        sweep_dist = self.get_parameter("sweep_distance").value
        n_wpts     = self.get_parameter("n_waypoints").value
        out_file   = self.get_parameter("output_file").value

        if not out_file:
            # Auto-detect: climb from this script's location to the config dir
            script_dir = os.path.dirname(os.path.abspath(__file__))
            out_file = os.path.join(
                script_dir, "..", "config", "cartesian_poses.yaml"
            )
        out_file = os.path.realpath(out_file)

        self.get_logger().info("Waiting for /joint_states …")
        joints = self.wait_for_joints()
        if joints is None:
            self.get_logger().error("No joint state received within 15 s – abort")
            return False

        q0 = np.array([joints[n] for n in JOINT_NAMES])
        T0 = fk(q0)

        tcp_pos = T0[:3, 3].copy()
        tcp_rot = T0[:3, :3].copy()
        quat    = Rotation.from_matrix(tcp_rot).as_quat()  # [qx, qy, qz, qw]

        self.get_logger().info(
            f"Current TCP  pos=({tcp_pos[0]:.4f}, {tcp_pos[1]:.4f}, {tcp_pos[2]:.4f})"
            f"  quat=({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})"
        )
        self.get_logger().info(
            f"Sweep: {n_wpts} waypoints, {sweep_dist:.3f} m to the RIGHT "
            f"(robot base –Y direction)"
        )

        # Linear waypoints: x and z fixed, y decreases (robot base -Y = operator right)
        y_start = tcp_pos[1]
        y_end   = tcp_pos[1] - sweep_dist

        waypoints = []
        for i in range(n_wpts):
            t   = i / (n_wpts - 1)
            pos = tcp_pos.copy()
            pos[1] = y_start + t * (y_end - y_start)
            waypoints.append(pos)

        # Solve IK for each waypoint
        joint_configs = []
        q_seed = q0.copy()
        all_ok = True

        for i, pos in enumerate(waypoints):
            q_sol, ok = ik(pos, tcp_rot, q_seed)
            if not ok:
                T_check = fk(q_sol)
                err = np.linalg.norm(T_check[:3, 3] - pos)
                self.get_logger().warn(
                    f"  WP{i}: IK converged with pos_err={err*1000:.1f} mm"
                )
                all_ok = False

            # Verify FK
            T_check = fk(q_sol)
            err_mm = np.linalg.norm(T_check[:3, 3] - pos) * 1000.0
            self.get_logger().info(
                f"  WP{i}: y={pos[1]:.4f}  joints=[{', '.join(f'{v:.5f}' for v in q_sol)}]"
                f"  pos_err={err_mm:.2f} mm"
            )

            joint_configs.append({
                "name": f"wp{i}",
                **{JOINT_NAMES[j]: float(q_sol[j]) for j in range(6)},
            })
            q_seed = q_sol.copy()

        # Build YAML document
        doc = {
            "poses": [
                {
                    "name": f"wp{i}",
                    "position": {
                        "x": float(waypoints[i][0]),
                        "y": float(waypoints[i][1]),
                        "z": float(waypoints[i][2]),
                    },
                    "orientation": {
                        "x": float(quat[0]),
                        "y": float(quat[1]),
                        "z": float(quat[2]),
                        "w": float(quat[3]),
                    },
                }
                for i in range(n_wpts)
            ],
            "joint_configs": joint_configs,
        }

        with open(out_file, "w") as f:
            yaml.dump(doc, f, default_flow_style=False, sort_keys=False)

        status = "OK" if all_ok else "WARN – some IK errors > 2 mm"
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"  Written {n_wpts} waypoints → {out_file}\n"
            f"  TCP orientation preserved throughout\n"
            f"  Y: {y_start:.4f} → {y_end:.4f}  (Δ={sweep_dist:.3f} m)\n"
            f"  Status: {status}\n"
            f"{'='*60}"
        )
        return all_ok


def main():
    rclpy.init(args=sys.argv)
    node = PoseGenerator()

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        ok = node.run()
    finally:
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
