#!/usr/bin/env python3
"""
spray_painting_demo.py
======================
Autonomous raster-scan spray painting demo for the UR5e in Gazebo Harmonic.

Workflow
--------
1. Move arm to a safe home pose (joints).
2. Move end-effector to the start of row 0 (Cartesian goal).
3. Enable spray → execute Cartesian sweep across the panel.
4. Disable spray → reposition to next row start.
5. Repeat for all rows, alternating sweep direction (boustrophedon).

Panel geometry (must match ur_spray_painting.sdf)
--------------------------------------------------
  Centre : x=0.75 m, y=0.0 m, z=0.4 m
  Width  : 1.0 m  (y from -0.5 to +0.5)
  Height : 0.8 m  (z from 0.0 to 0.8)
  Depth  : 0.05 m (robot-facing face at x=0.725)

Nozzle stand-off: 0.25 m from panel face → nozzle x = 0.725 - 0.25 = 0.475 m
Row spacing: 0.15 m (3 rows: z = 0.65, 0.50, 0.35)
Sweep y: -0.40 → +0.40 m (within panel width, 0.10 m margin)

Usage
-----
  # In a separate terminal (after the demo launch is up and stable):
  ros2 run gz_sim_spray_painting_plugin spray_painting_demo.py
"""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


# ── Demo constants ────────────────────────────────────────────────────────────

PLANNING_GROUP = "ur_manipulator"

# Safe home pose (joints, radians) — arm pointing up and slightly back
HOME_JOINTS = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

# Panel painting parameters
NOZZLE_X   = 0.475   # stand-off from panel face
SWEEP_Y    = [-0.40, 0.40]   # y sweep limits
ROW_Z      = [0.65, 0.50, 0.35]   # z height of each row

# End-effector orientation: pointing in +X (toward panel)
# In world frame: tool pointing +X means RPY = (0, π/2, 0) → quaternion
import math
_half = math.pi / 4.0
NOZZLE_QUAT = Quaternion(x=0.0, y=math.sin(math.pi / 4), z=0.0, w=math.cos(math.pi / 4))

CART_VELOCITY_SCALE = 0.25   # slow for even paint coverage
CART_ACCEL_SCALE    = 0.25
MOVE_VELOCITY_SCALE = 0.5    # repositioning speed


# ── Helper ────────────────────────────────────────────────────────────────────

def make_pose(x: float, y: float, z: float) -> Pose:
    """Build a Pose with nozzle pointing +X."""
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    p.orientation = NOZZLE_QUAT
    return p


# ── Demo node ─────────────────────────────────────────────────────────────────

class SprayPaintingDemo(Node):

    def __init__(self):
        super().__init__("spray_painting_demo")

        self._spray_pub = self.create_publisher(Bool, "/spray_paint/trigger", 10)

        # Give publishers a moment to connect
        time.sleep(1.0)

        self.get_logger().info("SprayPaintingDemo node started")

    # ── Spray control ──────────────────────────────────────────────────────

    def _spray(self, on: bool):
        msg = Bool()
        msg.data = on
        self._spray_pub.publish(msg)
        self.get_logger().info(f"Spray {'ON' if on else 'OFF'}")

    # ── MoveIt helpers ─────────────────────────────────────────────────────

    def _go_home(self, arm):
        """Move arm to safe home pose (joint target)."""
        self.get_logger().info("Moving to home pose…")
        with arm.get_planning_scene_monitor().read_write() as scene:
            rs = scene.current_state
            rs.set_joint_group_positions(PLANNING_GROUP, HOME_JOINTS)
            rs.update()

        arm.set_start_state_to_current_state()
        joint_goal = arm.set_joint_value_target(
            dict(zip(
                arm.get_active_joints(),
                HOME_JOINTS,
            ))
        )
        plan = arm.plan()
        if plan.error_code.val != 1:
            self.get_logger().error("Home pose planning failed!")
            return False
        arm.execute(plan.trajectory, controllers=[])
        return True

    def _cart_sweep(self, arm, start_pose: Pose, end_pose: Pose):
        """Execute a Cartesian sweep from start_pose to end_pose."""
        arm.set_start_state_to_current_state()
        waypoints = [start_pose, end_pose]
        (plan, fraction) = arm.compute_cartesian_path(
            waypoints,
            eef_step=0.01,          # 1 cm interpolation step
            jump_threshold=0.0,     # disable jump check in simulation
        )
        self.get_logger().info(f"Cartesian path fraction: {fraction:.2f}")
        if fraction < 0.8:
            self.get_logger().warn("Low path fraction — skipping this row")
            return False
        arm.execute(plan, controllers=[])
        return True

    def _go_to_pose(self, arm, pose: Pose):
        """Move to a single Cartesian pose (pre-position, spray off)."""
        arm.set_pose_target(pose)
        plan = arm.plan()
        if plan.error_code.val != 1:
            self.get_logger().warn("Pose planning failed — trying anyway")
        else:
            arm.execute(plan.trajectory, controllers=[])

    # ── Main demo ──────────────────────────────────────────────────────────

    def run(self, moveit: MoveItPy):
        arm = moveit.get_planning_component(PLANNING_GROUP)
        arm.set_max_velocity_scaling_factor(MOVE_VELOCITY_SCALE)
        arm.set_max_acceleration_scaling_factor(CART_ACCEL_SCALE)

        self.get_logger().info("=== UR5e Spray Painting Demo ===")

        # Step 1 — home
        self._spray(False)
        self._go_home(arm)
        time.sleep(1.0)

        # Step 2 — raster scan
        sweep_right = True
        for row_idx, z in enumerate(ROW_Z):
            self.get_logger().info(f"Row {row_idx + 1}/{len(ROW_Z)}  z={z:.2f} m")

            if sweep_right:
                y_start, y_end = SWEEP_Y[0], SWEEP_Y[1]
            else:
                y_start, y_end = SWEEP_Y[1], SWEEP_Y[0]

            start_pose = make_pose(NOZZLE_X, y_start, z)
            end_pose   = make_pose(NOZZLE_X, y_end,   z)

            # Pre-position (spray off)
            arm.set_max_velocity_scaling_factor(MOVE_VELOCITY_SCALE)
            self._go_to_pose(arm, start_pose)
            time.sleep(0.3)

            # Sweep (spray on)
            arm.set_max_velocity_scaling_factor(CART_VELOCITY_SCALE)
            self._spray(True)
            self._cart_sweep(arm, start_pose, end_pose)
            self._spray(False)

            sweep_right = not sweep_right
            time.sleep(0.3)

        # Step 3 — return home
        self.get_logger().info("Painting complete — returning home")
        arm.set_max_velocity_scaling_factor(MOVE_VELOCITY_SCALE)
        self._go_home(arm)

        self.get_logger().info("=== Demo finished ===")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    # MoveItPy must be initialised after rclpy.init()
    moveit = MoveItPy(node_name="moveit_py_spray_demo")

    node = SprayPaintingDemo()

    # Spin in background so ROS callbacks are served while we plan/execute
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run(moveit)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node._spray(False)   # safety: always turn spray off on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
