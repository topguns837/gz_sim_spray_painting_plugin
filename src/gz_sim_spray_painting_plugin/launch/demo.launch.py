"""
gz_sim.launch.py
================
Standalone spray-painting demo: Gazebo + ros_gz_bridge + spray nozzle.
No UR robot. Use this to verify the spray plugin in isolation.

Usage:
  ros2 launch gz_sim_spray_painting_plugin gz_sim.launch.py
  ros2 launch gz_sim_spray_painting_plugin gz_sim.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    pkg_share  = get_package_share_directory("gz_sim_spray_painting_plugin")
    pkg_prefix = get_package_prefix("gz_sim_spray_painting_plugin")
    ur_sim_models = os.path.join(
        get_package_share_directory("ur_simulation_gz"), "models"
    )

    world_path        = os.path.join(pkg_share, "worlds", "spray_painting.sdf")
    bridge_config     = os.path.join(pkg_share, "config", "ros_gz_bridge.yaml")
    nozzle_urdf       = os.path.join(pkg_share, "urdf", "spray_nozzle.urdf")

    headless = LaunchConfiguration("headless")

    set_gz_version = SetEnvironmentVariable(name="GZ_VERSION", value="harmonic")
    set_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            os.path.join(pkg_prefix, "lib", "gz_sim_spray_painting_plugin"),
            ":/ws/install/gz_ros2_control/lib:",
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.path.dirname(pkg_share) + ":",
            ur_sim_models + ":",
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
        ],
    )

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", world_path, "-r", "-v", "4"],
        output="screen",
        condition=UnlessCondition(headless),
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", world_path, "-r", "-v", "4"],
        output="screen",
        condition=IfCondition(headless),
    )

    from launch_ros.actions import Node
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    # Spawn the standalone spray nozzle at T+2 s (world loads in ~1-2 s).
    spawn_nozzle = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=[
                "gz", "service",
                "-s", "/world/spray_painting/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "20000",
                "--req",
                f'sdf_filename: "{nozzle_urdf}" name: "spray_nozzle" allow_renaming: false'
                ' pose: { position: { x: 0.0 y: 0.0 z: 0.2 } }',
            ],
            output="screen",
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo server only (no GUI).",
        ),
        set_gz_version,
        set_plugin_path,
        set_resource_path,
        gazebo,
        gazebo_headless,
        clock_bridge,
        spawn_nozzle,
    ])
