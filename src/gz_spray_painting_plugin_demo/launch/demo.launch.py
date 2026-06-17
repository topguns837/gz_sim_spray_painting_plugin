"""
demo.launch.py
==============
Standalone spray-painting demo: Gazebo + ros_gz_bridge + spray nozzle.
No UR robot. Use this to verify the spray plugin in isolation with any world.

Usage:
  ros2 launch gz_spray_painting_plugin_demo demo.launch.py
  ros2 launch gz_spray_painting_plugin_demo demo.launch.py world:=test_all_geometry
  ros2 launch gz_spray_painting_plugin_demo demo.launch.py world:=test_complex_meshes
  ros2 launch gz_spray_painting_plugin_demo demo.launch.py world:=test_cube
  ros2 launch gz_spray_painting_plugin_demo demo.launch.py headless:=true
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    world_stem        = LaunchConfiguration("world").perform(context)
    headless          = LaunchConfiguration("headless")
    demo_pkg_share    = get_package_share_directory("gz_spray_painting_plugin_demo")
    plugin_pkg_prefix = get_package_prefix("gz_sim_spray_painting_plugin")

    world_path    = os.path.join(demo_pkg_share, "worlds", f"{world_stem}.sdf")
    bridge_config = os.path.join(demo_pkg_share, "config", "ros_gz_bridge.yaml")
    nozzle_urdf   = os.path.join(demo_pkg_share, "urdf", "spray_nozzle.urdf")
    spawn_topic   = f"/world/{world_stem}/create"

    set_gz_version = SetEnvironmentVariable(name="GZ_VERSION", value="harmonic")
    set_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=[
            os.path.join(plugin_pkg_prefix, "lib", "gz_sim_spray_painting_plugin"),
            ":/ws/install/gz_ros2_control/lib:",
            EnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", default_value=""),
        ],
    )
    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            os.path.dirname(demo_pkg_share) + ":",
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
                "-s", spawn_topic,
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "20000",
                "--req",
                f'sdf_filename: "{nozzle_urdf}" name: "spray_nozzle" allow_renaming: false'
                ' pose: { position: { x: 0.0 y: 0.0 z: 0.6 }'
                ' orientation: { x: -0.5 y: 0.5 z: -0.5 w: 0.5 } }',
            ],
            output="screen",
        )],
    )

    return [
        set_gz_version,
        set_plugin_path,
        set_resource_path,
        gazebo,
        gazebo_headless,
        clock_bridge,
        spawn_nozzle,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="spray_painting",
            description="SDF world stem (filename without .sdf). "
                        "Available: spray_painting, test_all_geometry, test_complex_meshes, test_cube",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo server only (no GUI).",
        ),
        OpaqueFunction(function=launch_setup),
    ])
