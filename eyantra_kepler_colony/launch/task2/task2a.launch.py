#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
*****************************************************************************************
*  Filename:       task2a.launch.py
*  Description:    Task 2A -- sorting the ores. Brings up the arena, the UR7e with its mast
*                  RealSense and magnet, and four ores on the conveyor, then opens RViz. No
*                  eBot, no rocks and no ore package: nothing in this task drives.
*
*                      ros2 launch eyantra_kepler_colony task2a.launch.py
*                      ros2 launch eyantra_kepler_colony task2a.launch.py gui:=false
*
*                  It does NOT run a perception or an arm node. Detecting each ore, picking
*                  it with the magnet and dropping it in the container for its kind is the
*                  task; run your own nodes alongside this once it prints
*                  "Task 2A world ready: Start your nodes now.":
*
*                      ros2 run algorithms task2A_perception.py
*                      ros2 run algorithms task2A_manipulation.py
*
*                  What the arena gives you:
*
*                      /camera/camera/color/image_raw                   colour, 640x480
*                      /camera/camera/aligned_depth_to_color/image_raw  depth, 32FC1 m
*                      /camera/camera/color/camera_info                 intrinsics
*                      /magnet                              std_srvs/SetBool service
*                      /ur7e/end_effector/force_magnitude   std_msgs/Float32
*                      /arm_status                          std_msgs/Int32
*
*  Target:         ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8)
*****************************************************************************************
'''

import os
import tempfile

import xacro
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_ARM_DELAY = 5.0
_RVIZ_DELAY = 12.0


def _ready_line(markers, rviz_marker, message):
    """Log `message` once, after each (process, text) in `markers` has appeared in the
    output of a process whose name starts with that process, and `rviz_marker` too when
    RViz is opened. A second later, so it follows the rest of that burst of output."""

    def setup(context, *_args, **_kwargs):
        wanted = list(markers)
        if LaunchConfiguration("rviz").perform(context).strip().lower() in ("true", "1"):
            wanted.append(rviz_marker)
        seen = set()
        tails = {}

        def on_output(event):
            if len(seen) == len(wanted):
                return None
            name = event.process_name
            tail = tails.get(name, "") + event.text.decode(errors="replace")
            for i, (process, text) in enumerate(wanted):
                if name.startswith(process) and text in tail:
                    seen.add(i)
            tails[name] = tail[-512:]
            if len(seen) == len(wanted):
                return [TimerAction(period=1.0, actions=[LogInfo(msg=message)])]
            return None

        return [RegisterEventHandler(
            OnProcessIO(on_stdout=on_output, on_stderr=on_output))]

    return OpaqueFunction(function=setup)

def generate_launch_description():
    pkg_share = FindPackageShare("eyantra_kepler_colony")

    declared_arguments = [
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Run the Gazebo GUI. Set false for headless.",
        ),
        DeclareLaunchArgument(
            "verbosity", default_value="1",
            description="gz sim console verbosity (0-4).",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Open RViz.",
        ),
    ]

    resource_paths = [
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_share, "worlds"]),
            prepend=True,
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([pkg_share, "models", "rocks"]),
            prepend=True,
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([pkg_share, "models"]),
            prepend=True,
        ),
    ]

    def _launch_gz_sim(context, *_args, **_kwargs):
        gui = LaunchConfiguration("gui").perform(context).strip().lower()
        if gui not in ("true", "false"):
            raise RuntimeError(f"gui must be true or false, not '{gui}'")
        verbosity = LaunchConfiguration("verbosity").perform(context).strip()
        if verbosity not in ("0", "1", "2", "3", "4"):
            raise RuntimeError(f"verbosity must be 0, 1, 2, 3 or 4, not '{verbosity}'")
        world_path = PathJoinSubstitution(
            [pkg_share, "worlds", "eyantra_kepler_world.world.xacro"]
        ).perform(context)
        expanded_sdf = xacro.process_file(world_path).toxml()
        fd, resolved_world_path = tempfile.mkstemp(
            prefix="eyantra_kepler_world_", suffix=".world"
        )
        with os.fdopen(fd, "w") as f:
            f.write(expanded_sdf)

        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                )
            ),
            launch_arguments={
                "gz_args": " ".join(
                    (["-s"] if gui == "false" else [])
                    + ["-r", "-v", verbosity, resolved_world_path]
                ),
                "on_exit_shutdown": "true",
                "gz_version": "8",
                "ign_args": "",
                "ign_version": "",
                "debugger": "false",
                "debug_env": "false",
            }.items(),
        )
        return [gz_sim]

    gz_sim = OpaqueFunction(function=_launch_gz_sim)

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_description"), "launch", "spawn_ur7e.launch.py"]
            )
        ),
    )

    map_to_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_world",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "world",
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": True}],
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "task2a.rviz"])],
    )

    ready = _ready_line(
        [
            ("gazebo", "[KeplerArenaPlugin] Ore spawn complete."),
            ("ur_arm_controller", "ready: arm verified, safety armed"),
        ],
        ("rviz2", "[rviz2]: OpenGl version:"),
        "Task 2A world ready: Start your nodes now.",
    )

    return LaunchDescription(
        declared_arguments + resource_paths + [
            ready,
            gz_sim,
            clock_bridge,
            map_to_world,
            TimerAction(period=_ARM_DELAY, actions=[arm]),
            TimerAction(period=_RVIZ_DELAY, actions=[rviz]),
        ]
    )
