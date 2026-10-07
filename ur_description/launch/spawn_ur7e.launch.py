#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Spawn the UR7e arm into a running Gazebo world and bring up its controllers.

    ros2 launch ur_description spawn_ur7e.launch.py
'''

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_UR_TYPE = "ur7e"
_NAME = "ur7e"
_TF_PREFIX = ""
_X = "-0.597278"
_Y = "1.8653"
_Z = "0.8825"
_ROLL = "0.0"
_PITCH = "0.0"
_YAW = "0.0"


def generate_launch_description():
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", "ur_controllers.yaml"]
    )
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "urdf", "ur_gz.urdf.xacro"]
    )

    robot_description = {

        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", xacro_file,
                " ", "name:=", _NAME,
                " ", "ur_type:=", _UR_TYPE,
                " ", "tf_prefix:=", _TF_PREFIX,
                " ", "force_abs_paths:=true",
                " ", "mount_x:=", _X,
                " ", "mount_y:=", _Y,
                " ", "mount_z:=", _Z,
                " ", "mount_roll:=", _ROLL,
                " ", "mount_pitch:=", _PITCH,
                " ", "mount_yaw:=", _YAW,
                " ", "simulation_controllers:=", controllers_file,
            ]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    flange_to_end_effector = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="flange_to_end_effector",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "flange", "--child-frame-id", "end_effector",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", _NAME,
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    def gz_bridge(name, config, condition=None):
        return Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=name,
            parameters=[{
                "config_file": PathJoinSubstitution(
                    [FindPackageShare("ur_description"), "config", config]),
                "use_sim_time": True,
            }],
            condition=condition,
            output="screen",
        )

    bridge = gz_bridge("ur_bridge", "gz_bridge_sensors.yaml")
    camera_bridge = gz_bridge("ur_camera_bridge", "gz_bridge_camera.yaml")
    camera_depth_bridge = gz_bridge(
        "ur_camera_depth_bridge", "gz_bridge_camera_depth.yaml")
    camera_points = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        name="ur_camera_points",
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("depth_registered/image_rect",
             "/camera/camera/aligned_depth_to_color/image_raw"),
            ("rgb/image_rect_color", "/camera/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/camera/color/camera_info"),
            ("points", "/camera/camera/depth/color/points"),
        ],
        output="screen",
    )

    def spawner(controller):
        args = [
            controller,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
            "--switch-timeout", "60",
        ]
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=args,
            parameters=[{"use_sim_time": True}],
            output="screen",
        )

    joint_state_broadcaster_spawner = spawner("joint_state_broadcaster")

    forward_position_controller_spawner = spawner("forward_position_controller")

    ur_arm_controller = Node(
        package="ur_description",
        executable="ur_arm_controller",
        name="ur_arm_controller",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            flange_to_end_effector,
            spawn_entity,
            bridge,
            camera_bridge,
            camera_depth_bridge,
            camera_points,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[forward_position_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=forward_position_controller_spawner,
                    on_exit=[ur_arm_controller],
                )
            ),
        ]
    )
