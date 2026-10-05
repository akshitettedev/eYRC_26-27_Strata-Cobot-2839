#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Spawn the eBot into a running Gazebo world and bridge its topics.

The world must already be up. task2b.launch.py starts the world and includes this file,
so it is not normally launched on its own.
'''

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ebot_share = get_package_share_directory('ebot_description')

    spawn_x, spawn_y, spawn_z, spawn_yaw = '0.0', '0.0', '0.3862', '0'

    floor_top_z = '0.3312'

    xacro_file = os.path.join(pkg_ebot_share, 'models', 'ebot', 'ebot_description.xacro')
    robot_desc = xacro.process_file(xacro_file, mappings={'prefix': 'ebot_'}).toxml()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ebot_state_publisher',
        namespace='ebot',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc,
        }]
    )

    create_ebot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ebot',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-name', 'ebot',
            '-topic', '/ebot/robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
        ],
    )
    spawn_ebot = TimerAction(period=2.0, actions=[create_ebot])

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ebot_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultrasonic_rl@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultrasonic_rr@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/ebot/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
    )

    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--x', spawn_x, '--y', spawn_y, '--z', floor_top_z,
            '--roll', '0', '--pitch', '0', '--yaw', spawn_yaw,
            '--frame-id', 'world',
            '--child-frame-id', 'odom',
        ],
    )

    bridge_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=create_ebot, on_exit=[bridge_node])
    )

    return LaunchDescription([
        rsp_node,
        world_to_odom,
        spawn_ebot,
        bridge_after_spawn,
    ])
