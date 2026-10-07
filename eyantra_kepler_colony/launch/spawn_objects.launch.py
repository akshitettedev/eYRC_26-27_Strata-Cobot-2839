#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
Spawn the ore samples, the ore package and the scattered rocks into a running world.

The world must already be up. Neither Task 2 launch file includes this one: it is for
experimenting in a running world. An evaluation run refuses a world holding models the
subtask's world does not.
'''

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

_SHARE = get_package_share_directory("eyantra_kepler_colony")


def _model_file(model):
    """Absolute path to <model>/model.sdf, whether it sits in models/ or models/*/."""
    direct = os.path.join(_SHARE, "models", model, "model.sdf")
    if os.path.isfile(direct):
        return direct
    nested = sorted(glob.glob(os.path.join(_SHARE, "models", "*", model, "model.sdf")))
    if nested:
        return nested[0]
    raise RuntimeError(
        f"spawn_objects: no model.sdf for '{model}'. Looked for {direct} and "
        f"{os.path.join(_SHARE, 'models', '*', model, 'model.sdf')}. "
        "If you added a model, rebuild so it is installed."
    )


ORE_Z = 0.8
PKG_Z = 0.8

objects = [
    ("azurite_ore_1", "azurite_ore", -0.60, 1.22, ORE_Z, 0.0),
    ("malachite_ore_1", "malachite_ore", -0.74, 1.22, ORE_Z, 0.0),
    ("vanadinite_ore_1", "vanadinite_ore", -0.88, 1.22, ORE_Z, 0.0),
    ("azurite_ore_2", "azurite_ore", -0.60, 1.08, ORE_Z, 0.0),
    ("malachite_ore_2", "malachite_ore", -0.74, 1.08, ORE_Z, 0.0),
    ("vanadinite_ore_2", "vanadinite_ore", -0.88, 1.08, ORE_Z, 0.0),
    ("ore_package_1", "ore_package", -1.05, 1.15, PKG_Z, 1.5707963),
    ("rock_1", "rock_1", 1.05, 1.14, 0.3, 0.0),
    ("rock_2", "rock_2", 2.64, 2.57, 0.3, 0.5235988),
    ("rock_3", "rock_3", 4.51, 0.3877, 0.3, 1.0471976),
    ("rock_4", "rock_4", 0.82, 2.6, 0.3, 2.0943951),
    ("rock_5", "rock_5", 3.25, 1.06, 0.3, 3.6651914),
    ("rock_6", "rock_6", 4.33, 3.12, 0.3, 4.7123890),
    ("rock_7", "rock_7", 1.94, -0.3, 0.3, 5.7595865),
]


def spawn(name, model, x, y, z, yaw=0.0):
    return Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{name}",
        output="screen",
        arguments=[
            "-file",
            _model_file(model),
            "-name",
            name,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            str(yaw),
        ],
    )


def generate_launch_description():
    spawn_actions = [spawn(n, m, x, y, z, yaw) for n, m, x, y, z, yaw in objects]
    return LaunchDescription([TimerAction(period=2.0, actions=spawn_actions)])
