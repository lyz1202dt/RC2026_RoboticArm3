import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_moveit_rviz_launch

sys.path.insert(0, os.path.dirname(__file__))

from _moveit_config import build_moveit_config


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    moveit_config = build_moveit_config(backend)
    rviz_launch = generate_moveit_rviz_launch(moveit_config)

    return LaunchDescription(
        [DeclareLaunchArgument("backend", default_value="real"), *rviz_launch.entities]
    )
