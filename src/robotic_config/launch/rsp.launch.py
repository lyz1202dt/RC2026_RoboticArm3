import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from moveit_configs_utils.launches import generate_rsp_launch

sys.path.insert(0, os.path.dirname(__file__))

from _moveit_config import build_moveit_config


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    use_sim_time = LaunchConfiguration("use_sim_time")
    moveit_config = build_moveit_config(backend)
    rsp_launch = generate_rsp_launch(moveit_config)

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="real"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            SetParameter(name="use_sim_time", value=use_sim_time),
            *rsp_launch.entities,
        ]
    )
