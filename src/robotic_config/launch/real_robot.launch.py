import os
import sys
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))

from _moveit_config import build_moveit_config


def load_yaml(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    moveit_config_dir = get_package_share_directory("robotic_config")
    launch_dir = os.path.join(moveit_config_dir, "launch")
    moveit_config = build_moveit_config("real")
    servo_config = os.path.join(moveit_config_dir, "config", "moveit_servo.yaml")
    servo_params = {"moveit_servo": load_yaml(servo_config)}

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.04023",
            "-0.20514",
            "0.26134",
            "1.570796",
            "-1.570796",
            "1.570796",
            "link1",
            "camera_link",
        ],
    )

    robot_description_launch = os.path.join(launch_dir, "rsp.launch.py")
    move_group_launch = os.path.join(launch_dir, "move_group.launch.py")
    rviz_launch = os.path.join(launch_dir, "moveit_rviz.launch.py")
    ros_control_launch = os.path.join(launch_dir, "bringup.launch.py")

    backend_arguments = {"backend": "real"}.items()

    start_ros_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_control_launch),
        launch_arguments=backend_arguments,
    )

    robotic_task = Node(
        package="robotic_task",
        executable="robotic_task",
        name="robotic_task",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {"use_sim_time": False},
        ],
    )

    robot_description_launch_py = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch),
        launch_arguments={"backend": "real", "use_sim_time": "false"}.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch),
        launch_arguments={"backend": "real", "use_sim_time": "false"}.items(),
    )

    rviz_show = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch),
        launch_arguments={"backend": "real", "use_sim_time": "false"}.items(),
    )

    return LaunchDescription(
        [static_tf, robot_description_launch_py, move_group, rviz_show, start_ros_control, robotic_task]
    )
