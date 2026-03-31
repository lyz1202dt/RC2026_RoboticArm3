from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from _moveit_config import build_moveit_config


def load_yaml(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    moveit_config_dir = get_package_share_directory("robotic_config")
    launch_dir = os.path.join(moveit_config_dir, "launch")
    moveit_config = build_moveit_config("mujoco")
    servo_config = os.path.join(moveit_config_dir, "config", "moveit_servo.yaml")
    servo_params = {"moveit_servo": load_yaml(servo_config)}


    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.1", "0.09", "-0.03",  # x, y, z translation
            "0.0", "0.7071068", "0.0", "0.7071068",  # quaternion (x, y, z, w) - 90° rotation about Y
            "link4",
            "camera_link"
        ],
        output="screen",
    )

    robot_description_launch = os.path.join(launch_dir, "rsp.launch.py")
    move_group_launch = os.path.join(launch_dir, "move_group.launch.py")
    rviz_launch = os.path.join(launch_dir, "moveit_rviz.launch.py")
    ros_control_launch = os.path.join(launch_dir, "bringup.launch.py")

    robot_description_launch_py = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch),
        launch_arguments={"backend": "mujoco", "use_sim_time": "true"}.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch),
        launch_arguments={"backend": "mujoco", "use_sim_time": "true"}.items(),
    )

    rviz_show = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch),
        launch_arguments={"backend": "mujoco", "use_sim_time": "true"}.items(),
    )

    start_ros_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_control_launch),
        launch_arguments={"backend": "mujoco"}.items(),
    )

    robotic_task = Node(
        package="robotic_task",
        executable="robotic_task",
        name="robotic_task",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [static_tf_camera, robot_description_launch_py, move_group, rviz_show, start_ros_control, robotic_task]
    )
