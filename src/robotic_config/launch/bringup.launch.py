import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(__file__))

from _moveit_config import build_moveit_config


def _create_real_bringup(moveit_config, controller_config):
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controller_config,
            moveit_config.robot_description,
            {"hardware": "arm_hw"},
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="both",
            )
        ],
    )

    robotic_arm_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "robotic_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="both",
            )
        ],
    )

    return [
        ros2_control_node,
        joint_state_broadcaster_spawner,
        robotic_arm_controller_spawner,
    ]


def _create_mujoco_bringup(moveit_config, controller_config):
    robotic_arm_share = get_package_share_directory("robotic_arm")
    mujoco_model_file = os.path.join(robotic_arm_share, "robotic_arm_mujoco.xml")

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        parameters=[
            moveit_config.robot_description,
            controller_config,
            {"simulation_frequency": 500.0},
            {"realtime_factor": 1.0},
            {"robot_model_path": mujoco_model_file},
            {"show_gui": True},
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="both",
    )

    arm_pid_controller_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_pid_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="both",
            )
        ],
    )

    robotic_arm_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "robotic_arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="both",
            )
        ],
    )

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting mujoco controllers..."),
                joint_state_broadcaster_spawner,
                arm_pid_controller_spawner,
                robotic_arm_controller_spawner,
            ],
        )
    )

    return [mujoco, load_controllers]


def _launch_setup(context):
    backend = LaunchConfiguration("backend").perform(context)
    moveit_config = build_moveit_config(backend)
    controller_config_dir = get_package_share_directory("robotic_config")

    controller_file = (
        "ros2_controllers_mujoco.yaml"
        if backend == "mujoco"
        else "ros2_controllers.yaml"
    )
    controller_config = os.path.join(controller_config_dir, "config", controller_file)

    if backend == "mujoco":
        return _create_mujoco_bringup(moveit_config, controller_config)

    return _create_real_bringup(moveit_config, controller_config)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="real"),
        OpaqueFunction(function=_launch_setup),
    ])
