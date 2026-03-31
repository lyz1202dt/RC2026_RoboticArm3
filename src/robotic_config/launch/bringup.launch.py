import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
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

    suction_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "suction_controller",
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
        suction_controller_spawner,
    ]


def _create_mujoco_bringup(moveit_config, controller_config):
    robotic_arm_share = get_package_share_directory("robotic_arm")
    mujoco_model_file = os.path.join(robotic_arm_share, "scene.xml")

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

    arm_pid_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_pid_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="both",
    )

    robotic_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robotic_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="both",
    )

    suction_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "suction_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="both",
    )

    start_arm_pid_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                LogInfo(msg="joint_state_broadcaster ready, starting arm_pid_controller..."),
                arm_pid_controller_spawner,
            ],
        )
    )

    start_robotic_arm_after_arm_pid = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_pid_controller_spawner,
            on_exit=[
                LogInfo(msg="arm_pid_controller ready, starting robotic_arm_controller..."),
                robotic_arm_controller_spawner,
            ],
        )
    )

    start_suction_after_robotic_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=robotic_arm_controller_spawner,
            on_exit=[
                LogInfo(msg="robotic_arm_controller processed, starting suction_controller..."),
                suction_controller_spawner,
            ],
        )
    )

    load_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="Starting mujoco controllers..."),
                joint_state_broadcaster_spawner,
            ],
        )
    )

    return [
        mujoco,
        load_controllers,
        start_arm_pid_after_joint_state,
        start_robotic_arm_after_arm_pid,
        start_suction_after_robotic_arm,
    ]


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
