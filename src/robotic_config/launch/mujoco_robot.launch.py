from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config_dir = get_package_share_directory("robotic_config")
    launch_dir = os.path.join(moveit_config_dir, "launch")

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

    robot_description_launch_py = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch),
        launch_arguments={"backend": "mujoco"}.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch),
        launch_arguments={"backend": "mujoco"}.items(),
    )

    rviz_show = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch),
        launch_arguments={"backend": "mujoco"}.items(),
    )

    start_ros_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_control_launch),
        launch_arguments={"backend": "mujoco"}.items(),
    )

    return LaunchDescription(
        [static_tf, robot_description_launch_py, move_group, rviz_show, start_ros_control]
    )
