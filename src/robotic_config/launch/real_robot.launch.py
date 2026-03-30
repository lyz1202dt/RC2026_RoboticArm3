from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():


#获取其他launch文件的目录
    moveit_config_dir = get_package_share_directory("robotic_config")
    launch_dir = os.path.join(moveit_config_dir, "launch")
    
    #发布相机到关节1的静态坐标变换x+90,y+90,z+90
    static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0.04023', '-0.20514', '0.26134',
        '1.570796', '-1.570796', '1.570796',
        'link1', 'camera_link'
    ]
)


    #选择启动Moveit Setup Assistant生成的launch文件
    robot_description_launch=os.path.join(launch_dir,"rsp.launch.py")
    move_group_launch = os.path.join(launch_dir, "move_group.launch.py")
    rviz_launch        = os.path.join(launch_dir, "moveit_rviz.launch.py")
    ros_control_launch        = os.path.join(launch_dir, "bringup.launch.py")

    backend_arguments = {"backend": "real"}.items()

    start_ros_control=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ros_control_launch),
        launch_arguments=backend_arguments,
    )

    robot_description_launch_py=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch),
        launch_arguments={"backend": "real"}.items(),
    )
    
    move_group=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch),
        launch_arguments={"backend": "real"}.items(),
    )

    rviz_show=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch),
        launch_arguments={"backend": "real"}.items(),
    )


    return LaunchDescription([static_tf,robot_description_launch_py,move_group,rviz_show,joint_driver,start_ros_control])
