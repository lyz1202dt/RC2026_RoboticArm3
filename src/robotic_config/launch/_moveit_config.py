import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config(backend):
    robotic_config_share = get_package_share_directory("robotic_config")
    initial_positions_file = os.path.join(
        robotic_config_share, "config", "initial_positions.yaml"
    )

    return (
        MoveItConfigsBuilder("robotic_arm", package_name="robotic_config")
        .robot_description(
            file_path="config/robotic_arm.urdf.xacro",
            mappings={
                "backend": backend,
                "initial_positions_file": initial_positions_file,
            },
        )
        .to_moveit_configs()
    )
