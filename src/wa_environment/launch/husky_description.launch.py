from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    xacro_file = xacro.process_file(
        (
            Path(get_package_share_directory("husky_description"))
            / "urdf/husky.urdf.xacro"
        ),
        mappings={
            "prefix": "/husky",
            "robot_namespace": "/husky",
            "laser_enabled": "true",
            "realsense_enabled": "true",
        },
    )

    print(xacro_file.toxml())

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time (Gazebo) if true",
            ),
        ],
    )
