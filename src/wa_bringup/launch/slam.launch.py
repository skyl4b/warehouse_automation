"""A launch file for SLAM in the warehouse automation project.

This launch files starts the simulation of the warehouse world with
SLAM enabled (see: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html).
Move the mobilebot around and, after mapping, the map file can be saved using

```bash
ros2 run nav2_map_server map_saver_cli -f ./map
```

so that it can be used later.
"""  # noqa: INP001

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def generate_launch_description():
    # Environment package for the simulation (model and world definitions)
    wa_environment = FindPackageShare("wa_environment")

    # Current gazebo model path (where to look for models)
    gazebo_model_path = EnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        default_value="",
    )

    # Build description
    return LaunchDescription(
        [
            # Configure gazebo model path
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value=[
                    gazebo_model_path,
                    ":",
                    PathJoinSubstitution([wa_environment, "models"]),
                ],
            ),
            # Launch arguments
            DeclareLaunchArgument(
                "headless",
                default_value="False",
                description="Run in headless mode",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="False",
                description="Use composition",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="True",
                description="Use SLAM",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [
                        wa_environment,
                        "worlds",
                        "warehouse.world",
                    ],
                ),
                description="Full path to world file to load",
            ),
            DeclareLaunchArgument(
                "pose",
                default_value="{x: 0, y: 0}",
                description="Initial pose for the robot",
            ),
            DeclareLaunchArgument(
                "robot_sdf",
                default_value=PathJoinSubstitution(
                    [
                        wa_environment,
                        "models",
                        "mobilebot",
                        "model.sdf",
                    ],
                ),
                description=(
                    "Full path to robot sdf file to spawn the robot in gazebo"
                ),
            ),
            # Start navigation2 bringup
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nav2_bringup"),
                            "launch",
                            "tb3_simulation_launch.py",
                        ],
                    ),
                ),
                launch_arguments={
                    "headless": LaunchConfiguration("headless"),
                    "use_composition": LaunchConfiguration("use_composition"),
                    "slam": LaunchConfiguration("slam"),
                    "world": LaunchConfiguration("world"),
                    "pose": LaunchConfiguration("pose"),
                    "robot_sdf": LaunchConfiguration("robot_sdf"),
                }.items(),
            ),
        ],
    )
