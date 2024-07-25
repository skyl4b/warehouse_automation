"""A launch file for manual navigation in the warehouse automation project.

This launch files starts the simulation of the warehouse world with
teleop_twist_keyboard enabled, so that one can manually override
a robot's movement in the warehouse automation project.
"""  # noqa: INP001

from __future__ import annotations

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the manual_navigation."""
    # Manual robot control with teleop_twist_keyboard
    teleop_twist_keyboard = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twist_keyboard",
        output="screen",
        prefix="xterm -e",
        remappings=[("/cmd_vel", "/wa/mobilebot_1/cmd_vel")],
    )

    # Build description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "n_robots",
                default_value="1",
                description="The number of robots in to be spawned immediatly",
            ),
            DeclareLaunchArgument(
                "populate",
                default_value="0000000000",
                description="The boxes in storage to populate on startup, "
                "either a binary string or 'all'",
            ),
            # Standard simulation launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        ThisLaunchFileDir(),
                        "simulation.launch.py",
                    ]),
                ),
                launch_arguments={
                    "headless": "False",
                    "n_robots": LaunchConfiguration("n_robots"),
                    "populate": LaunchConfiguration("populate"),
                    "automatic_navigation": "not 0",
                }.items(),
            ),
            # Manual robot control with teleop_twist_keyboard
            teleop_twist_keyboard,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=teleop_twist_keyboard,
                    on_exit=[EmitEvent(event=Shutdown())],
                ),
            ),
        ],
    )
