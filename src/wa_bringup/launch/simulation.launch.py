"""A launch file for simulations in the warehouse automation project.

This launch files starts the simulation of the warehouse world with
the base nodes required for it to function.
"""  # noqa: INP001

from __future__ import annotations

from pathlib import Path

import yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from wa_warehouse_control.utils.map import BASE_MAP

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

# Define the robot spawning space
Y_MIN, Y_MAX = -6, 6


def generate_radial_points(
    min_: float,
    max_: float,
    n_points: int,
) -> list[float]:
    """Distribute points radially between min and max."""
    # Calculate the midpoint
    mid = (min_ + max_) / 2

    # Skip to prevent division by zero
    if n_points == 1:
        return [mid]

    # Calculate the range
    half = n_points // 2
    range_ = (max_ - min_) / 2

    # Generate the points such that they are radially distributed
    # from the midpoint
    return [
        mid + (-1) ** i * (i // 2) * (range_ / half)
        for i in range(1, n_points + 1)
    ]


def parse_automatic_navigation(arg: str, n_robots: int) -> list[bool]:
    """Parse the populate storage argument."""
    enabled = [True] * n_robots
    if arg == "all":
        return enabled
    if arg.startswith("not"):
        not_id = int(arg.split(" ")[1])
        enabled[not_id] = False
        return enabled

    return [bool(int(c)) for c in arg.ljust(n_robots)]


def spawn_robots(
    context: LaunchContext,
) -> list[GroupAction]:
    """Spawn multiple robots in the designated space."""
    n_robots = int(LaunchConfiguration("n_robots").perform(context))
    y = generate_radial_points(Y_MIN, Y_MAX, n_robots)
    automatic_navigation = parse_automatic_navigation(
        LaunchConfiguration("automatic_navigation").perform(context),
        n_robots,
    )

    return [
        GroupAction([
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    # Source
                    "-database",
                    "mobilebot",
                    # Name
                    "-entity",
                    f"mobilebot_{i + 1}",
                    # Namespace
                    "-robot_namespace",
                    f"/wa/mobilebot_{i + 1}",
                    # Pose
                    "-y",
                    str(y[i]),
                ],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=f"/wa/mobilebot_{i + 1}",
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": Path(
                            PathJoinSubstitution([
                                FindPackageShare("wa_environment"),
                                "robots",
                                "mobilebot.urdf",
                            ]).perform(context),
                        ).read_text(encoding="utf-8"),
                    },
                ],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("nav2_bringup"),
                        "launch",
                        "bringup_launch.py",
                    ]),
                ),
                launch_arguments={
                    "namespace": f"wa/mobilebot_{i + 1}",
                    "use_namespace": "True",
                    "slam": "False",
                    "map": PathJoinSubstitution([
                        FindPackageShare("wa_environment"),
                        "maps",
                        "map.yaml",
                    ]),
                    "use_sim_time": "True",
                    "params_file": PathJoinSubstitution([
                        FindPackageShare("wa_environment"),
                        "params",
                        f"nav2_params_all_{i + 1}.yaml",
                    ]),
                    "autostart": "True",
                    "use_composition": "False",
                    "use_respawn": "True",
                }.items(),
                condition=IfCondition(str(automatic_navigation_i)),
            ),
        ])
        for i, automatic_navigation_i in zip(
            range(n_robots),
            automatic_navigation,
        )
    ]


def parse_populate(arg: str) -> set[str]:
    """Parse the populate storage argument."""
    all_units = [
        "storage_unit_0_0",
        "storage_unit_1_0",
        "storage_unit_2_0",
        "storage_unit_3_0",
        "storage_unit_4_0",
        "storage_unit_0_1",
        "storage_unit_1_1",
        "storage_unit_2_1",
        "storage_unit_3_1",
        "storage_unit_4_1",
    ]
    if arg == "all":
        return set(all_units)
    return {unit for c, unit in zip(arg, all_units) if bool(int(c))}


def populate_storage(context: LaunchContext) -> list[Node]:
    """Populate the storage with inital boxes."""
    populate = parse_populate(
        str(LaunchConfiguration("populate").perform(context)),
    )

    return [
        Node(
            condition=IfCondition(str(len(populate) != 0)),
            package="wa_bringup",
            executable="box_spawner",
            name="box_spawner",
            namespace="/wa/",
            parameters=[
                {
                    "box_spawns": yaml.safe_dump([
                        {
                            "position": {
                                "x": storage_unit["position"]["x"],
                                "y": storage_unit["position"]["y"],
                                "z": storage_unit["position"]["z"],
                            },
                            "attach_to": storage_unit["name"],
                        }
                        for storage_unit in filter(
                            lambda storage_unit: storage_unit["name"]
                            in populate,
                            BASE_MAP["storage_units"],
                        )
                    ]),
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the simulation."""
    # Environment package for the simulation (model and world definitions)
    wa_environment = FindPackageShare("wa_environment")

    # Current gazebo model path (where to look for models)
    gazebo_model_path = EnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        default_value="",
    )

    # Gazebo client process (head)
    gazebo_client = ExecuteProcess(
        condition=IfCondition(
            PythonExpression(
                ["not ", LaunchConfiguration("headless")],
            ),
        ),
        cmd=["gzclient"],
        output="screen",
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
                "n_robots",
                default_value="1",
                description="The number of robots to be spawned immediatly",
            ),
            DeclareLaunchArgument(
                "populate",
                default_value="0000000000",
                description="The boxes in storage to populate on startup, "
                "either a binary string or 'all'",
            ),
            DeclareLaunchArgument(
                "automatic_navigation",
                default_value="all",
                description="For which robots should be moved automatically, "
                "either a binary string, 'all' or 'not {index}'",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="False",
                description="Start rviz to control the first robot",
            ),
            # Gazebo server
            ExecuteProcess(
                cmd=[
                    "gzserver",
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                    PathJoinSubstitution(
                        [
                            wa_environment,
                            "worlds",
                            "warehouse.world",
                        ],
                    ),
                ],
                output="screen",
            ),
            # Gazebo client (head)
            gazebo_client,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gazebo_client,
                    on_exit=[EmitEvent(event=Shutdown())],
                ),
                condition=IfCondition(
                    PythonExpression(
                        ["not ", LaunchConfiguration("headless")],
                    ),
                ),
            ),
            # Gazebo bridge, interaction simplifier
            Node(
                package="wa_gazebo",
                executable="gazebo_bridge",
                name="gazebo_bridge",
                output="screen",
                namespace="/wa/",
            ),
            # Spawn robots
            OpaqueFunction(function=spawn_robots),
            # Rviz for robot control and visualization
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("nav2_bringup"),
                        "launch",
                        "rviz_launch.py",
                    ]),
                ),
                launch_arguments={
                    "namespace": "wa/mobilebot_1/",
                    "use_namespace": "True",
                    "rviz_config_file": PathJoinSubstitution([
                        FindPackageShare("wa_environment"),
                        "rviz",
                        "mobilebot_1.rviz",
                    ]),
                }.items(),
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            # Populate storage with inital boxes
            OpaqueFunction(function=populate_storage),
        ],
    )
