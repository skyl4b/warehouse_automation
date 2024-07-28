"""A launch file for simulations in the warehouse automation project.

This launch files starts the simulation of the warehouse world with
the base nodes required for it to function.
"""  # noqa: INP001

from __future__ import annotations

from pathlib import Path
from typing import Final

import yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from wa_warehouse_control.utils.map import BASE_MAP, Map

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

NAMESPACE: Final[str] = "/wa"
"""Global namespace for the project nodes."""

# Define the robot spawning space
Y_MIN: Final[float] = -6.0
"""Minimum y position to spawn."""

Y_MAX: Final[float] = 6.0
"""Maximum y position to spawn."""


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
    if arg == "all":
        return [True] * n_robots
    if arg == "none":
        return [False] * n_robots
    if arg.startswith("not"):
        not_id = int(arg.split(" ")[1])
        return [i != not_id for i in range(n_robots)]

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
                    f"{NAMESPACE}/mobilebot_{i + 1}",
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
                namespace=f"{NAMESPACE}/mobilebot_{i + 1}",
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
                    # Nav2 does not accept namespaces starting with '/'
                    "namespace": f"{NAMESPACE.lstrip('/')}/mobilebot_{i + 1}",
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
                        "nav2_params_all.yaml",
                    ]),
                    "autostart": "True",
                    "use_composition": "False",
                    "use_respawn": "True",
                }.items(),
                condition=IfCondition(str(automatic_navigation_i)),
            ),
            Node(
                package="wa_warehouse_control",
                executable="mobilebot",
                name=f"mobilebot_{i + 1}",
                output="screen",
                namespace=NAMESPACE,
                parameters=[{"initial_position": [0.0, y[i]]}],
                condition=IfCondition(
                    LaunchConfiguration("warehouse_automation"),
                ),
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
    if arg == "none":
        return set()

    return {unit for c, unit in zip(arg, all_units) if bool(int(c))}


def update_map(map_: Map, populate: set[str]) -> Map:
    """Update the map with the populated storage."""
    box_counter = 0
    for storage_unit in map_["storage_units"]:
        if storage_unit["name"] in populate:
            box_counter += 1
            storage_unit["box_id"] = box_counter
        else:
            storage_unit["box_id"] = "empty"
    return map_


def populate_storage(context: LaunchContext) -> list[Node]:
    """Populate the storage with inital boxes, startup task_transmitter."""
    populate = parse_populate(
        str(LaunchConfiguration("populate").perform(context)),
    )
    map_ = update_map(BASE_MAP, populate)

    return [
        Node(
            package="wa_bringup",
            executable="box_spawner",
            name="box_spawner",
            namespace=NAMESPACE,
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
                            lambda storage_unit: storage_unit["box_id"]
                            != "empty",
                            map_["storage_units"],
                        )
                    ]),
                },
            ],
            condition=IfCondition(str(len(populate) != 0)),
        ),
        Node(
            package="wa_warehouse_control",
            executable="task_transmitter",
            name="task_transmitter",
            output="screen",
            namespace=NAMESPACE,
            parameters=[{"map": yaml.safe_dump(map_)}],
            condition=IfCondition(
                LaunchConfiguration("warehouse_automation"),
            ),
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
                "warehouse_automation",
                default_value="True",
                description="Whether the warehouse automation should be "
                "initialized, spawn moves, create tasks, allocate robots.",
            ),
            DeclareLaunchArgument(
                "n_robots",
                default_value="1",
                description="The number of robots to be spawned immediatly",
            ),
            DeclareLaunchArgument(
                "populate",
                default_value="0101010101",
                description="The boxes in storage to populate on startup, "
                "either a binary string, 'all' or 'none'",
            ),
            DeclareLaunchArgument(
                "automatic_navigation",
                default_value="all",
                description="For which robots should be moved automatically, "
                "either a binary string, 'all', 'none' or 'not {index}'",
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
                namespace=NAMESPACE,
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
                    "namespace": f"{NAMESPACE}/mobilebot_1/",
                    "use_namespace": "True",
                    "rviz_config_file": PathJoinSubstitution([
                        wa_environment,
                        "rviz",
                        "mobilebot_1.rviz",
                    ]),
                }.items(),
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            # Populate storage with inital boxes
            OpaqueFunction(function=populate_storage),
            # Start warehouse automation
            Node(
                package="wa_warehouse_control",
                executable="demand_generator",
                name="demand_generator",
                output="screen",
                namespace=NAMESPACE,
                condition=IfCondition(
                    LaunchConfiguration("warehouse_automation"),
                ),
            ),
        ],
    )
