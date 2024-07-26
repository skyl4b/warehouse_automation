"""Helpers to execute actions to Gazebo."""

from dataclasses import InitVar, dataclass, field

import rclpy
from gazebo_msgs import srv as gazebo_srvs
from geometry_msgs import msg as geometry_msgs
from rclpy.client import Client
from rclpy.node import Node
from std_srvs import srv as std_srvs
from wa_interfaces import srv as wa_srvs

ATTACH_JOINT = "attached_joint"
"""Joint name for attaching models."""

ATTACH_SOURCE_LINK = "attacher_link"
"""Link name for the movement driver model."""

ATTACH_TARGET_LINK = "attacher_link"
"""Link name for the carried model."""


@dataclass
class GazeboClients:
    """Clients for Gazebo simulation control and model management."""

    node: InitVar[Node]
    """Node from where to create the clients."""

    start: Client = field(init=False)
    """Client for starting / unpausing the Gazebo simulation."""

    stop: Client = field(init=False)
    """Client for pausing / stopping the Gazebo simulation."""

    spawn: Client = field(init=False)
    """Client for spawning Gazebo entities."""

    destroy: Client = field(init=False)
    """Client for destroying Gazebo entities."""

    attach: Client = field(init=False)
    """Client for attaching Gazebo entities."""

    detach: Client = field(init=False)
    """Client for detaching Gazebo entities."""

    def __post_init__(self, node: Node) -> None:
        """Instantiate clients."""
        self.start = node.create_client(
            std_srvs.Empty,
            "/unpause_physics",
        )
        self.stop = node.create_client(
            std_srvs.Empty,
            "/pause_physics",
        )
        self.spawn = node.create_client(
            gazebo_srvs.SpawnEntity,
            "/spawn_entity",
        )
        self.destroy = node.create_client(
            gazebo_srvs.DeleteEntity,
            "/delete_entity",
        )
        self.attach = node.create_client(
            wa_srvs.Attach,
            "/gazebo/attach",
        )
        self.detach = node.create_client(
            wa_srvs.Detach,
            "/gazebo/detach",
        )

    def wait_for_all(self, timeout_sec: float) -> bool:
        """Wait for all clients to be ready."""
        return all(
            client.wait_for_service(timeout_sec=timeout_sec)
            for client in self.__dict__.values()
        )


@dataclass
class GazeboActions:
    """Execute interactions to Gazebo."""

    node: InitVar[Node]
    """Node from where to create the clients."""

    client: GazeboClients = field(init=False)
    """Clients to interact with the Gazebo simulation."""

    def __post_init__(self, node: Node) -> None:
        """Instantiate clients."""
        self.client = GazeboClients(node)

    def start_simulation(self) -> rclpy.Future:
        """Start the Gazebo simulation."""
        return self.client.start.call_async(
            std_srvs.Empty.Request(),
        )

    def stop_simulation(self) -> rclpy.Future:
        """Pause / stop the Gazebo simulation."""
        return self.client.stop.call_async(
            std_srvs.Empty.Request(),
        )

    def spawn(
        self,
        name: str,
        description: str,
        namespace: str,
        pose: geometry_msgs.Pose,
    ) -> rclpy.Future:
        """Spawn an entity in Gazebo."""
        return self.client.spawn.call_async(
            gazebo_srvs.SpawnEntity.Request(
                name=name,
                xml=description,
                robot_namespace=namespace,
                initial_pose=pose,
            ),
        )

    def destroy(self, name: str) -> rclpy.Future:
        """Destroy an entity in Gazebo."""
        return self.client.destroy.call_async(
            gazebo_srvs.DeleteEntity.Request(
                name=name,
            ),
        )

    def attach(self, model_1: str, model_2: str) -> rclpy.Future:
        """Attach two models in Gazebo using the standard joint & link."""
        return self.client.attach.call_async(
            wa_srvs.Attach.Request(
                joint_name=ATTACH_JOINT,
                model_name_1=model_1,
                link_name_1=ATTACH_SOURCE_LINK,
                model_name_2=model_2,
                link_name_2=ATTACH_TARGET_LINK,
            ),
        )

    def detach(self, model_1: str, model_2: str) -> rclpy.Future:
        """Detach two models in Gazebo using the standard joint & link."""
        return self.client.detach.call_async(
            wa_srvs.Detach.Request(
                joint_name=ATTACH_JOINT,
                model_name_1=model_1,
                model_name_2=model_2,
            ),
        )
