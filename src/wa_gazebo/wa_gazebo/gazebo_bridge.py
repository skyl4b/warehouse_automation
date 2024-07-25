"""A node for simplifying interactions with the Gazebo simulator.

This file defines a GazeboBridge node that is responsible for streamlining
the interactions of the warehouse automation project with the Gazebo simulator.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path
from queue import Empty, Queue
from typing import TYPE_CHECKING, Any, Callable, ClassVar, Final, cast

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs import srv as gazebo_srvs
from geometry_msgs import msg as geometry_msgs
from rclpy.node import Node
from std_msgs import msg as std_msgs
from std_srvs import srv as std_srvs
from wa_interfaces import srv as wa_srvs

if TYPE_CHECKING:
    from rclpy.client import Client

environment_dir = Path(get_package_share_directory("wa_environment"))
models_dir = environment_dir / "models"

ATTACH_JOINT = "attached_joint"
"""Joint name for attaching models."""

ATTACH_SOURCE_LINK = "attacher_link"
"""Link name for the movement driver model."""

ATTACH_TARGET_LINK = "attacher_link"
"""Link name for the carried model."""

QUEUE_HANDLER_PERIOD_S: Final[float] = 0.2
"""Period to call the queue handler in seconds."""

MAX_QUEUE_SIZE: Final[int] = 20
"""Max number of tasks in the queue of Gazebo interactions."""


@dataclass
class GazeboBridgeClients:
    """Clients for Gazebo simulation control and model management."""

    start: Client
    """Client for starting / unpausing the Gazebo simulation."""

    stop: Client
    """Client for pausing / stopping the Gazebo simulation."""

    spawn: Client
    """Client for spawning Gazebo entities."""

    destroy: Client
    """Client for destroying Gazebo entities."""

    attach: Client
    """Client for attaching Gazebo entities."""

    detach: Client
    """Client for detaching Gazebo entities."""

    def wait_for_all(self, timeout_sec: float) -> bool:
        """Wait for all clients to be ready."""
        return all(
            client.wait_for_service(timeout_sec=timeout_sec)
            for client in self.__dict__.values()
        )


@dataclass
class GazeboBridgeQueues:
    """Queues for the queue handler to operate."""

    spawner: Queue[wa_srvs.Spawn.Request] = field(
        default_factory=lambda: Queue(MAX_QUEUE_SIZE),
    )
    """Queue for the periodic spawn."""

    spawn_counter: int = 0
    """Counter for when spawning is done."""

    destroyer: Queue[wa_srvs.Destroy.Request] = field(
        default_factory=lambda: Queue(MAX_QUEUE_SIZE),
    )
    """Queue for the periodic destroy."""

    attacher: Queue[wa_srvs.ToggleAttach.Request] = field(
        default_factory=lambda: Queue(MAX_QUEUE_SIZE),
    )
    """Queue for the periodic attach."""

    detacher: Queue[wa_srvs.ToggleAttach.Request] = field(
        default_factory=lambda: Queue(MAX_QUEUE_SIZE),
    )
    """Queue for the periodic detach."""

    detach_counter: int = 0
    """Counter for when attaching is done."""

    def empty(self) -> bool:
        """Check if all GazeboBridge queues are empty."""
        return all(
            queue.empty()
            for key, queue in self.__dict__.items()
            if not key.endswith("counter")
        )


class GazeboBridge(Node):
    """A node to streamline the interactions with the Gazebo simulator."""

    name: ClassVar[str] = "gazebo_bridge"
    """Node name."""

    model_descriptions: ClassVar[dict[str, str]] = {
        file.parent.name: file.read_text()
        for file in models_dir.glob("*/model.sdf")
    }
    """Dictionary with model descriptions.

    Keys are model names and values are SDF contents.
    """

    model_pattern: re.Pattern[str]
    """Pattern to match one of the models in model_descriptions."""

    client: GazeboBridgeClients
    """Clients for Gazebo simulation control and model management."""

    box_ids: list[int]
    """Storage for product_box ids in use."""

    queue: GazeboBridgeQueues
    """Queue for Gazebo interactions."""

    def __init__(self) -> None:
        super().__init__(self.name)  # type: ignore[reportArgumentType]

        # Calculate model pattern from descriptions
        joined_models = "|".join(
            re.escape(model) for model in self.model_descriptions
        )
        self.model_pattern = re.compile(f"^({joined_models})")

        # Gazebo forward clients
        self.client = GazeboBridgeClients(
            start=self.create_client(
                std_srvs.Empty,
                "/unpause_physics",
            ),
            stop=self.create_client(
                std_srvs.Empty,
                "/pause_physics",
            ),
            spawn=self.create_client(
                gazebo_srvs.SpawnEntity,
                "/spawn_entity",
            ),
            destroy=self.create_client(
                gazebo_srvs.DeleteEntity,
                "/delete_entity",
            ),
            attach=self.create_client(
                wa_srvs.Attach,
                "/gazebo/attach",
            ),
            detach=self.create_client(
                wa_srvs.Detach,
                "/gazebo/detach",
            ),
        )

        # Wait for services
        while not self.client.wait_for_all(timeout_sec=5.0):
            self.get_logger().warn("Waiting for Gazebo services")

        # Start queues
        self.queue = GazeboBridgeQueues()

        # Queue handler timer
        self.create_timer(
            QUEUE_HANDLER_PERIOD_S,
            self.queue_handler_callback,
        )

        # Start empty box ids
        self.box_ids = []

        # Simulation control
        self.create_subscription(
            std_msgs.Empty,
            f"/wa/{self.name}/simulation/start",
            self.start_simulation_callback,
            10,
        )
        self.create_subscription(
            std_msgs.Empty,
            f"/wa/{self.name}/simulation/stop",
            self.stop_simulation_callback,
            10,
        )

        # Spawner & destroyer
        self.create_service(
            wa_srvs.Spawn,
            f"/wa/{self.name}/model/spawn",
            self.enqueue_spawn_callback,
            # self.spawn_model_callback,
        )
        self.create_service(
            wa_srvs.Destroy,
            f"/wa/{self.name}/model/destroy",
            self.enqueue_destroy_callback,
            # self.destroy_model_callback,
        )

        # Pick & drop boxes
        self.create_service(
            wa_srvs.ToggleAttach,
            f"/wa/{self.name}/box/attach",
            self.enqueue_attach_callback,
            # self.pick_box_callback,
        )
        self.create_service(
            wa_srvs.ToggleAttach,
            f"/wa/{self.name}/box/detach",
            self.enqueue_detach_callback,
            # self.drop_box_callback,
        )

        # High level interactions
        self.create_service(
            wa_srvs.BoxOrder,
            "/wa/box/order",
            self.box_order_callback,
        )
        self.create_service(
            wa_srvs.BoxSend,
            "/wa/box/send",
            self.box_send_callback,
        )

        self.get_logger().info("Gazebo handler ready")

    def start_simulation_callback(self, _: std_msgs.Empty) -> None:
        """Start or continue the simulation in Gazebo."""
        self.client.start.call_async(
            std_srvs.Empty.Request(),
        ).add_done_callback(
            lambda _: self.get_logger().info("Started the simulation"),
        )

    def stop_simulation_callback(self, _: std_msgs.Empty) -> None:
        """Pause or stop the simulation in Gazebo."""
        self.client.stop.call_async(
            std_srvs.Empty.Request(),
        ).add_done_callback(
            lambda _: self.get_logger().info("Stopped the simulation"),
        )

    def queue_handler_callback(self) -> None:
        """Process the queue of Gazebo interactions."""
        if self.queue.empty():
            # Nothing to process
            return

        # Pause simulation before handling the queue
        self.stop_simulation_callback(std_msgs.Empty())

        # Spawn models
        if not self.queue.spawner.empty():
            try:
                attach = self.queue.spawner.get_nowait()
                self.spawn_model_callback(attach, wa_srvs.Spawn.Response())
            except Empty:
                pass

        # Attach models
        elif not self.queue.attacher.empty() and self.queue.spawn_counter == 0:
            try:
                while True:
                    attach = self.queue.attacher.get_nowait()
                    self.pick_box_callback(
                        attach,
                        wa_srvs.ToggleAttach.Response(),
                    )
            except Empty:
                pass

        # Detach models
        elif not self.queue.detacher.empty():
            try:
                while True:
                    detach = self.queue.detacher.get_nowait()
                    self.drop_box_callback(
                        detach,
                        wa_srvs.ToggleAttach.Response(),
                    )
            except Empty:
                pass

        # Destroy models
        elif (
            not self.queue.destroyer.empty() and self.queue.detach_counter == 0
        ):
            try:
                while True:
                    destroy = self.queue.destroyer.get_nowait()
                    self.destroy_model_callback(
                        destroy,
                        wa_srvs.Destroy.Response(),
                    )
            except Empty:
                pass

        # Continue simulation
        self.start_simulation_callback(std_msgs.Empty())

    def enqueue_spawn_callback(
        self,
        request: wa_srvs.Spawn.Request,
        response: wa_srvs.Spawn.Response,
    ) -> wa_srvs.Spawn.Response:
        self.queue.spawner.put(request)
        self.queue.spawn_counter += 1
        return response

    def enqueue_destroy_callback(
        self,
        request: wa_srvs.Destroy.Request,
        response: wa_srvs.Destroy.Response,
    ) -> wa_srvs.Destroy.Response:
        self.queue.destroyer.put(request)
        return response

    def enqueue_attach_callback(
        self,
        request: wa_srvs.ToggleAttach.Request,
        response: wa_srvs.ToggleAttach.Response,
    ) -> wa_srvs.ToggleAttach.Response:
        self.queue.attacher.put(request)
        return response

    def enqueue_detach_callback(
        self,
        request: wa_srvs.ToggleAttach.Request,
        response: wa_srvs.ToggleAttach.Response,
    ) -> wa_srvs.ToggleAttach.Response:
        self.queue.detacher.put(request)
        self.queue.detach_counter += 1
        return response

    def spawn_model_callback(
        self,
        request: wa_srvs.Spawn.Request,
        response: wa_srvs.Spawn.Response,
        *,
        callback: Callable[[], Any] | None = None,
    ) -> wa_srvs.Spawn.Response:
        """Spawn a model in Gazebo."""
        # Match into one of the model prefixes
        match_ = self.model_pattern.match(request.name)
        if match_ is None:
            self.get_logger().warning(
                f"Spawner for '{request.name}' not implemented",
            )
            return response
        model = match_.group(0)

        def done_callback(future: rclpy.Future) -> None:
            result = cast(gazebo_srvs.SpawnEntity.Response, future.result())
            self.queue.spawn_counter -= 1
            if result.success:
                self.get_logger().info(
                    f"Model '{request.name}' spawned at "
                    f"(x: {request.pose.position.x}, "
                    f"y: {request.pose.position.y}, "
                    f"z: {request.pose.position.z})",
                )
                if callback is not None:
                    callback()
            else:
                self.get_logger().error(
                    f"Failed to spawn model '{request.name}': "
                    f" {result.status_message}",
                )

        self.get_logger().debug(f"Spawning model '{request.name}'")

        # Forward spawn request to Gazebo
        self.client.spawn.call_async(
            gazebo_srvs.SpawnEntity.Request(
                name=request.name,
                xml=self.model_descriptions[model],
                robot_namespace=request.model_namespace,
                initial_pose=request.pose,
            ),
        ).add_done_callback(done_callback)

        return response

    def destroy_model_callback(
        self,
        request: wa_srvs.Destroy.Request,
        response: wa_srvs.Destroy.Response,
        *,
        callback: Callable[[], Any] | None = None,
    ) -> wa_srvs.Destroy.Response:
        """Destroy a model in Gazebo."""
        self.get_logger().debug(f"Destroying model '{request.name}'")

        def done_callback(future: rclpy.Future) -> None:
            result = cast(gazebo_srvs.DeleteEntity.Response, future.result())
            if result.success:
                self.get_logger().info(
                    f"Model '{request.name}' destroyed",
                )
                if callback is not None:
                    callback()
            else:
                self.get_logger().error(
                    f"Failed to destroy model '{request.name}': "
                    f"{result.status_message}",
                )

        # Forward destroy request to Gazebo
        self.client.destroy.call_async(
            gazebo_srvs.DeleteEntity.Request(
                name=request.name,
            ),
        ).add_done_callback(done_callback)

        return response

    def pick_box_callback(
        self,
        request: wa_srvs.ToggleAttach.Request,
        response: wa_srvs.ToggleAttach.Response,
        *,
        callback: Callable[[], Any] | None = None,
    ) -> wa_srvs.ToggleAttach.Response:
        """Attach a model with a box in Gazebo."""
        if not any(
            request.model_1.startswith(model)
            # Attachable models
            for model in ("mobilebot", "conveyor_belt", "storage_unit")
        ) or not request.model_2.startswith("product_box"):
            self.get_logger().error(
                "Unable to pick box: "
                f"invalid model names '{request.model_1}' "
                f"and '{request.model_2}'",
            )
            return response

        # Pause simulation before spawning and attaching
        self.stop_simulation_callback(std_msgs.Empty())

        def done_callback(future: rclpy.Future) -> None:
            self.start_simulation_callback(std_msgs.Empty())
            result = cast(wa_srvs.Attach.Response, future.result())
            if result.success:
                self.get_logger().info(
                    f"Box '{request.model_2}' picked up",
                )
                if callback is not None:
                    callback()
            else:
                self.get_logger().error(
                    f"Failed to pick up box '{request.model_2}': "
                    f"{result.message}",
                )

        self.client.attach.call_async(
            wa_srvs.Attach.Request(
                joint_name=ATTACH_JOINT,
                model_name_1=request.model_1,
                link_name_1=ATTACH_SOURCE_LINK,
                model_name_2=request.model_2,
                link_name_2=ATTACH_TARGET_LINK,
            ),
        ).add_done_callback(done_callback)

        return response

    def drop_box_callback(
        self,
        request: wa_srvs.ToggleAttach.Request,
        response: wa_srvs.ToggleAttach.Response,
        *,
        callback: Callable[[], Any] | None = None,
    ) -> wa_srvs.ToggleAttach.Response:
        """Detach a model with a box in Gazebo."""
        if not any(
            request.model_1.startswith(model)
            # Attachable models
            for model in ("mobilebot", "conveyor_belt", "storage_unit")
        ) or not request.model_2.startswith("product_box"):
            self.get_logger().error(
                "Unable to drop box: "
                f"invalid model names '{request.model_1}' "
                f"and '{request.model_2}'",
            )
            return response

        def done_callback(future: rclpy.Future) -> None:
            result = cast(wa_srvs.Detach.Response, future.result())
            self.queue.detach_counter -= 1
            if result.success:
                self.get_logger().info(
                    f"Box '{request.model_2}' dropped",
                )
                if callback is not None:
                    callback()
            else:
                self.get_logger().error(
                    f"Failed to drop box '{request.model_2}': "
                    f"{result.message}",
                )

        self.client.detach.call_async(
            wa_srvs.Detach.Request(
                joint_name=ATTACH_JOINT,
                model_name_1=request.model_1,
                model_name_2=request.model_2,
            ),
        ).add_done_callback(done_callback)

        return response

    def next_box_id(self) -> int:
        """Get a new box id."""
        id_ = self.box_ids[-1] + 1 if len(self.box_ids) > 0 else 1
        self.box_ids.append(id_)
        return id_

    def box_order_callback(
        self,
        request: wa_srvs.BoxOrder.Request,
        response: wa_srvs.BoxOrder.Response,
    ) -> wa_srvs.BoxOrder.Response:
        """Spawn a product box inside Gazebo at a point."""
        id_ = self.next_box_id()
        position = request.position

        # Spawn
        self.enqueue_spawn_callback(
            wa_srvs.Spawn.Request(
                name=f"product_box_{id_}",
                pose=geometry_msgs.Pose(
                    position=position,
                ),
            ),
            wa_srvs.Spawn.Response(),
        )

        # Attach
        self.enqueue_attach_callback(
            wa_srvs.ToggleAttach.Request(
                model_1=f"{request.attach_to}",
                model_2=f"product_box_{id_}",
            ),
            wa_srvs.ToggleAttach.Response(),
        )

        # # Spawn
        # self.spawn_model_callback(
        #     wa_srvs.Spawn.Request(
        #         name=f"product_box_{id_}",
        #         pose=geometry_msgs.Pose(
        #             position=position,
        #         ),
        #     ),
        #     wa_srvs.Spawn.Response(),
        #     # Attach
        #     callback=lambda: self.pick_box_callback(
        #         wa_srvs.ToggleAttach.Request(
        #             model_1=f"{request.attach_to}",
        #             model_2=f"product_box_{id_}",
        #         ),
        #         wa_srvs.ToggleAttach.Response(),
        #     ),
        # )

        response.box_id = id_
        return response

    def box_send_callback(
        self,
        request: wa_srvs.BoxSend.Request,
        response: wa_srvs.BoxSend.Response,
    ) -> wa_srvs.BoxSend.Response:
        """Destroy a product box inside Gazebo by its id."""
        id_ = request.box_id
        try:
            self.box_ids.remove(id_)
        except ValueError:
            self.get_logger().warning(
                f"Unknown box_id '{id_}' received.",
            )

        callback = lambda: self.destroy_model_callback(  # noqa: E731
            wa_srvs.Destroy.Request(
                name=f"product_box_{id_}",
            ),
            wa_srvs.Destroy.Response(),
        )
        if len(request.detach_from) != 0:
            # Detach
            self.drop_box_callback(
                wa_srvs.ToggleAttach.Request(
                    model_1=f"{request.detach_from}",
                    model_2=f"product_box_{id_}",
                ),
                wa_srvs.ToggleAttach.Response(),
                # Destroy
                callback=callback,
            )
        else:
            # Destroy
            callback()

        return response


def main() -> None:
    """Run the GazeboBridge node."""
    rclpy.init()
    node = GazeboBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
