"""A node for simplifying interactions with the Gazebo simulator.

This file defines a GazeboBridge node that is responsible for streamlining
the interactions of the warehouse automation project with the Gazebo simulator.
"""

from __future__ import annotations

from typing import ClassVar, cast

import rclpy
from gazebo_msgs import srv as gazebo_srvs
from geometry_msgs import msg as geometry_msgs
from rclpy.node import Node
from wa_interfaces import srv as wa_srvs

from wa_gazebo.actions import GazeboActions
from wa_gazebo.model_parser import ModelParser
from wa_gazebo.queues import GazeboQueueHandler


class GazeboBridge(Node):
    """A node to streamline the interactions with the Gazebo simulator."""

    name: ClassVar[str] = "gazebo_bridge"
    """Node name."""

    gazebo_actions: GazeboActions
    """Gazebo simulation control and model management."""

    box_ids: list[int]
    """Storage for product_box ids in use."""

    model_parser: ModelParser
    """Gazebo model parser."""

    queue_handler: GazeboQueueHandler
    """Process Gazebo actions in batches."""

    def __init__(self) -> None:
        super().__init__(self.name)  # type: ignore[reportArgumentType]

        # Connect to services
        self.gazebo_actions = GazeboActions(self)
        while not self.gazebo_actions.client.wait_for_all(timeout_sec=5.0):
            self.get_logger().warn("Waiting for Gazebo services")

        # Model parser
        self.model_parser = ModelParser()

        # Process Gazebo actions in batches
        self.queue_handler = GazeboQueueHandler(
            self,
            self.spawn_handler,
            self.destroy_handler,
            self.attach_handler,
            self.detach_handler,
            self.gazebo_actions.start_simulation,
            self.gazebo_actions.stop_simulation,
        )

        # Start empty box ids
        self.box_ids = []

        # Spawner & destroyer
        self.create_service(
            wa_srvs.Spawn,
            f"/wa/{self.name}/spawn",
            self.queue_handler.enqueue_spawn_callback,
        )
        self.create_service(
            wa_srvs.Destroy,
            f"/wa/{self.name}/destroy",
            self.queue_handler.enqueue_destroy_callback,
        )

        # Attach & detach
        self.create_service(
            wa_srvs.ToggleAttach,
            f"/wa/{self.name}/attach",
            self.queue_handler.enqueue_attach_callback,
        )
        self.create_service(
            wa_srvs.ToggleAttach,
            f"/wa/{self.name}/detach",
            self.queue_handler.enqueue_detach_callback,
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
        self.create_service(
            wa_srvs.BoxTransfer,
            "/wa/box/transfer",
            self.box_transfer_callback,
        )

        self.get_logger().info("Gazebo bridge ready")

    def spawn_handler(
        self,
        task: wa_srvs.Spawn.Request,
    ) -> rclpy.Future | None:
        """Spawn a model in Gazebo."""
        model = self.model_parser.parse(task.name)
        if model is None:
            self.get_logger().warning(
                f"Spawner for '{task.name}' not implemented",
            )
            return None

        def done_callback(future: rclpy.Future) -> None:
            """Log spawn result."""
            result = cast(gazebo_srvs.SpawnEntity.Response, future.result())
            if result.success:
                self.get_logger().info(
                    f"Model '{task.name}' spawned at "
                    f"(x: {task.pose.position.x}, "
                    f"y: {task.pose.position.y}, "
                    f"z: {task.pose.position.z})",
                )
            else:
                self.get_logger().error(
                    f"Failed to spawn model '{task.name}': "
                    f" {result.status_message}",
                )

        # Forward spawn request to Gazebo
        self.get_logger().debug(f"Spawning model '{task.name}'")
        future = self.gazebo_actions.spawn(
            task.name,
            self.model_parser.descriptions[model],
            task.model_namespace,
            task.pose,
        )
        future.add_done_callback(done_callback)

        return future

    def destroy_handler(
        self,
        task: wa_srvs.Destroy.Request,
    ) -> rclpy.Future:
        """Destroy a model in Gazebo."""

        def done_callback(future: rclpy.Future) -> None:
            """Log destroy result."""
            result = cast(gazebo_srvs.DeleteEntity.Response, future.result())
            if result.success:
                self.get_logger().info(
                    f"Model '{task.name}' destroyed",
                )
            else:
                self.get_logger().error(
                    f"Failed to destroy model '{task.name}': "
                    f"{result.status_message}",
                )

        # Forward destroy request to Gazebo
        self.get_logger().debug(f"Destroying model '{task.name}'")
        future = self.gazebo_actions.destroy(task.name)
        future.add_done_callback(done_callback)

        return future

    def attach_handler(
        self,
        task: wa_srvs.ToggleAttach.Request,
    ) -> rclpy.Future | None:
        """Attach a model to a box in Gazebo."""
        if not any(
            task.model_1.startswith(model)
            # Attachable models
            for model in ("mobilebot", "conveyor_belt", "storage_unit")
        ) or not task.model_2.startswith("product_box"):
            self.get_logger().error(
                "Unable to pick box: "
                f"invalid model names '{task.model_1}' "
                f"and '{task.model_2}'",
            )
            return None

        def done_callback(future: rclpy.Future) -> None:
            """Log attach result."""
            result = cast(wa_srvs.Attach.Response, future.result())
            if result.success:
                self.get_logger().info(
                    f"Attached '{task.model_1}' to '{task.model_2}'",
                )
            else:
                self.get_logger().error(
                    f"Failed to attach '{task.model_1}' to '{task.model_2}': "
                    f"{result.message}",
                )

        # Forward attach request to Gazebo
        future = self.gazebo_actions.attach(task.model_1, task.model_2)
        future.add_done_callback(done_callback)

        return future

    def detach_handler(
        self,
        task: wa_srvs.ToggleAttach.Request,
    ) -> rclpy.Future | None:
        """Detach a model from a box in Gazebo."""
        if not any(
            task.model_1.startswith(model)
            # Attachable models
            for model in ("mobilebot", "conveyor_belt", "storage_unit")
        ) or not task.model_2.startswith("product_box"):
            self.get_logger().error(
                "Unable to drop box: "
                f"invalid model names '{task.model_1}' "
                f"and '{task.model_2}'",
            )
            return None

        def done_callback(future: rclpy.Future) -> None:
            """Log detach result."""
            result = cast(wa_srvs.Detach.Response, future.result())
            if result.success:
                self.get_logger().info(
                    f"Detached '{task.model_2}' from '{task.model_1}'",
                )
            else:
                self.get_logger().error(
                    f"Failed to detach '{task.model_2}'"
                    f" from '{task.model_1}': "
                    f"{result.message}",
                )

        future = self.gazebo_actions.detach(task.model_1, task.model_2)
        future.add_done_callback(done_callback)

        return future

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
        self.get_logger().info(f"Box {id_} ordered")

        # Spawn
        self.queue_handler.enqueue_spawn_callback(
            wa_srvs.Spawn.Request(
                name=f"product_box_{id_}",
                pose=geometry_msgs.Pose(
                    position=position,
                ),
            ),
            wa_srvs.Spawn.Response(),
        )

        # Attach
        self.queue_handler.enqueue_attach_callback(
            wa_srvs.ToggleAttach.Request(
                model_1=f"{request.attach_to}",
                model_2=f"product_box_{id_}",
            ),
            wa_srvs.ToggleAttach.Response(),
        )

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

        if len(request.detach_from) != 0:
            # Detach
            self.queue_handler.enqueue_detach_callback(
                wa_srvs.ToggleAttach.Request(
                    model_1=f"{request.detach_from}",
                    model_2=f"product_box_{id_}",
                ),
                wa_srvs.ToggleAttach.Response(),
            )

        # Destroy
        self.queue_handler.enqueue_destroy_callback(
            wa_srvs.Destroy.Request(
                name=f"product_box_{id_}",
            ),
            wa_srvs.Destroy.Response(),
        )

        return response

    def box_transfer_callback(
        self,
        request: wa_srvs.BoxTransfer.Request,
        response: wa_srvs.BoxTransfer.Response,
    ) -> wa_srvs.BoxTransfer.Response:
        """Transfer a box between entities in Gazebo."""
        id_ = request.box_id
        if id_ not in self.box_ids:
            self.get_logger().warning(
                f"Unknown box_id '{id_}' received.",
            )
        # Detach
        self.queue_handler.enqueue_detach_callback(
            wa_srvs.ToggleAttach.Request(
                model_1=f"{request.from_entity}",
                model_2=f"product_box_{id_}",
            ),
            wa_srvs.ToggleAttach.Response(),
        )

        # Attach
        self.queue_handler.enqueue_attach_callback(
            wa_srvs.ToggleAttach.Request(
                model_1=f"{request.to_entity}",
                model_2=f"product_box_{id_}",
            ),
            wa_srvs.ToggleAttach.Response(),
        )

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
