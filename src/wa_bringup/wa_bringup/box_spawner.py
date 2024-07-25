from __future__ import annotations

from time import sleep
from typing import TYPE_CHECKING, Callable, ClassVar, TypedDict

import rclpy
import yaml
from geometry_msgs import msg as geometry_msgs
from rclpy.node import Node
from std_msgs import msg as std_msgs
from wa_interfaces import srv as wa_srvs

if TYPE_CHECKING:
    from wa_warehouse_control.utils.map import Position


class BoxSpawnRequest(TypedDict):
    """A request to spawn a box."""

    position: Position
    """The position where to spawn the box."""

    attach_to: str
    """The model to attach the box to."""


class BoxSpawner(Node):
    """A simple node to spawn boxes in the warehouse automation project."""

    name: ClassVar[str] = "box_spawner"
    """Node name."""

    def __init__(
        self,
        box_spawns: list[BoxSpawnRequest] | BoxSpawnRequest | None = None,
    ) -> None:
        super().__init__(self.name)  # type: ignore[reportArgumentType]
        if box_spawns is None:
            box_spawns = [
                {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "attach_to": ""},
            ]
        elif not isinstance(box_spawns, list):
            box_spawns = [box_spawns]

        # Parameters
        self.declare_parameter("box_spawns", yaml.safe_dump(box_spawns))

        # Spawner service
        self.spawner = self.create_client(
            wa_srvs.BoxOrder,
            "/wa/box/order",
        )
        while not self.spawner.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for box spawner service")

        # Spawning tracker
        self._done = False

    @property
    def box_spawns(self) -> list[BoxSpawnRequest]:
        """Where and how to spawn the boxes."""
        return yaml.safe_load(self.get_parameter("box_spawns").value)  # type: ignore[reportIncompatibleType]

    def _set_done(self) -> None:
        """Set spawner done."""
        self._done = True

    def done(self) -> bool:
        """Return whether the spawner is done."""
        return self._done

    def _spawn_next_box(
        self,
        i: int,
        callback: Callable[[rclpy.Future], None],
    ) -> None:
        """Spawn the next box in the queue."""
        if i < 0:
            callback(rclpy.Future())
            return

        def spawn_callback(_: rclpy.Future) -> None:
            """Call back for logging info and sleep."""
            self.get_logger().info(
                "Spawning box at "
                "("
                f"x: {self.box_spawns[i]['position']['x']}, "
                f"y: {self.box_spawns[i]['position']['y']}, "
                f"z: {self.box_spawns[i]['position']['z']}"
                ") and attaching to "
                f"'{self.box_spawns[i]['attach_to']}'",
            )

            self.spawner.call_async(
                wa_srvs.BoxOrder.Request(
                    position=geometry_msgs.Point(
                        x=self.box_spawns[i]["position"]["x"],
                        y=self.box_spawns[i]["position"]["y"],
                        z=self.box_spawns[i]["position"]["z"],
                    ),
                    attach_to=self.box_spawns[i]["attach_to"],
                ),
            ).add_done_callback(callback)

        self._spawn_next_box(i - 1, spawn_callback)

    def spawn(self) -> None:
        """Spawn the boxes."""
        if len(self.box_spawns) == 0:
            self.get_logger().info("No boxes to spawn.")
            self._set_done()
            return

        # self._spawn_next_box(
        #     len(self.box_spawns) - 1,
        #     callback=lambda _: self._set_done(),
        # )
        for box_spawn in self.box_spawns:
            self.spawner.call_async(
                wa_srvs.BoxOrder.Request(
                    position=geometry_msgs.Point(
                        x=box_spawn["position"]["x"],
                        y=box_spawn["position"]["y"],
                        z=box_spawn["position"]["z"],
                    ),
                    attach_to=box_spawn["attach_to"],
                ),
            )


def main() -> None:
    rclpy.init()
    node = BoxSpawner(
        [
            {
                "position": {"x": 1.5, "y": -3.0, "z": 0.35},
                "attach_to": "storage_unit_1_0",
            },
            {
                "position": {"x": 1.5, "y": 3.0, "z": 0.35},
                "attach_to": "storage_unit_3_0",
            },
        ],
    )
    node.spawn()
    while not node.done():
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
