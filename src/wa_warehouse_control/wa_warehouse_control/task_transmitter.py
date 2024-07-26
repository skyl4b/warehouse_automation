from __future__ import annotations

import random
from typing import TYPE_CHECKING, ClassVar, Literal, cast

import rclpy
import yaml
from geometry_msgs import msg as geometry_msgs
from rclpy.node import Node
from std_msgs import msg as std_msgs
from wa_interfaces import msg as wa_msgs
from wa_interfaces import srv as wa_srvs

from wa_warehouse_control.task_stats import Task
from wa_warehouse_control.utils.map import BASE_MAP

if TYPE_CHECKING:
    from wa_warehouse_control.utils.map import ConveyorBelt, Map, StorageUnit


class TaskTransmitter(Node):
    """Task transmitter and allocator for robots.

    In the warehouse automation project, the tasks of moving boxes are
    demanded in random intervals. This node is responsible for coordinating
    the allocation of tasks to the robots that request them.
    """

    name: ClassVar[str] = "task_transmitter"
    """Node name."""

    input_demand: int
    """Current input demand."""

    output_demand: int
    """Current output demand."""

    task: Task | None
    """Task being broadcasted."""

    task_box_id: int | None
    """The box_id of the task being broadcasted."""

    active_tasks: list[Task]
    """Active tasks being executed by robots."""

    def __init__(self, broadcast_period: float, map_: Map) -> None:
        super().__init__(self.name)  # type: ignore[reportArgumentType]

        # Demand tracker
        self.input_demand = 0
        self.output_demand = 0

        # Task management
        self.task = None
        self.task_box_id = None
        self.active_tasks = []

        # Parameters
        self.declare_parameter("broadcast_period", broadcast_period)
        # TODO: self.add_on_set_parameters_callback(callback)
        self.declare_parameter("map", yaml.safe_dump(map_))

        # Subscribers
        self.create_subscription(
            wa_msgs.Demand,
            "/wa/demand_generator/demand",
            self.track_demand_callback,
            10,
        )

        # Timers
        self.create_timer(broadcast_period, self.broadcast_callback)

        # Publishers
        self.broadcast = self.create_publisher(
            std_msgs.UInt8,
            f"/wa/{self.name}/broadcast",
            10,
        )

        # Clients
        self.box_order = self.create_client(
            wa_srvs.BoxOrder,
            "/wa/box/order",
        )
        self.box_send = self.create_client(
            wa_srvs.BoxSend,
            "/wa/box/send",
        )
        while not (
            self.box_order.wait_for_service(5.0)
            and self.box_send.wait_for_service(5.0)
        ):
            self.get_logger().info("Waiting for box order services")

        # Services
        self.confirm = self.create_service(
            wa_srvs.TaskConfirmation,
            f"/wa/{self.name}/task_confirmation",
            self.confirmation_callback,
        )

    @property
    def broadcast_period(self) -> float:
        """Broadcast period of tasks."""
        return self.get_parameter("broadcast_period").value  # type: ignore[reportIncompatibleType]

    @property
    def map(self) -> Map:
        """Map of the warehouse."""
        return yaml.safe_load(
            self.get_parameter("map").value,  # type: ignore[reportIncompatibleType]
        )

    def track_demand_callback(self, demand: wa_msgs.Demand) -> None:
        """Track the current demand of the warehouse."""
        self.input_demand = demand.input_demand
        self.output_demand = demand.output_demand

    def broadcast_callback(self) -> None:
        """Broadcast the latest task to all robots."""
        if self.task is None:
            if (task := self.generate_task()) is None:
                return
            self.task = task

        self.get_logger().info(f"Broadcasting task {self.task.uid}")
        self.broadcast.publish(std_msgs.UInt8(data=self.task.uid))

    def generate_task(self) -> Task | None:
        """Generate an input or output task according to demand."""
        # Prioritize highest demand
        demand, task_type = max(
            (self.input_demand, "input"),
            (self.output_demand, "output"),
        )

        # No tasks to do
        if demand == 0:
            return None

        # Find conveyor belt and storage unit
        storage_unit = self.find_storage_unit(empty=task_type == "input")
        conveyor_belt = self.find_conveyor_belt(task_type)
        if storage_unit is None or conveyor_belt is None:
            storage_unit = self.find_storage_unit(empty=task_type == "output")
            task_type = "input" if task_type == "output" else "output"
            conveyor_belt = self.find_conveyor_belt(task_type)

            if storage_unit is None or conveyor_belt is None:
                # Can't do tasks right now
                return None

        if task_type == "input":
            start = conveyor_belt
            end = storage_unit
            # Input tasks should request boxes on their start and occupy
            # their target units even before arriving (prevent deadlock)
            self.box_order.call_async(
                wa_srvs.BoxOrder.Request(
                    position=geometry_msgs.Point(
                        **start["position"],
                    ),
                    attach_to=start["name"],
                ),
            ).add_done_callback(self.set_box_id_callback)
            # TODO: empty on midpoint
            start["empty"] = False
            end["empty"] = False
        else:
            start = storage_unit
            end = conveyor_belt
            end["empty"] = False

        # Generate task
        task = Task.create_task(start, end)
        self.get_logger().info(
            f"Generating {task_type} task {task.uid} from "
            f"'{start['name']}' at "
            "("
            f"x: {start['position']['x']:.2f}, y: {start['position']['y']:.2f}"
            ") "
            f"to '{end['name']}' at "
            "("
            f"x: {end['position']['x']:.2f}, y: {end['position']['y']:.2f}"
            ")",
        )
        return task

    def find_conveyor_belt(
        self,
        belt_type: Literal["input", "output"],
    ) -> ConveyorBelt | None:
        """Find an available conveyor belt."""
        for conveyor_belt in random.sample(
            self.map["conveyor_belts"][belt_type],
            k=len(self.map["conveyor_belts"][belt_type]),
        ):
            if conveyor_belt["empty"]:
                return conveyor_belt

        self.get_logger().warn("Conveyor belt not in use not found")
        return None

    def find_storage_unit(
        self,
        empty: bool,
    ) -> StorageUnit | None:
        """Find an empty or full storage unit."""
        # TODO: find closest
        for storage_unit in random.sample(
            self.map["storage_units"],
            k=len(self.map["storage_units"]),
        ):
            if storage_unit["empty"] == empty:
                return storage_unit

        self.get_logger().warn(f"Storage unit with empty={empty} not found")
        return None

    def confirmation_callback(
        self,
        request: wa_srvs.TaskConfirmation.Request,
        response: wa_srvs.TaskConfirmation.Response,
    ) -> wa_srvs.TaskConfirmation.Response:
        """Confirm or not a task to a robot that requested it."""
        if self.task is None or self.task.uid != request.task_uid:
            self.get_logger().info(
                f"Denying task {request.task_uid} to {request.robot}",
            )
            response.confirmation = False
            return response

        self.get_logger().info(
            f"Confirming task {request.task_uid} to {request.robot}",
        )

        # Build response
        response.confirmation = True
        response.task = wa_msgs.Task(
            uid=self.task.uid,
            start=wa_msgs.Model(
                name=self.task.start["name"],
                pose=geometry_msgs.Pose(
                    position=geometry_msgs.Point(
                        **self.task.start["position"],
                    ),
                ),
            ),
            end=wa_msgs.Model(
                name=self.task.end["name"],
                pose=geometry_msgs.Pose(
                    position=geometry_msgs.Point(
                        **self.task.end["position"],
                    ),
                ),
            ),
            box_id=self.task_box_id,
        )
        self.active_tasks.append(self.task)
        self.task = None
        self.task_box_id = None

        return response

    def set_box_id_callback(self, future: rclpy.Future) -> None:
        """Set the box_id of the current task."""
        result = cast(wa_srvs.BoxOrder.Response, future.result())
        self.task_box_id = result.box_id


def main() -> None:
    """Run the task transmitter."""
    rclpy.init()
    node = TaskTransmitter(
        5.0,
        map_=BASE_MAP,
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
