from __future__ import annotations

import random
from typing import TYPE_CHECKING, ClassVar, Literal, cast

import rclpy
import yaml
from geometry_msgs import msg as geometry_msgs
from rclpy.node import Node
from rclpy.parameter import Parameter
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
    """Default node name."""

    namespace: ClassVar[str] = "/wa"
    """Default node namespace."""

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
        super().__init__(  # type: ignore[reportArgumentType]
            node_name=self.name,
            namespace=self.namespace,
        )

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
            "demand_generator/demand",
            self.track_demand_callback,
            10,
        )
        self.create_subscription(
            std_msgs.UInt8,
            "task/midpoint",
            self.task_midpoint_callback,
            10,
        )
        self.create_subscription(
            std_msgs.UInt8,
            "task/completed",
            self.task_completed_callback,
            10,
        )

        # Timers
        self.create_timer(broadcast_period, self.broadcast_callback)

        # Publishers
        self.broadcast = self.create_publisher(
            std_msgs.UInt8,
            f"{self.get_name()}/broadcast",
            10,
        )

        # Clients
        self.box_order = self.create_client(
            wa_srvs.BoxOrder,
            "box/order",
        )
        self.box_send = self.create_client(
            wa_srvs.BoxSend,
            "box/send",
        )
        while not (
            self.box_order.wait_for_service(5.0)
            and self.box_send.wait_for_service(5.0)
        ):
            self.get_logger().info("Waiting for box order services")

        # Services
        self.confirm = self.create_service(
            wa_srvs.TaskConfirmation,
            f"{self.get_name()}/task_confirmation",
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

    def update_map(self, map_: Map) -> None:
        """Update the map of the warehouse."""
        self.set_parameters([
            Parameter("map", Parameter.Type.STRING, yaml.safe_dump(map_)),
        ])

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

    def task_midpoint_callback(self, message: std_msgs.UInt8) -> None:
        """Update the box ids and full status for the task."""

    def task_completed_callback(self, message: std_msgs.UInt8) -> None:
        """Update the box ids and full status for the task, set as complete."""
        # Remove from active tasks
        for i in range(len(self.active_tasks)):
            if self.active_tasks[i].uid == message.data:
                self.active_tasks.pop(i)
                self.get_logger().info(f"Task {message.data} completed")
                return
        self.get_logger().warn(
            f"Task {message.data} not found, can't complete",
        )

    def generate_task(self) -> Task | None:
        """Generate an input or output task according to demand."""
        # Prioritize highest demand
        high_demand, high_task_type = max(
            (self.input_demand, "input"),
            (self.output_demand, "output"),
        )
        low_demand, low_task_type = min(
            (self.input_demand, "input"),
            (self.output_demand, "output"),
        )

        # No tasks to do
        if high_demand == 0:
            return None
        task_type = high_task_type

        # Find conveyor belt and storage unit
        map_ = self.map
        resources = self.find_resources(high_task_type, map_)
        if resources is None and low_demand > 0:
            # Try lower priority demand
            task_type = low_task_type
            resources = self.find_resources(low_task_type, map_)
        if resources is None:
            # Can't do tasks right now
            return None
        conveyor_belt, storage_unit = resources

        # Parse task type
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
            start["empty"] = False
            end["box_id"] = -1  # Set to -1 for now, override with callback
        else:
            start = storage_unit
            end = conveyor_belt
            # Output tasks should known the box ids on their start and occupy
            # their target units even before arriving (prevent deadlock)
            self.task_box_id = start["box_id"]
            end["empty"] = False
        self.update_map(map_)

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

    def find_resources(
        self,
        task_type: Literal["input", "output"],
        map_: Map | None = None,
    ) -> tuple[ConveyorBelt, StorageUnit] | None:
        """Find resources for a task, a conveyor belt and a storage unit."""
        if map_ is None:
            map_ = self.map

        conveyor_belt = self.find_conveyor_belt(
            task_type,
            map_=map_,
        )
        storage_unit = None

        # Find closest storage unit to conveyor belt
        if conveyor_belt is not None:
            storage_unit = self.find_storage_unit(
                empty=task_type == "input",
                map_=map_,
                closest_to=(
                    conveyor_belt["position"]["x"],
                    conveyor_belt["position"]["y"],
                ),
            )

        if conveyor_belt is None or storage_unit is None:
            return None

        return (conveyor_belt, storage_unit)

    def find_conveyor_belt(
        self,
        belt_type: Literal["input", "output"],
        map_: Map | None = None,
    ) -> ConveyorBelt | None:
        """Find an available conveyor belt."""
        if map_ is None:
            map_ = self.map

        for i in random.sample(
            range(len(map_["conveyor_belts"][belt_type])),
            k=len(map_["conveyor_belts"][belt_type]),
        ):
            if map_["conveyor_belts"][belt_type][i]["empty"]:
                return map_["conveyor_belts"][belt_type][i]

        self.get_logger().warn(f"Empty {belt_type} conveyor belt not found")
        return None

    def find_storage_unit(
        self,
        empty: bool,
        closest_to: tuple[float, float] | None = None,
        map_: Map | None = None,
    ) -> StorageUnit | None:
        """Find an empty or full storage unit."""
        if map_ is None:
            map_ = self.map
        if closest_to is None:
            # Choose center on unknown
            closest_to = 0.0, 0.0

        # Find closest free storage_unit
        x, y = closest_to
        for _, i in sorted(
            (
                (map_["storage_units"][i]["position"]["x"] - x) ** 2
                + (map_["storage_units"][i]["position"]["y"] - y) ** 2,
                i,
            )
            for i in range(len(map_["storage_units"]))
        ):
            # Ignore fallback box_ids
            if map_["storage_units"][i]["box_id"] != -1 and (
                empty == (map_["storage_units"][i]["box_id"] is None)
            ):
                return map_["storage_units"][i]

        self.get_logger().warn(
            f"{'Empty' if empty else 'Full'} storage unit not found",
        )
        return None

    def confirmation_callback(
        self,
        request: wa_srvs.TaskConfirmation.Request,
        response: wa_srvs.TaskConfirmation.Response,
    ) -> wa_srvs.TaskConfirmation.Response:
        """Confirm or not a task to a robot that requested it."""
        if (
            self.task is None
            or self.task_box_id is None
            or self.task_box_id == -1
            or self.task.uid != request.task_uid
        ):
            self.get_logger().info(
                f"Denying task {request.task_uid} to {request.robot}",
            )
            response.confirmation = False
            response.task.uid = request.task_uid
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
        if self.task is None:
            self.get_logger().error("Cannot set box_id, no task currently.")
            return

        result = cast(wa_srvs.BoxOrder.Response, future.result())
        self.task_box_id = result.box_id

        # This callback should only be called on input tasks
        # Set the unit to the box_id received
        map_ = self.map
        for storage_unit in map_["storage_units"]:
            if storage_unit["name"] == self.task.end["name"]:
                storage_unit["box_id"] = result.box_id
                self.update_map(map_)
                return
        self.get_logger().error(
            "Couldn't find storage_unit "
            f"'{self.task.end['name']}' to set box_id",
        )


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
