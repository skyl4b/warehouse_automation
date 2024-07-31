from __future__ import annotations

import random
from typing import TYPE_CHECKING, ClassVar, Literal, cast

import rclpy
import yaml
from geometry_msgs import msg as geometry_msgs
from rcl_interfaces import msg as rcl_msgs
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs import msg as std_msgs
from std_srvs import srv as std_srvs
from wa_interfaces import msg as wa_msgs
from wa_interfaces import srv as wa_srvs

from wa_warehouse_control.task_stats import Task
from wa_warehouse_control.utils.map import BASE_MAP

if TYPE_CHECKING:
    from rclpy.client import Client
    from rclpy.publisher import Publisher
    from rclpy.timer import Timer

    from wa_warehouse_control.utils.map import (
        BoxId,
        ConveyorBelt,
        Map,
        StorageUnit,
    )


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

    active_tasks: list[Task]
    """Active tasks being executed by robots."""

    broadcast_timer: Timer
    """Timer for demand generation."""

    publish_map_timer: Timer
    """Timer for map publishing."""

    broadcast: Publisher
    """Publisher for broadcasting tasks."""

    map_publisher: Publisher
    """Publisher for the current map."""

    box_order: Client
    """Client for ordering boxes."""

    box_send: Client
    """Client for sending boxes."""

    consume_input_demand: Client
    """Client for consuming input demands."""

    consume_output_demand: Client
    """Client for consuming output demands."""

    def __init__(
        self,
        broadcast_period: float,
        map_: Map,
        publish_map_period: float = 2.0,
    ) -> None:
        super().__init__(  # pyright: ignore[reportArgumentType]
            node_name=self.name,
            namespace=self.namespace,
        )

        # Demand tracker
        self.input_demand = 0
        self.output_demand = 0

        # Task management
        self.task = None
        self.active_tasks = []

        # Parameters
        self.declare_parameter("broadcast_period", broadcast_period)
        self.declare_parameter("map", yaml.safe_dump(map_))
        self.declare_parameter("publish_map_period", publish_map_period)
        self.create_subscription(  # Monitor set parameter
            rcl_msgs.ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            10,
        )

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
        self.broadcast_timer = self.create_timer(
            self.broadcast_period,
            self.broadcast_callback,
        )
        self.publish_map_timer = self.create_timer(
            self.publish_map_period,
            self.publish_map_callback,
        )

        # Publishers
        self.broadcast = self.create_publisher(
            std_msgs.UInt8,
            f"{self.get_name()}/broadcast",
            10,
        )
        self.map_publisher = self.create_publisher(
            std_msgs.String,
            f"{self.get_name()}/map",
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
        self.consume_input_demand = self.create_client(
            std_srvs.Empty,
            "demand_generator/consume/input",
        )
        self.consume_output_demand = self.create_client(
            std_srvs.Empty,
            "demand_generator/consume/output",
        )
        while not (
            self.box_order.wait_for_service(5.0)
            and self.box_send.wait_for_service(5.0)
            and self.consume_input_demand.wait_for_service(5.0)
            and self.consume_output_demand.wait_for_service(5.0)
        ):
            self.get_logger().info(
                "Waiting for box order & consume demand services",
            )

        # Services
        self.create_service(
            wa_srvs.TaskConfirmation,
            f"{self.get_name()}/task_confirmation",
            self.confirmation_callback,
        )

    @property
    def broadcast_period(self) -> float:
        """Broadcast period of tasks."""
        return cast(float, self.get_parameter("broadcast_period").value)

    @property
    def map(self) -> Map:
        """Map of the warehouse."""
        return yaml.safe_load(cast(str, self.get_parameter("map").value))

    @property
    def publish_map_period(self) -> float:
        """Publish period of the map."""
        return cast(float, self.get_parameter("publish_map_period").value)

    def parameter_event_callback(
        self,
        message: rcl_msgs.ParameterEvent,
    ) -> None:
        """Monitor changes to this node's parameters to update the timer."""
        if message.node == self.get_fully_qualified_name():
            for parameter in message.changed_parameters:
                if parameter.name == "broadcast_period":
                    # Regenerate broadcast timer
                    self.get_logger().info(
                        "Updating broadcast timer to "
                        f"broadcast_period {self.broadcast_period}",
                    )
                    self.broadcast_timer.destroy()
                    self.broadcast_timer = self.create_timer(
                        self.broadcast_period,
                        self.broadcast_callback,
                    )
                elif parameter.name == "publish_map_period":
                    # Regenerate publish map timer
                    self.get_logger().info(
                        "Updating publish map timer to "
                        f"publish_map_period {self.publish_map_period}",
                    )
                    self.publish_map_timer.destroy()
                    self.publish_map_timer = self.create_timer(
                        self.publish_map_period,
                        self.publish_map_callback,
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

            # Consume demand
            if task.type_ == "input":
                self.consume_input_demand.call_async(
                    std_srvs.Empty.Request(),
                ).add_done_callback(
                    lambda _: self.get_logger().info(
                        "Consumed 1 input demand",
                    ),
                )
            elif task.type_ == "output":
                self.consume_output_demand.call_async(
                    std_srvs.Empty.Request(),
                ).add_done_callback(
                    lambda _: self.get_logger().info(
                        "Consumed 1 output demand",
                    ),
                )

            # Set task
            self.task = task

        self.get_logger().info(f"Broadcasting task {self.task.uid}")
        self.broadcast.publish(std_msgs.UInt8(data=self.task.uid))

    def publish_map_callback(self) -> None:
        """Publish the current map."""
        map_ = self.map
        map_builder = [
            "".join(
                "E"
                if conveyor_belt["box_id"] == "empty"
                else "I"
                if conveyor_belt["box_id"] == "in-use"
                else "F"
                for conveyor_belt in map_["conveyor_belts"]["output"]
            ),
            "".join(
                "E"
                if storage_unit["box_id"] == "empty"
                else "I"
                if storage_unit["box_id"] == "in-use"
                else "F"
                for storage_unit in map_["storage_units"]
            ),
            "".join(
                "E"
                if conveyor_belt["box_id"] == "empty"
                else "I"
                if conveyor_belt["box_id"] == "in-use"
                else "F"
                for conveyor_belt in map_["conveyor_belts"]["input"]
            ),
        ]
        self.map_publisher.publish(
            std_msgs.String(data="||".join(map_builder)),
        )

    def task_midpoint_callback(self, message: std_msgs.UInt8) -> None:
        """Update the box ids and full status for the task."""
        for task in self.active_tasks:
            if task.uid == int(message.data):
                self.set_model_box_id(task.start["name"], "empty")
                return

        self.get_logger().warn(
            f"Task {message.data} not found, "
            "can't set midpoint. "
            f"Active tasks {[task.uid for task in self.active_tasks]}",
        )

    def task_completed_callback(self, message: std_msgs.UInt8) -> None:
        """Update the box ids and full status for the task, set as complete."""
        # Remove from active tasks
        for i, task in enumerate(self.active_tasks):
            if task.uid == message.data:
                self.get_logger().info(f"Task {message.data} completed")
                if task.box_id is None:
                    self.get_logger().error(
                        f"Task {task.uid} box id not set, "
                        "can't update map or send box.",
                    )
                    self.active_tasks.pop(i)  # noqa: B909
                    return
                self.set_model_box_id(task.end["name"], task.box_id)
                if task.type_ == "output":
                    # Send box out
                    self.box_send.call_async(
                        wa_srvs.BoxSend.Request(
                            box_id=task.box_id,
                            detach_from=task.end["name"],
                        ),
                    ).add_done_callback(
                        lambda _, task=task: self.set_model_box_id(
                            task.end["name"],
                            "empty",
                        ),
                    )
                self.active_tasks.pop(i)  # noqa: B909
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
            box_id = None
        else:
            start = storage_unit
            end = conveyor_belt
            # Output tasks should known the box ids on their start and occupy
            # their target units even before arriving (prevent deadlock)
            if start["box_id"] in {"in-use", "empty"}:
                self.get_logger().error(
                    f"Box id of '{start}' should not be undefined",
                )
                return None
            box_id = start["box_id"]

        # Set to in-use for now, override with callback
        self.set_model_box_id(start["name"], "in-use")
        self.set_model_box_id(end["name"], "in-use")

        # Generate task
        task = Task.create_task(
            task_type,
            start,
            end,
            box_id,  # pyright: ignore[reportArgumentType]
        )
        self.get_logger().info(
            f"Generating {task.type_} task {task.uid} from "
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
            if map_["conveyor_belts"][belt_type][i]["box_id"] == "empty":
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
            if (empty and map_["storage_units"][i]["box_id"] == "empty") or (
                not empty
                and isinstance(map_["storage_units"][i]["box_id"], int)
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
            or not isinstance(self.task.box_id, int)
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
            box_id=self.task.box_id,
        )
        self.active_tasks.append(self.task)
        self.task = None

        return response

    def set_box_id_callback(self, future: rclpy.Future) -> None:
        """Set the box_id of the current task."""
        if self.task is None:
            self.get_logger().error("Cannot set box_id, no task currently.")
            return

        result = cast(wa_srvs.BoxOrder.Response, future.result())
        self.task.box_id = result.box_id

        # This callback should only be called on input tasks
        # Set the storage unit to the box_id received
        self.set_model_box_id(
            self.task.start["name"],
            result.box_id,
        )

    def set_model_box_id(self, model_name: str, box_id: BoxId) -> None:
        """Set model box_id in the map."""
        map_ = self.map

        # Search storage units and conveyor belts
        for model in (
            map_["storage_units"]
            + map_["conveyor_belts"]["input"]
            + map_["conveyor_belts"]["output"]
        ):
            if model["name"] == model_name:
                self.get_logger().info(
                    f"Setting '{model['name']}' box_id to '{box_id}'",
                )
                model["box_id"] = box_id
                self.update_map(map_)
                return

        self.get_logger().error(
            f"Couldn't find model '{model_name}' to set box_id {box_id}",
        )


def main() -> None:
    """Run the task transmitter."""
    rclpy.init()
    node = TaskTransmitter(
        5.0,
        map_=BASE_MAP,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
