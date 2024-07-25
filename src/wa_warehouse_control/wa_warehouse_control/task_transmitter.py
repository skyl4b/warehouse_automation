from __future__ import annotations

import random
from functools import partial
from typing import TYPE_CHECKING, Literal

import rclpy
import yaml
from geometry_msgs import msg as geometry_msgs
from std_msgs import msg as std_msgs
from wa_interfaces import srv as wa_srvs

from wa_warehouse_control.task_queue import TaskQueue
from wa_warehouse_control.task_stats import Task
from wa_warehouse_control.utils.entity import TickActionEntity
from wa_warehouse_control.utils.map import BASE_MAP

if TYPE_CHECKING:
    from wa_warehouse_control.utils.map import Map, StorageUnit


class TaskTransmitter(TickActionEntity):
    def __init__(self, broadcast_period: float, map_: Map) -> None:
        super().__init__()

        # Parameters
        self.declare_parameter("broadcast_period", broadcast_period)
        # self.add_on_set_parameters_callback(callback)
        self.declare_parameter("map", yaml.safe_dump(map_))

        # Queue of tasks
        self._task_queue = TaskQueue[Task]()

        # Inputs
        for belt_type in ("input", "output"):
            self.create_subscription(
                std_msgs.UInt8,
                f"/wa/conveyor_belt/{belt_type}/in",
                partial(self.task_callback, belt_type=belt_type),
                10,
            )

        # Outputs
        self.broadcast = self.create_publisher(
            std_msgs.UInt8,
            f"/wa/{self.name}/broadcast",
            10,
        )
        self._confirmation_srv = self.create_service(
            wa_srvs.TaskConfirmation,
            f"/wa/{self.name}/task_confirmation",
            self.confirmation_callback,
        )
        self.box_order = self.create_client(
            wa_srvs.BoxOrder,
            "/wa/box/order",
        )

        # Timers
        self.start_action_timer()

    @property
    def action_period(self) -> float:  # type: ignore[reportImplicitOverride]
        return self.get_parameter("broadcast_period").value  # type: ignore[reportIncompatibleType]

    @property
    def map(self) -> Map:
        return yaml.safe_load(
            self.get_parameter("map").value,  # type: ignore[reportIncompatibleType]
        )

    def find_storage_unit(
        self,
        empty: bool,
    ) -> StorageUnit:
        for storage_unit in random.sample(
            self.map["storage_units"],
            k=len(self.map["storage_units"]),
        ):
            if storage_unit["empty"] == empty:
                return storage_unit

        self.get_logger().info(f"Storage unit with empty={empty} not found")
        raise ValueError(f"Storage unit with full={empty} not found")

    @classmethod
    def basename(cls) -> str:  # type: ignore[reportImplicitOverride]
        return "task_transmitter"

    def action_callback(self) -> None:  # type: ignore[reportImplicitOverride]
        if task := self._task_queue.get():
            self.get_logger().info(f"Broadcasting task {task.uid}")
            self.broadcast.publish(std_msgs.UInt8(data=task.uid))

    def task_callback(
        self,
        msg: std_msgs.UInt8,
        belt_type: Literal["input", "output"],
    ) -> None:
        # Choose storage unit
        try:
            storage_unit = self.find_storage_unit(
                empty=belt_type == "input",
            )
        except ValueError:
            # TODO: check again
            return

        # Define conveyor belt
        conveyor_belt = self.map["conveyor_belts"][belt_type][msg.data - 1]

        # Create task
        if belt_type == "input":
            task = Task(conveyor_belt["position"], storage_unit["position"])
            # Input tasks should request boxes on their start and occupy
            # their target units even before arriving (prevent deadlock)
            if self.box_order.service_is_ready():
                self.box_order.call_async(
                    wa_srvs.BoxOrder.Request(
                        position=geometry_msgs.Point(
                            **conveyor_belt["position"],
                        ),
                        attach_to=conveyor_belt["name"],
                    ),
                )
            storage_unit["empty"] = False
        else:
            task = Task(
                storage_unit["position"],
                conveyor_belt["position"],
            )

        self.get_logger().info(
            f"Received task {task.uid} from "
            f"'{conveyor_belt['name']}' at "
            "("
            f"x: {conveyor_belt['position']['x']}, "
            f"y: {conveyor_belt['position']['y']}, "
            f"z: {conveyor_belt['position']['z']}"
            ")",
        )
        self._task_queue.put(task)

    def confirmation_callback(
        self,
        request: wa_srvs.TaskConfirmation.Request,
        response: wa_srvs.TaskConfirmation.Response,
    ) -> wa_srvs.TaskConfirmation.Response:
        task_uid, entity = request.task_uid, request.entity
        broadcasting = self._task_queue.get()

        if broadcasting is not None and task_uid == broadcasting.uid:
            self.get_logger().info(f"Confirming task {task_uid} to {entity}")
            self._task_queue.done()
            response.task_confirmed = True
        else:
            self.get_logger().info(f"Denying task {task_uid} to {entity}")
            response.task_confirmed = False

        return response

    def set_robot(self) -> None:
        """Set a robot to a specific task."""


def main():
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
