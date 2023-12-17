from __future__ import annotations

from random import random
from typing import Literal, TypedDict

# from rclpy.action import ActionServer
import rclpy
from std_msgs.msg import UInt32
from wa_interfaces.srv import TaskConfirmation

from src.entity import TickActionEntity
from src.task_queue import TaskQueue
from src.task_stats import Task


class TaskTransmitter(TickActionEntity):
    def __init__(self, broadcast_period: float) -> None:
        super().__init__(broadcast_period)

        self._task_queue = TaskQueue[Task]()

        # Conveyor belt positions
        self._conveyor_belt_positions: ConveyorBeltPosition = {
            "input": [(5 * random(), 0.0) for _ in range(3)],
            "output": [(5 * random(), 10.0) for _ in range(3)],
        }

        # Inputs
        self._input_task_sub = self.create_subscription(
            UInt32,
            "wa/conveyor_belt/input",
            self.input_task_callback,
            10,
        )
        self._output_task_sub = self.create_subscription(
            UInt32,
            "wa/conveyor_belt/output",
            self.output_task_callback,
            10,
        )

        # Outputs
        self._broadcast_pub = self.create_publisher(
            UInt32,
            f"wa/{self.name}/broadcast",
            10,
        )
        self._confirmation_srv = self.create_service(
            TaskConfirmation,
            f"wa/{self.name}/confirmation",
            self.confirmation_callback,
        )

    @classmethod
    def basename(cls) -> str:
        return "task_transmitter"

    def action_callback(self) -> None:
        if task := self._task_queue.get():
            self.get_logger().info(f"Broadcasting task {task.uid}")
            msg = UInt32()
            msg.data = task.uid
            self._broadcast_pub.publish(msg)

    def task_callback(
        self,
        source_type: Literal["input", "output"],
        msg: UInt32,
    ) -> None:
        # Search the conveyor belts position
        x, y = self._conveyor_belt_positions[source_type][msg.data - 1]

        # Create task
        task = Task(x, y)
        self.get_logger().info(
            f"Received task {task.uid} from {source_type}_{msg.data}",
        )
        self._task_queue.put(task)

    def input_task_callback(self, msg: UInt32) -> None:
        self.task_callback("input", msg)

    def output_task_callback(self, msg: UInt32) -> None:
        self.task_callback("output", msg)

    def confirmation_callback(
        self,
        request: TaskConfirmation.Request,
        response: TaskConfirmation.Response,
    ) -> TaskConfirmation.Response:
        task_uid = request.task_uid
        broadcasting = self._task_queue.get()

        if broadcasting is not None and task_uid == broadcasting.uid:
            self.get_logger().info(f"Confirming task {task_uid}")
            self._task_queue.done()
            response.task_confirmed = True
        else:
            self.get_logger().info(f"Denying task {task_uid}")
            response.task_confirmed = False

        return response

    def set_robot(self) -> None:
        """Set a robot to a specific task."""


class ConveyorBeltPosition(TypedDict):
    input: PositionMap
    output: PositionMap


PositionMap = list[tuple[float, float]]


def main():
    rclpy.init()
    node = TaskTransmitter(1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
