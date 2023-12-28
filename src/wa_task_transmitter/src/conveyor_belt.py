from __future__ import annotations

import random
from abc import ABC, abstractmethod
from enum import StrEnum

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import UInt32

from src.entity import VariableTickActionEntity


class ConveyorBelt(VariableTickActionEntity, ABC):
    def __init__(
        self,
        task_gen_min_time: float,
        task_gen_max_time: float,
    ) -> None:
        super().__init__(
            task_gen_min_time,
            task_gen_max_time,
        )

        # Outputs
        self._task_pub = self.create_publisher(
            UInt32,
            f"wa/conveyor_belt/{self.belt_type()}",
            10,
        )

    @classmethod
    def basename(cls) -> str:
        return f"{cls.belt_type()}_conveyor_belt"

    @classmethod
    @abstractmethod
    def belt_type(cls) -> ConveyorBeltType:
        """Define the type of the conveyor belt."""

    def action_callback(self) -> None:
        self.get_logger().info("Publishing task")
        msg = UInt32()
        msg.data = self.uid
        self._task_pub.publish(msg)


class ConveyorBeltType(StrEnum):
    Input = "input"
    Output = "output"


class InputConveyorBelt(ConveyorBelt):
    @classmethod
    def belt_type(cls) -> ConveyorBeltType:
        return ConveyorBeltType.Input


class OutputConveyorBelt(ConveyorBelt):
    @classmethod
    def belt_type(cls) -> ConveyorBeltType:
        return ConveyorBeltType.Output


def main():
    # Set seed for deterministic results
    random.seed(42)

    rclpy.init()
    executor = SingleThreadedExecutor()

    belts = 3
    min_time = 3.0
    max_time = 20.0

    nodes = [InputConveyorBelt(min_time, max_time) for _ in range(belts)] + [
        OutputConveyorBelt(min_time, max_time) for _ in range(belts)
    ]

    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
