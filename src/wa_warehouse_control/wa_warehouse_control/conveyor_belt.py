from __future__ import annotations

from abc import ABC, abstractmethod
from enum import StrEnum

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs import msg as std_msgs

from wa_warehouse_control.automaton.automaton import Automaton
from wa_warehouse_control.utils.entity import VariableTickActionEntity


class ConveyorBelt(VariableTickActionEntity, ABC):
    """A conveyor belt either producing or consuming boxes."""

    def __init__(
        self,
        box_min_time: float = 5.0,
        box_max_time: float = 20.0,
    ) -> None:
        super().__init__()

        # Parameters
        self.declare_parameter("box_min_time", box_min_time)
        self.declare_parameter("box_max_time", box_max_time)

        self.automaton = Automaton(
            state="empty",
            transitions={
                "empty": {"box_in": "full"},
                "full": {"box_out": "empty"},
            },
        )

        # Intput & output
        self.box_in = self.create_publisher(
            std_msgs.UInt8,
            f"/wa/conveyor_belt/{self.belt_type()}/in",
            10,
        )
        self.box_out = self.create_subscription(
            std_msgs.UInt8,
            f"/wa/conveyor_belt/{self.belt_type()}/out",
            self.box_out_callback,
            10,
        )

        self.start_action_timer()

    @property
    def action_min_period(self) -> float:  # type: ignore[reportImplicitOverride]
        return self.get_parameter("box_min_time").value  # type: ignore[reportReturnType]

    @property
    def action_max_period(self) -> float:  # type: ignore[reportImplicitOverride]
        return self.get_parameter("box_max_time").value  # type: ignore[reportReturnType]

    @classmethod
    def basename(cls) -> str:  # type: ignore[reportImplicitOverride]
        """Get the name of the model (not the entity)."""
        return f"{cls.belt_type()}_conveyor_belt"

    @classmethod
    @abstractmethod
    def belt_type(cls) -> ConveyorBeltType:
        """Define the type of the conveyor belt."""

    def action_callback(self) -> None:  # type: ignore[reportImplicitOverride]
        """Request boxes on a timer."""
        if self.automaton.state == "full":
            self.get_logger().debug("Conveyor belt full")
            self.stop_action_timer()
            return
        self.automaton.transition("box_in")

        # Request a box
        self.get_logger().info("Ordering box")
        self.box_in.publish(std_msgs.UInt8(data=self.uid))
        self.stop_action_timer()

    def box_out_callback(self, message: std_msgs.UInt8) -> None:
        """Send boxes out of the conveyor belt."""
        if message.data == self.uid:
            self.automaton.transition("box_out")
            self.get_logger().debug("Conveyor belt empty")
            self.start_action_timer()


class ConveyorBeltType(StrEnum):
    """The types of conveyor belts."""

    Input = "input"
    Output = "output"


class InputConveyorBelt(ConveyorBelt):
    """An input conveyor belts."""

    @classmethod
    def belt_type(cls) -> ConveyorBeltType:  # type: ignore[reportImplicitOverride]
        return ConveyorBeltType.Input


class OutputConveyorBelt(ConveyorBelt):
    """An output conveyor belts."""

    @classmethod
    def belt_type(cls) -> ConveyorBeltType:  # type: ignore[reportImplicitOverride]
        return ConveyorBeltType.Output


def main() -> None:
    rclpy.init()
    executor = SingleThreadedExecutor()

    belts = 3
    min_time = 5.0
    max_time = 10.0

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
