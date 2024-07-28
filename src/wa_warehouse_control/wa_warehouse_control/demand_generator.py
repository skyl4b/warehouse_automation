from __future__ import annotations

import random
from typing import TYPE_CHECKING, ClassVar, Final, Literal

import rclpy
from rcl_interfaces import msg as rcl_msgs
from rclpy.node import Node
from std_srvs import srv as std_srvs
from wa_interfaces import msg as wa_msgs

if TYPE_CHECKING:
    from rclpy.publisher import Publisher
    from rclpy.timer import Timer


MAX_DEMAND: Final[int] = 255
"""The maximum possible demand."""


class DemandGenerator(Node):
    """Generates the demand of tasks in random intervals."""

    name: ClassVar[str] = "demand_generator"
    """Default node name."""

    namespace: ClassVar[str] = "/wa"
    """Default node namespace."""

    input_demand: int
    """Current input demand."""

    output_demand: int
    """Current output demand."""

    demand_timer: Timer
    """Timer for demand generation."""

    demand_publisher: Publisher
    """Publisher for the current demand."""

    def __init__(
        self,
        min_demand_period: float,
        max_demand_period: float,
        input_probability: float = 0.5,
    ) -> None:
        super().__init__(  # type: ignore[reportArgumentType]
            node_name=self.name,
            namespace=self.namespace,
        )

        # Initialize demand
        self.input_demand = 0
        self.output_demand = 0

        # Parameters
        self.declare_parameter("min_demand_period", min_demand_period)
        self.declare_parameter("max_demand_period", max_demand_period)
        self.declare_parameter("input_probability", input_probability)
        self.create_subscription(  # Monitor set parameter
            rcl_msgs.ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            10,
        )

        # Timers
        self.demand_timer = self.create_timer(
            restricted_normal_distribution(
                self.min_demand_period,
                self.max_demand_period,
            ),
            self.demand_callback,
        )

        # Publishers
        self.demand_publisher = self.create_publisher(
            wa_msgs.Demand,
            f"{self.get_name()}/demand",
            10,
        )

        # Services
        for type_ in ("input", "output"):
            self.create_service(
                std_srvs.Empty,
                f"{self.get_name()}/consume/{type_}",
                lambda request, response, type_=type_: self.consume(
                    request,  # type: ignore[reportArgumentType]
                    response,
                    type_,
                ),
            )

    @property
    def min_demand_period(self) -> float:
        """Minimum period for the demand."""
        return self.get_parameter("min_demand_period").value  # type: ignore[reportReturnType]

    @property
    def max_demand_period(self) -> float:
        """Maximum period for the demand."""
        return self.get_parameter("max_demand_period").value  # type: ignore[reportReturnType]

    @property
    def input_probability(self) -> float:
        """Probability for an input demand (1 - probability output)."""
        return self.get_parameter("input_probability").value  # type: ignore[reportReturnType]

    def parameter_event_callback(
        self,
        message: rcl_msgs.ParameterEvent,
    ) -> None:
        """Monitor changes to this node's parameters to update the timer."""
        if message.node == self.get_fully_qualified_name():
            for parameter in message.changed_parameters:
                if parameter.name in {
                    "min_demand_period",
                    "max_demand_period",
                }:
                    # Regenerate demand timer
                    self.demand_callback(increase_demand=False)

    def demand_callback(self, increase_demand: bool = True) -> None:
        """Generate a demand."""
        if increase_demand:
            # Input or output
            if random.random() < self.input_probability:
                self.input_demand = min(
                    self.input_demand + 1,
                    MAX_DEMAND,
                )
            else:
                self.output_demand = min(
                    self.output_demand + 1,
                    MAX_DEMAND,
                )

        # Setup next demand time
        self.demand_timer.destroy()
        next_demand = restricted_normal_distribution(
            self.min_demand_period,
            self.max_demand_period,
        )
        self.demand_timer = self.create_timer(
            next_demand,
            self.demand_callback,
        )

        # Publish demand
        self.get_logger().info(
            "Demand "
            f"{{input: {self.input_demand}, output: {self.output_demand}}}, "
            f"next in {next_demand:.2f} s",
        )
        self.demand_publisher.publish(
            wa_msgs.Demand(
                input_demand=self.input_demand,
                output_demand=self.output_demand,
            ),
        )

    def consume(
        self,
        _request: std_srvs.Empty.Request,
        response: std_srvs.Empty.Response,
        type_: Literal["input", "output"],
    ) -> std_srvs.Empty.Response:
        """Consume a demand, decreasing it by one."""
        if type_ == "input":
            self.input_demand = max(self.input_demand - 1, 0)
        else:
            self.output_demand = max(self.output_demand - 1, 0)

        self.get_logger().info(
            f"Consumed 1 {type_}, demand "
            f"{{input: {self.input_demand}, output: {self.output_demand}}}",
        )
        self.demand_publisher.publish(
            wa_msgs.Demand(
                input_demand=self.input_demand,
                output_demand=self.output_demand,
            ),
        )
        return response


def restricted_normal_distribution(min_: float, max_: float) -> float:
    """Get a random samble from a limited normal distribution."""
    sample = random.normalvariate(
        mu=(max_ + min_) / 2,
        sigma=(max_ - min_) / 4,
    )
    return min(max(min_, sample), max_)


def main() -> None:
    """Run the demand generator."""
    rclpy.init()
    node = DemandGenerator(
        20.0,
        60.0,
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
