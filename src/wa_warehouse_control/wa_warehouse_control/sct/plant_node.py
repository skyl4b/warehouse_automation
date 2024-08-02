from __future__ import annotations

from typing import TYPE_CHECKING, Callable, ClassVar, Final, cast

import rclpy
import yaml
from rcl_interfaces import msg as rcl_msgs
from rclpy.node import Node
from std_msgs import msg as std_msgs
from wa_interfaces import srv as wa_srvs

from wa_warehouse_control.sct.automaton import (
    Automaton,
    AutomatonTransitions,
)

if TYPE_CHECKING:
    from rclpy.publisher import Publisher

    from wa_warehouse_control.sct.models import AutomatonModel

SPECIAL_DELIMITER: Final[str] = "|"
"""A special character to delimit automata message sections."""


class PlantNode(Node):
    """A plant (in the SCT sense) ROS node.

    Supervisory Control Theory (SCT) defines plants as "generators" of
    a language (possible sequences of events) that we desire to restrict,
    in order to obtain a specific behaviour. Those generators can be
    represented as automata, as this node does. To obtain the desired
    behaviour, we synthesize supervisors, responsible of disabling events
    that lead to undesired behaviour.

    Note that the events it disables must be "controllable"
    in the Ramadge-Wonham sense, e.g. may be disabled.

    The events are published in a "event" topic.
    """

    name: ClassVar[str] = "plant"
    """Default node name."""

    namespace: ClassVar[str] = "/wa"
    """Default node namespace."""

    event_publisher: Publisher
    """Publisher for events."""

    automaton: Automaton
    """Automaton that generates the plants's language."""

    supervisors: list[bool]
    """The current supervisors of the plant."""

    def __init__(
        self,
        state: str | None = None,
        transitions: AutomatonTransitions | None = None,
        expected_supervisors: int = 0,
        template: Callable[[int], AutomatonModel] | None = None,
    ) -> None:
        if (state is None or transitions is None) and template is None:
            raise ValueError(
                "Either a template or state and transitions must be provided",
            )

        super().__init__(  # pyright: ignore[reportArgumentType]
            node_name=self.name,
            namespace=self.namespace,
        )

        # Parameters
        self.declare_parameter("template_id", 0)
        # Handle template
        if template is not None:
            model = template(self.template_id)
            state = model["initial_state"]
            transitions = model["transitions"]
        self.declare_parameter("state", state)
        self.declare_parameter("transitions", yaml.safe_dump(transitions))
        self.declare_parameter("expected_supervisors", expected_supervisors)
        self.create_subscription(  # Monitor set parameter
            rcl_msgs.ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            10,
        )

        # Variables
        self.automaton = Automaton(self.state, self.transitions)
        self.supervisors = []

        # Subscribers
        self.create_subscription(
            std_msgs.String,
            f"{self.get_name()}/sct/enabled_events",
            self.enabled_events_callback,
            10,
        )

        # Publishers
        self.event_publisher = self.create_publisher(
            std_msgs.String,
            f"{self.get_name()}/sct/event",
            10,
        )

        # Services
        self.create_service(
            wa_srvs.RegisterSupervisor,
            f"{self.get_name()}/sct/register_supervisor",
            self.register_supervisor_callback,
        )

    @property
    def template_id(self) -> int:
        """The template id of the model on instantiation."""
        return cast(int, self.get_parameter("template_id").value)

    @property
    def state(self) -> str:
        """The last externally set state of the plant automaton."""
        return cast(str, self.get_parameter("state").value)

    @property
    def transitions(self) -> AutomatonTransitions:
        """The transitions that define the plant automaton."""
        return yaml.safe_load(
            cast(str, self.get_parameter("transitions").value),
        )

    @property
    def expected_supervisors(self) -> int:
        """The expected number of supervisors."""
        return cast(int, self.get_parameter("expected_supervisors").value)

    def parameter_event_callback(
        self,
        message: rcl_msgs.ParameterEvent,
    ) -> None:
        """Monitor this node's parameters to update the automaton."""
        if message.node == self.get_fully_qualified_name():
            for parameter in message.changed_parameters:
                if parameter.name == "state":
                    # Set current state
                    self.automaton.state = self.state
                elif parameter.name == "transitions":
                    # Set the possible transitions (rebuild automaton)
                    self.automaton = Automaton(self.state, self.transitions)

    def get_state(self) -> str:
        """Get the current state of the plant automaton."""
        return self.automaton.state

    def transition(self, event: str) -> None:
        """Undergo a state transition to the plant automaton."""
        if len(self.supervisors) != self.expected_supervisors:
            self.get_logger().warning(
                f"Event '{event}' ignored as there are "
                f"{len(self.supervisors)} registered, "
                f"not the expected number '{self.expected_supervisors}'",
            )
            return
        if not all(self.supervisors):
            self.get_logger().warning(
                f"Event '{event}' ignored as there are "
                "some supervisors have not provided enabled events, "
                f"current supervisors '{self.supervisors}'",
            )
            return

        self.supervisors = [False for _ in self.supervisors]
        self.automaton.transition(event)
        self.event_publisher.publish(std_msgs.String(data=event))
        self.get_logger().info(f"Transition with event '{event}'")

    def enabled_events_callback(self, message: std_msgs.String) -> None:
        """Set current enabled events from the supervisor."""
        id_str, events = message.data.split(SPECIAL_DELIMITER, maxsplit=1)
        id_ = int(id_str)
        if self.supervisors[id_]:
            self.get_logger().error(
                f"Supervisor '{id_}' already set "
                "enabled events in this state.",
            )
            return

        self.automaton.restrict_enabled_events(
            set(events.split(SPECIAL_DELIMITER)),
        )
        self.supervisors[int(id_)] = True

    def register_supervisor_callback(
        self,
        request: wa_srvs.RegisterSupervisor.Request,
        response: wa_srvs.RegisterSupervisor.Response,
    ) -> wa_srvs.RegisterSupervisor.Response:
        """Register a supervisor of this plant."""
        self.supervisors.append(False)
        enabled_events = set(request.enabled_events.split(SPECIAL_DELIMITER))
        response.supervisor_id = len(self.supervisors) - 1
        self.get_logger().info(
            f"Registering supervisor '{response.supervisor_id}'"
            f"with enabled events {enabled_events}",
        )
        self.automaton.restrict_enabled_events(enabled_events)
        self.supervisors[-1] = True
        return response


def main() -> None:
    """Run the plant node with random transitions."""
    import random  # noqa: PLC0415
    from time import sleep  # noqa: PLC0415

    rclpy.init()
    node = PlantNode(
        state="q0",
        transitions={
            "q0": {
                "a": "q1",
                "c": "q2",
            },
            "q1": {
                "b": "q0",
            },
        },
        expected_supervisors=1,
    )

    try:
        while True:
            try:
                if len(node.supervisors) == node.expected_supervisors:
                    event = random.choice(
                        list(
                            node.automaton.enabled_events.intersection(
                                node.automaton.current_events(),
                            ),
                        ),
                    )
                    node.transition(event)
            except IndexError:
                # Empty sequence choice
                pass
            rclpy.spin_once(node)
            sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
