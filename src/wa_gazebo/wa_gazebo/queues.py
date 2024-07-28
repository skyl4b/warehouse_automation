"""Queue for batch processing of Gazebo interactions."""

from __future__ import annotations

from dataclasses import InitVar, dataclass, field
from typing import TYPE_CHECKING, Callable, Final, Literal, cast

from wa_interfaces import srv as wa_srvs

if TYPE_CHECKING:
    import rclpy
    from rclpy.node import Node

QUEUE_HANDLER_PERIOD_S: Final[float] = 0.2
"""Period to call the queue handler in seconds."""

MAX_RETRIES: Final[int] = 5
"""Maximum number of retries for a Gazebo interaction."""


@dataclass
class GazeboActionHandles:
    """Handles for each Gazebo action."""

    spawn: Callable[[wa_srvs.Spawn.Request], rclpy.Future | None]
    """Spawn action handle."""

    destroy: Callable[[wa_srvs.Destroy.Request], rclpy.Future | None]
    """Destroy action handle."""

    attach: Callable[[wa_srvs.ToggleAttach.Request], rclpy.Future | None]
    """Attach action handle."""

    detach: Callable[[wa_srvs.ToggleAttach.Request], rclpy.Future | None]
    """Detach action handle."""


@dataclass
class GazeboQueueHandler:
    """Queue handler for Gazebo actions."""

    node: InitVar[Node]
    """Node to attach the handles and timer."""

    spawn: InitVar[Callable[[wa_srvs.Spawn.Request], rclpy.Future | None]]
    """Spawn action handle."""

    destroy: InitVar[Callable[[wa_srvs.Destroy.Request], rclpy.Future | None]]
    """Destroy action handle."""

    attach: InitVar[
        Callable[[wa_srvs.ToggleAttach.Request], rclpy.Future | None]
    ]
    """Attach action handle."""

    detach: InitVar[
        Callable[[wa_srvs.ToggleAttach.Request], rclpy.Future | None]
    ]
    """Detach action handle."""

    handles: GazeboActionHandles = field(init=False)
    """Handles for each type of a Gazebo action."""

    queue: list[
        tuple[Literal["spawn"], wa_srvs.Spawn.Request]
        | tuple[Literal["destroy"], wa_srvs.Destroy.Request]
        | tuple[Literal["attach"], wa_srvs.ToggleAttach.Request]
        | tuple[Literal["detach"], wa_srvs.ToggleAttach.Request]
    ] = field(init=False, default_factory=list)
    """Queue to handle Gazebo actions."""

    retries: int = field(init=False, default=0)
    """Current retries for a Gazebo interaction."""

    start_simulation: Callable[..., rclpy.Future | None]
    """Action to start the Gazebo simulation."""

    stop_simulation: Callable[..., rclpy.Future | None]
    """Action to stop the Gazebo simulation."""

    def __post_init__(
        self,
        node: Node,
        spawn: Callable[[wa_srvs.Spawn.Request], rclpy.Future | None],
        destroy: Callable[[wa_srvs.Destroy.Request], rclpy.Future | None],
        attach: Callable[[wa_srvs.ToggleAttach.Request], rclpy.Future | None],
        detach: Callable[[wa_srvs.ToggleAttach.Request], rclpy.Future | None],
    ) -> None:
        """Attach handles to the handles dataclass and create timer."""
        self.handles = GazeboActionHandles(spawn, destroy, attach, detach)
        node.create_timer(
            QUEUE_HANDLER_PERIOD_S,
            self.queue_callback,
        )

    def queue_callback(self) -> None:
        """Process the queue of Gazebo interactions."""
        if len(self.queue) == 0:
            # Nothing to process
            return

        # Pause simulation before handling the next action in the queue
        self.stop_simulation()
        future = None

        action, task = self.queue[0]

        # Spawn models
        if action == "spawn":
            task = cast(wa_srvs.Spawn.Request, task)
            future = self.handles.spawn(task)
        # Attach models
        elif action == "attach":
            task = cast(wa_srvs.ToggleAttach.Request, task)
            future = self.handles.attach(task)
        # Detach models
        elif action == "detach":
            task = cast(wa_srvs.ToggleAttach.Request, task)
            future = self.handles.detach(task)
        # Destroy models
        elif action == "destroy":
            task = cast(wa_srvs.Destroy.Request, task)
            future = self.handles.destroy(task)
        # Continue simulation
        if future is not None:
            future.add_done_callback(self.retry_if_needed_callback)
        else:
            self.start_simulation()

    def retry_if_needed_callback(self, future: rclpy.Future) -> None:
        """Retry Gazebo interaction if necessary."""
        if future.result().success or self.retries >= MAX_RETRIES:  # type: ignore[reportOptionalMemberAccess]
            self.retries = 0
            self.queue.pop(0)
            self.start_simulation()
            return
        # Try again
        self.retries += 1

    def enqueue_spawn_callback(
        self,
        request: wa_srvs.Spawn.Request,
        response: wa_srvs.Spawn.Response,
    ) -> wa_srvs.Spawn.Response:
        self.queue.append(("spawn", request))
        return response

    def enqueue_destroy_callback(
        self,
        request: wa_srvs.Destroy.Request,
        response: wa_srvs.Destroy.Response,
    ) -> wa_srvs.Destroy.Response:
        self.queue.append(("destroy", request))
        return response

    def enqueue_attach_callback(
        self,
        request: wa_srvs.ToggleAttach.Request,
        response: wa_srvs.ToggleAttach.Response,
    ) -> wa_srvs.ToggleAttach.Response:
        self.queue.append(("attach", request))
        return response

    def enqueue_detach_callback(
        self,
        request: wa_srvs.ToggleAttach.Request,
        response: wa_srvs.ToggleAttach.Response,
    ) -> wa_srvs.ToggleAttach.Response:
        self.queue.append(("detach", request))
        return response
