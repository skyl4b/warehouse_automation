"""Queue for batch processing of Gazebo interactions."""

from __future__ import annotations

from dataclasses import InitVar, dataclass, field
from queue import Empty, Queue
from typing import TYPE_CHECKING, Callable, Final

if TYPE_CHECKING:
    import rclpy
    from rclpy.node import Node
    from wa_interfaces import srv as wa_srvs

QUEUE_HANDLER_PERIOD_S: Final[float] = 0.2
"""Period to call the queue handler in seconds."""

MAX_QUEUE_SIZE: Final[int] = 20
"""Max number of tasks in the queue of Gazebo interactions."""


@dataclass
class GazeboQueues:
    """Queues for the queue handler to operate."""

    spawner: Queue[wa_srvs.Spawn.Request] = field(
        default_factory=lambda: Queue(MAX_QUEUE_SIZE),
    )
    """Queue for the periodic spawn."""

    spawn_counter: int = 0
    """Counter for when spawning is done."""

    destroyer: Queue[wa_srvs.Destroy.Request] = field(
        default_factory=lambda: Queue(MAX_QUEUE_SIZE),
    )
    """Queue for the periodic destroy."""

    attacher: Queue[wa_srvs.ToggleAttach.Request] = field(
        default_factory=lambda: Queue(MAX_QUEUE_SIZE),
    )
    """Queue for the periodic attach."""

    detacher: Queue[wa_srvs.ToggleAttach.Request] = field(
        default_factory=lambda: Queue(MAX_QUEUE_SIZE),
    )
    """Queue for the periodic detach."""

    detach_counter: int = 0
    """Counter for when attaching is done."""

    def empty(self) -> bool:
        """Check if all GazeboBridge queues are empty."""
        return all(
            queue.empty()
            for key, queue in self.__dict__.items()
            if not key.endswith("counter")
        )

    def increase_spawn_counter(self) -> None:
        """Increase the spawn counter."""
        self.spawn_counter += 1

    def decrease_spawn_counter(self) -> None:
        """Decrease the spawn counter."""
        self.spawn_counter = max(self.spawn_counter - 1, 0)

    def increase_detach_counter(self) -> None:
        """Increase the detach counter."""
        self.detach_counter += 1

    def decrease_detach_counter(self) -> None:
        """Decrease the detach counter."""
        self.detach_counter = max(self.detach_counter - 1, 0)


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

    queue: GazeboQueues = field(init=False)
    """Quees for each handle of a Gazebo action."""

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
        self.queue = GazeboQueues()

    def queue_callback(self) -> None:  # noqa: PLR0912
        """Process the queue of Gazebo interactions."""
        if self.queue.empty():
            # Nothing to process
            return

        # Pause simulation before handling the queue
        self.stop_simulation()
        future = None

        # Spawn models
        if not self.queue.spawner.empty():
            try:
                task = self.queue.spawner.get_nowait()
                future = self.handles.spawn(task)
                if future is not None:
                    future.add_done_callback(
                        lambda _: self.queue.decrease_spawn_counter(),
                    )
            except Empty:
                pass
        # Attach models
        elif (
            not self.queue.attacher.empty()
            and self.queue.spawn_counter == 0
            and self.queue.detach_counter == 0
        ):
            try:
                while True:
                    task = self.queue.attacher.get_nowait()
                    future = self.handles.attach(task)
            except Empty:
                pass
        # Detach models
        elif not self.queue.detacher.empty():
            try:
                while True:
                    task = self.queue.detacher.get_nowait()
                    future = self.handles.detach(task)
                    if future is not None:
                        future.add_done_callback(
                            lambda _: self.queue.decrease_detach_counter(),
                        )
            except Empty:
                pass
        # Destroy models
        elif (
            not self.queue.destroyer.empty() and self.queue.detach_counter == 0
        ):
            try:
                while True:
                    task = self.queue.destroyer.get_nowait()
                    future = self.handles.destroy(task)
            except Empty:
                pass
        # Continue simulation
        if future is not None:
            future.add_done_callback(lambda _: self.start_simulation())
        else:
            self.start_simulation()

    def enqueue_spawn_callback(
        self,
        request: wa_srvs.Spawn.Request,
        response: wa_srvs.Spawn.Response,
    ) -> wa_srvs.Spawn.Response:
        self.queue.spawner.put(request)
        self.queue.increase_spawn_counter()
        return response

    def enqueue_destroy_callback(
        self,
        request: wa_srvs.Destroy.Request,
        response: wa_srvs.Destroy.Response,
    ) -> wa_srvs.Destroy.Response:
        self.queue.destroyer.put(request)
        return response

    def enqueue_attach_callback(
        self,
        request: wa_srvs.ToggleAttach.Request,
        response: wa_srvs.ToggleAttach.Response,
    ) -> wa_srvs.ToggleAttach.Response:
        self.queue.attacher.put(request)
        return response

    def enqueue_detach_callback(
        self,
        request: wa_srvs.ToggleAttach.Request,
        response: wa_srvs.ToggleAttach.Response,
    ) -> wa_srvs.ToggleAttach.Response:
        self.queue.detacher.put(request)
        self.queue.increase_detach_counter()
        return response
