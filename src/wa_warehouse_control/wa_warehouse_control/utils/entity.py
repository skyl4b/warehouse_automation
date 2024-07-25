from __future__ import annotations

import random
from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.time import Duration

from wa_warehouse_control.utils.uids import UIDIndexed


class Entity(Node, UIDIndexed, ABC):  # type: ignore[reportUnsafeMultipleInheritance]
    def __init__(self) -> None:
        # Initialize superclasses in the correct order
        ABC.__init__(self)
        UIDIndexed.__init__(self)
        self.name = f"{self.basename()}_{self.uid}"
        Node.__init__(self, self.name)  # type: ignore[reportArgumentType]

    @classmethod
    @abstractmethod
    def basename(cls) -> str:
        """Define the basename of the entity."""


class ActionEntity(Entity, ABC):
    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def action_callback(self) -> None:
        pass


class TickActionEntity(ActionEntity, ABC):
    @property
    @abstractmethod
    def action_period(self) -> float:
        raise NotImplementedError("The inherited class must define the period")

    def start_action_timer(self) -> None:
        self._action_timer = self.create_timer(
            self.action_period,
            self.action_callback,
        )

    def stop_actiont_timer(self) -> None:
        self._action_timer.destroy()


class VariableTickActionEntity(ActionEntity, ABC):
    def start_action_timer(self) -> None:
        self._action_timer = self.create_timer(
            self.action_min_period,
            self.wrapped_callback,
        )
        self.gen_next_callback_time()

    def stop_action_timer(self) -> None:
        self._action_timer.destroy()

    @property
    @abstractmethod
    def action_min_period(self) -> float:
        raise NotImplementedError("The inherited class must define the period")

    @property
    @abstractmethod
    def action_max_period(self) -> float:
        raise NotImplementedError("The inherited class must define the period")

    def wrapped_callback(self) -> None:
        """Call callback on a random timer."""
        if self.get_clock().now() >= self._next_callback_time:
            self.action_callback()
            self.gen_next_callback_time()

    def gen_next_callback_time(self) -> None:
        """Generate next random callback time."""
        global previous_generated_time  # noqa: PLW0603
        time = self.get_clock().now()
        if previous_generated_time is not None:
            time = max(time, previous_generated_time)

        interval = Duration(
            seconds=random.uniform(  # type: ignore[reportArgumentType]
                self.action_min_period,
                self.action_max_period,
            ),
        )
        self._next_callback_time = time + interval
        previous_generated_time = self._next_callback_time


previous_generated_time = None
