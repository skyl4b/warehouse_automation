from __future__ import annotations

from abc import ABC, abstractmethod

from rclpy.node import Node

from src.uids import UIDIndexed
from src.variable_timer import VariableTimer


class Entity(Node, UIDIndexed, ABC):
    def __init__(self) -> None:
        # Initialize superclasses in the correct order
        ABC.__init__(self)
        UIDIndexed.__init__(self)
        self.name = f"{self.basename()}_{self.uid}"
        Node.__init__(self, self.name)  # type: ignore

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
    def __init__(self, action_period: float) -> None:
        super().__init__()

        self._action_timer = self.create_timer(
            action_period,
            self.action_callback,
        )


class VariableTickActionEntity(ActionEntity, ABC):
    def __init__(
        self,
        action_min_period: float,
        action_max_period: float,
    ) -> None:
        super().__init__()

        self._action_timer = VariableTimer(
            self,
            action_min_period,
            action_max_period,
            self.action_callback,
        )
