from __future__ import annotations

import contextlib
from dataclasses import dataclass
from typing import TYPE_CHECKING, Final, Literal, Self, TypeAlias

if TYPE_CHECKING:
    from wa_interfaces import msg as wa_msgs

    from wa_warehouse_control.utils.map import Model


task_uids = set[int]()
"""The list of known task uids."""

MIN_UID: Final[int] = 0
"""Minimal possible task uid."""

MAX_UID: Final[int] = 255
"""Maximal possible task uid."""

TaskType: TypeAlias = Literal["input", "output"]
"""Possible types of tasks."""


@dataclass
class Task:
    """A representation of a task.

    It should read task 'uid' is about
    carrying a box from 'start' to 'end'.
    """

    uid: int
    """The unique id of the task."""

    type_: TaskType
    """The type of the task."""

    start: Model
    """Where to pick the box."""

    end: Model
    """Where to place to the box."""

    box_id: int | None = None
    """The box id being carried in the task."""

    def __post_init__(self) -> None:
        """Add the uid to the list of known task_uids."""
        if self.uid < MIN_UID or self.uid > MAX_UID:
            raise ValueError(
                f"Task id should be between {MIN_UID} and {MAX_UID}",
            )
        if self.uid in task_uids:
            raise ValueError(f"Task id '{self.uid}' in use")
        task_uids.add(self.uid)

    @classmethod
    def create_task(
        cls,
        type_: TaskType,
        start: Model,
        end: Model,
        box_id: int | None = None,
    ) -> Self:
        """Create a task with an unique id."""
        uid = cls.generate_uid()
        return cls(uid, type_, start, end, box_id)

    @staticmethod
    def generate_uid() -> int:
        """Generate an unique id for the task."""
        if len(task_uids) == 0:
            return MIN_UID

        # Bigger uid
        id_ = max(task_uids) + 1
        if id_ < MAX_UID:
            return id_

        # Find the first gap
        for id_ in range(MIN_UID, MAX_UID + 1):
            if id_ not in task_uids:
                return id_

        raise RuntimeError("No task id left")

    def __del__(self) -> None:
        """Cleanup task uid."""
        with contextlib.suppress(KeyError):
            task_uids.remove(self.uid)


@dataclass
class TaskStats:
    active: int
    done: int
    idle: int
    active_time: list[float]
    done_time: list[float]
    idle_time: list[float]
