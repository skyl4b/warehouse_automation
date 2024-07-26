from __future__ import annotations

import contextlib
from dataclasses import dataclass
from typing import TYPE_CHECKING, Final, Self

if TYPE_CHECKING:
    from wa_warehouse_control.utils.map import Model
    from wa_interfaces import msg as wa_msgs


task_uids = set[int]()
"""The list of known task uids."""

MIN_UID: Final[int] = 0
"""Minimal possible task uid."""

MAX_UID: Final[int] = 255
"""Maximal possible task uid."""


@dataclass
class Task:
    """A representation of a task.

    It should read task 'uid' is about
    carrying a box from 'start' to 'end'.
    """

    uid: int
    """The unique id of the task."""

    start: Model
    """Where to pick the box."""

    end: Model
    """Where to place to the box."""

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
    def create_task(cls, start: Model, end: Model) -> Self:
        """Create a task with an unique id."""
        uid = cls.generate_uid()
        return cls(uid, start, end)

    @staticmethod
    def generate_uid() -> int:
        """Generate an unique id for the task."""
        # Find the first gap
        for id_ in range(MIN_UID, MAX_UID + 1):
            if id_ not in task_uids:
                return id_

        raise RuntimeError("No task id left")

    @classmethod
    def from_task_message(cls, task: wa_msgs.Task) -> Self:
        """Generate a task from a task message."""
        return cls(
            uid=task.uid,
            start={
                "name": task.start.name,
                "position": {
                    "x": task.start.pose.position.x,
                    "y": task.start.pose.position.y,
                    "z": task.start.pose.position.z,
                },
            },
            end={
                "name": task.end.name,
                "position": {
                    "x": task.end.pose.position.x,
                    "y": task.end.pose.position.y,
                    "z": task.end.pose.position.z,
                },
            },
        )

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
