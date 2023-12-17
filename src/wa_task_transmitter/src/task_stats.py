from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from src.uids import UIDIndexed

if TYPE_CHECKING:
    from std_msgs.msg import Point


@dataclass
class Task(UIDIndexed):
    start: Point
    end: Point

    def __post_init__(self) -> None:
        super().__init__()


@dataclass
class TaskStats:
    active: int
    done: int
    idle: int
    active_time: list[float]
    done_time: list[float]
    idle_time: list[float]
