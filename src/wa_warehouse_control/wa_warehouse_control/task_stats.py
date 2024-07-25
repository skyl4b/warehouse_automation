from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from wa_warehouse_control.utils.uids import UIDIndexed

if TYPE_CHECKING:
    from wa_warehouse_control.utils.map import Position


@dataclass
class Task(UIDIndexed):
    start: Position
    end: Position

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
