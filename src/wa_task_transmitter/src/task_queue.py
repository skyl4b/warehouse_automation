from __future__ import annotations

from math import inf
from typing import Generic, TypeVar

T = TypeVar("T")

INFINITE = inf


class TaskQueue(Generic[T]):
    def __init__(self, size=INFINITE) -> None:
        self.size = size
        self._inner = []

    def put(self, item: T) -> None:
        if len(self._inner) >= self.size:
            raise IndexError("Task queue is full")
        self._inner.append(item)

    def get(self) -> T | None:
        try:
            return self._inner[0]
        except IndexError:
            return None

    def full(self) -> bool:
        return len(self._inner) >= self.size

    def done(self) -> None:
        self._inner.pop(0)
