from __future__ import annotations

import random
from typing import TYPE_CHECKING, Callable

from rclpy.timer import Timer

if TYPE_CHECKING:
    from rclpy.callback_groups import CallbackGroup
    from rclpy.clock import Clock
    from rclpy.node import Node


class VariableTimer(Timer):
    def __init__(  # noqa: PLR0913
        self,
        node: Node,
        min_interval_sec: float,
        max_interval_sec: float,
        callback: Callable,
        callback_group: CallbackGroup | None = None,
        clock: Clock | None = None,
    ) -> None:
        self._node = node
        self._min_interval_sec = min_interval_sec
        self._max_interval_sec = max_interval_sec
        self._callback = callback

        self._gen_next_callback_time()
        self._register_timer(callback_group, clock)

    def _gen_next_callback_time(self) -> None:
        time = self._node.get_clock().now().nanoseconds
        interval = sec_to_nsec(
            get_random_interval(
                self._min_interval_sec,
                self._max_interval_sec,
            ),
        )

        self._next_callback_time = time + interval

    def _register_timer(
        self,
        callback_group: CallbackGroup | None,
        clock: Clock | None,
    ) -> None:
        if callback_group is None:
            callback_group = self._node.default_callback_group
        if clock is None:
            clock = self._node._clock

        super().__init__(
            self._wrapped_callback,
            callback_group,
            sec_to_nsec(self._min_interval_sec),
            clock,
            context=self._node.context,
        )

        callback_group.add_entity(self)
        self._node._timers.append(self)
        self._node._wake_executor()

    def _wrapped_callback(self):
        current_time = self._node.get_clock().now().nanoseconds
        if current_time >= self._next_callback_time:
            self._callback()
            self._gen_next_callback_time()


def sec_to_nsec(sec: float) -> int:
    return int(sec * 1e9)


def get_random_interval(min_: float, max_: float) -> float:
    num = random.gauss(
        mu=(max_ + min_) / 2,
        sigma=(max_ - min_) / 4,
    )

    match (min_ <= num, num <= max_):
        case (True, True):
            return num
        case (False, _):
            return min_
        case (_, False):
            return max_

    raise ValueError("Unreachable")
