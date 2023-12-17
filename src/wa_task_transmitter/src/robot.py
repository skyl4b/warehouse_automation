from __future__ import annotations

from abc import ABC
from enum import Enum, auto

from geometry_msgs.msg import Point, Pose, Quaternion

from src.entity import TickActionEntity


class RobotState(Enum):
    Idle = auto()
    Active = auto()
    Off = auto()


class Robot(TickActionEntity, ABC):
    def __init__(self, action_period: float) -> None:
        super().__init__(action_period)

        self._state = RobotState.Idle
        self._pose = Pose()

    @property
    def state(self) -> RobotState:
        """Get this robot's state."""
        return self._state

    @property
    def pose(self) -> Pose:
        """Get the robot's pose."""
        return self._pose

    @property
    def position(self) -> Point:
        """Get the robot's position."""
        return self._pose.position

    @property
    def orientation(self) -> Quaternion:
        """Get the robot's orientation."""
        return self._pose.orientation

    def turn_on(self) -> None:
        """Turn on this robot."""
        self.activate()
        self._state = RobotState.Idle

    def shutdown(self) -> None:
        """Shutdown this robot."""
        self.deactivate()
        self._state = RobotState.Off
