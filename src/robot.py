from __future__ import annotations

from abc import ABC
from enum import Enum, auto

from geometry_msgs.msg import Point, Pose, Quaternion
from id_handler import IDHandler
from rclpy.node import Node


class RobotState(Enum):
    Idle = auto()
    Active = auto()
    Off = auto()


class Robot(Node, ABC):
    _id_handler = IDHandler()

    @classmethod
    def get_active_ids(cls) -> tuple[int, ...]:
        """Get the list of current active bot ids."""
        return cls._id_handler.active_ids

    def __init__(self, bot: str) -> None:
        self._r_id = self._id_handler.gen_id()
        super().__init__(node_name=f"{bot}_{self.id}")
        self._name = bot
        self._state = RobotState.Idle
        self._pose = Pose()

    @property
    def r_id(self) -> int:
        """Get this robot's id."""
        return self._r_id

    @property
    def name(self) -> str:
        """Get this robot's name."""
        return self._name

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
        self._id_handler.deactivate_id(self.id)
        self._state = RobotState.Off

    def shutdown(self) -> None:
        """Shutdown this robot."""
        self._id_handler.deactivate_id(self.id)
        self._state = RobotState.Off
