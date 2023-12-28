from __future__ import annotations

from abc import ABC
from enum import Enum, auto

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

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
        self.goal = None
        # self._navigator = BasicNavigator(namespace=self.name)
        # self._navigator.setInitialPose(PoseStamped(pose=self._pose))
        # self._navigator.lifecycleStartup()

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
        if self.is_on():
            raise ValueError("This robot is already on")

        self.activate()
        self._state = RobotState.Idle

    def shutdown(self) -> None:
        """Shutdown this robot."""
        if not self.is_on():
            raise ValueError("This robot is already off")

        self.deactivate()
        self._state = RobotState.Off

    def is_on(self) -> bool:
        """Return whether the robot is on."""
        return self._state != RobotState.Off

    def set_idle(self) -> None:
        """Set state to indicate that the robot is idle."""
        self._state = RobotState.Idle

    def is_idle(self) -> bool:
        """Return whether the robot is idle."""
        return self._state == RobotState.Idle

    def active_state(self) -> None:
        """Set state to indicate that the robot is active."""
        self._state = RobotState.Active

    def is_active(self) -> bool:
        """Return whether the robot is active."""
        return self._state == RobotState.Active

    def set_goal(self, x: float, y: float) -> None:
        """Set the goal for this robot."""
        position = Point(x=x, y=y, z=0.0)
        self.goal = Pose(position=position)

    def move_to_goal(self) -> None:
        """Move the robot to its goal."""
        if self.goal is None:
            raise ValueError("This robot has no goal")
        if not self.is_on():
            raise ValueError("This robot is off")
        self.active_state()
        # self._navigator.goToPose(PoseStamped(pose=pose))

    def has_arrived(self) -> bool:
        """Return whether the robot has arrived at its destination."""
        if self.goal is None:
            raise ValueError("This robot has no goal")

        if round(self.position.x, 2) == round(
            self.goal.position.x,
            2,
        ) and round(self.position.y, 2) == round(self.goal.position.y, 2):
            self.goal = None
            self.set_idle()
            return True
        return False
        # match self._navigator.getResult():
        #     case TaskResult.SUCCEEDED:
        #         self.goal = None
        #         return True
        #     case TaskResult.FAILED | TaskResult.CANCELED | TaskResult.UNKNOWN:
        #         self.goal = None
        #         return False

    def get_navigation_feedback(self) -> str:
        """Get the navigation feedback."""
        if self.goal is None:
            raise ValueError("This robot has no goal")

        # Simulate movement
        dx = 0.5 * (self.goal.position.x - self.position.x)
        if -0.2 < dx < 0.2:  # noqa: PLR2004
            dx = self.goal.position.x - self.position.x
        self.position.x += dx
        dy = 0.5 * (self.goal.position.y - self.position.y)
        if -0.2 < dy < 0.2:  # noqa: PLR2004
            dy = self.goal.position.y - self.position.y
        self.position.y += dy

        return (
            "Position: "
            f"({round(self.position.x, 2)}, {round(self.position.y, 2)}) "
            "Goal: "
            "("
            f"{round(self.goal.position.x, 2)}, "
            f"{round(self.goal.position.y, 2)}"
            ")"
        )
        # if self._navigator.isTaskComplete():
        #     raise ValueError("Navigation task is complete")
        # return self._navigator.getFeedback()  # type: ignore
