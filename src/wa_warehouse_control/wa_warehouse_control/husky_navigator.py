from __future__ import annotations

from enum import Enum, auto
from random import random, seed
from typing import TYPE_CHECKING

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

if TYPE_CHECKING:
    from rclpy.client import Future


class State(Enum):
    FRONT = auto()
    LEFT = auto()
    BACK = auto()
    RIGHT = auto()

    def next(self) -> State:
        match self:
            case State.FRONT:
                return State.LEFT
            case State.LEFT:
                return State.BACK
            case State.BACK:
                return State.RIGHT
            case State.RIGHT:
                return State.FRONT


class HuskyNavigator(Node):
    def __init__(self) -> None:
        super().__init__("husky_navigator")
        self.state = State.FRONT

        # Timer action
        self.create_timer(2.0, self.timer_callback)

        # Outputs
        self.navigation_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

    def timer_callback(self) -> None:
        msg = Twist()
        match self.state:
            case State.FRONT:
                msg.linear.x = 1.0
            case State.LEFT:
                msg.angular.z = 1.0
            case State.BACK:
                msg.linear.x = -1.0
            case State.RIGHT:
                msg.angular.z = -1.0
        self.navigation_pub.publish(msg)
        self.state = self.state.next()


def main():
    rclpy.init()
    node = HuskyNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
