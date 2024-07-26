from __future__ import annotations

from math import atan2, cos, sin
from typing import TYPE_CHECKING, ClassVar, Final, Literal, cast

import rclpy
from geometry_msgs import msg as geometry_msgs
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs import msg as nav_msgs
from rclpy.node import Node
from std_msgs import msg as std_msgs
from wa_interfaces import msg as wa_msgs
from wa_interfaces import srv as wa_srvs

if TYPE_CHECKING:
    from rclpy.client import Client

NAVIGATION_FRAME_ID: Final[str] = "map"
"""Relative frame for the automatic navigation."""

CLOSE_RADIUS: Final[float] = 0.5
"""Radius to consider the robot able to pick up a box."""


class Mobilebot(Node):
    """A mobile robot in the warehouse automation project.

    The mobile robots are the entities responsible for moving the
    boxes (the tasks) between startpoints and endpoints. This node
    is responsible for accepting tasks and managing the robot.
    """

    name: ClassVar[str] = "mobilebot"
    """Node name."""

    confirmation: Client
    """Task confirmation client."""

    box_transfer: Client
    """Box transfer client (pick & drop box)."""

    pose: geometry_msgs.Pose
    """The mobilebot's current pose."""

    task: wa_msgs.Task | None
    """The task the robot is currently executing."""

    navigator: BasicNavigator
    """Robot automatic navigation."""

    # TODO: automaton
    goal_status: Literal["idle", "to_start", "to_end"]
    """Current goal status."""

    def __init__(self, goal_check_period: float = 0.5) -> None:
        super().__init__(self.name)  # type: ignore[reportArgumentType]

        # Parameters
        self.declare_parameter("goal_check_period", goal_check_period)
        # Initialize robot tracker
        self.pose = geometry_msgs.Pose()
        # TODO: fix hardcode namespace
        self.navigator = BasicNavigator(namespace="/wa/mobilebot_1")
        # TODO: set initial pose
        self.navigator.waitUntilNav2Active()
        self.task = None
        self.goal_status = "idle"

        # Timers
        self.create_timer(
            self.goal_check_period,
            self.goal_check_callback,
        )

        # Subscriptions
        self.create_subscription(
            std_msgs.UInt8,
            "/wa/task_transmitter/broadcast",
            self.broadcast_callback,
            1,
        )
        self.create_subscription(
            nav_msgs.Odometry,
            # TODO: fix hardcode namespace
            "/wa/mobilebot_1/odom",
            self.pose_tracker_callback,
            10,
        )

        # Clients
        self.confirmation = self.create_client(
            wa_srvs.TaskConfirmation,
            "/wa/task_transmitter/task_confirmation",
        )
        self.box_transfer = self.create_client(
            wa_srvs.BoxTransfer,
            "/wa/box/transfer",
        )

        # Outputs
        self.task_started = self.create_publisher(
            std_msgs.UInt8,
            f"/wa/{self.get_name()}/task/started",
            10,
        )
        self.task_completed = self.create_publisher(
            std_msgs.UInt8,
            f"/wa/{self.get_name()}/task/completed",
            10,
        )

        while not self.confirmation.wait_for_service(5.0):
            self.get_logger().info("Waiting for task_transmitter services")

    @property
    def goal_check_period(self) -> float:
        """Goal check period of the robot."""
        return self.get_parameter("goal_check_period").value  # type: ignore[reportIncompatibleType]

    def pose_tracker_callback(self, message: nav_msgs.Odometry) -> None:
        """Track the robot's pose from its odometry."""
        self.pose = message.pose.pose
        self.get_logger().debug(f"Robot pose: {self.pose}")

    def broadcast_callback(self, message: std_msgs.UInt8) -> None:
        """Handle broadcast of a task, may request it or not."""
        if self.task is None:
            task_uid = message.data
            self.get_logger().info(f"Requesting task {task_uid}")
            self.confirmation.call_async(
                wa_srvs.TaskConfirmation.Request(
                    task_uid=message.data,
                    robot=self.get_name(),
                ),
            ).add_done_callback(self.handle_task_confirmation)

    def handle_task_confirmation(self, future: rclpy.Future) -> None:
        """Handle the confirmation response of a task."""
        result = cast(wa_srvs.TaskConfirmation.Response, future.result())
        if not result.confirmation:
            self.get_logger().info(f"Task {result.task.uid} denied")
            return

        self.get_logger().info(f"Task {result.task.uid} confirmed")

        # Save task
        self.task = result.task
        self.task_started.publish(std_msgs.UInt8(data=self.task.uid))

        # HACK: fix conveyor belt position (so that no collision happens)
        if self.task.start.name.startswith("conveyor_belt"):
            self.task.start.pose.position.y = round(
                self.task.start.pose.position.y + 0.8,
                ndigits=2,
            )
        if self.task.end.name.startswith("conveyor_belt"):
            self.task.end.pose.position.y = round(
                self.task.end.pose.position.y - 0.8,
                ndigits=2,
            )

    def goal_check_callback(self) -> None:
        """Check if the robot has reached the goal."""
        if self.task is None:
            return

        if self.goal_status == "idle":
            self.goal_status = "to_start"
            self.go_to(
                self.task.start.pose.position.x,
                self.task.start.pose.position.y,
            )
            return

        if self.goal_status == "to_start" and self.is_close(
            self.task.start.pose.position.x,
            self.task.start.pose.position.y,
        ):
            self.goal_status = "to_end"
            self.pick_box(self.task.start.name).add_done_callback(
                lambda _: self.go_to(
                    self.task.end.pose.position.x,  # type: ignore[reportAttributeAccessIssue]
                    self.task.end.pose.position.y,  # type: ignore[reportAttributeAccessIssue]
                ),
            )
        elif self.goal_status == "to_end" and self.is_close(
            self.task.end.pose.position.x,
            self.task.end.pose.position.y,
        ):
            self.goal_status = "to_end"
            self.drop_box(self.task.end.name)
            self.task = None

    def go_to(self, x: float, y: float) -> None:
        """Move the robot to a specific pose."""
        self.get_logger().info(
            f"Moving to (x: {x}, y: {y})",
        )
        # Face the goal
        yaw = atan2(y - self.pose.position.y, x - self.pose.position.x)
        self.navigator.goToPose(
            geometry_msgs.PoseStamped(
                header=std_msgs.Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=NAVIGATION_FRAME_ID,
                ),
                pose=geometry_msgs.Pose(
                    position=geometry_msgs.Point(
                        x=x,
                        y=y,
                    ),
                    # Quarternion from yaw
                    orientation=geometry_msgs.Quaternion(
                        x=0.0,
                        y=0.0,
                        z=sin(yaw / 2),
                        w=cos(yaw / 2),
                    ),
                ),
            ),
        )

    def is_close(self, x: float, y: float) -> bool:
        """Return whether the robot is close to a point."""
        return (self.pose.position.x - x) ** 2 + (
            self.pose.position.y - y
        ) ** 2 <= CLOSE_RADIUS**2

    def pick_box(self, from_entity: str) -> rclpy.Future:
        """Pick a box from an entity."""
        if self.task is None:
            raise RuntimeError("Can't pick a box without any task")

        return self.box_transfer.call_async(
            wa_srvs.BoxTransfer.Request(
                box_id=self.task.box_id,
                from_entity=from_entity,
                # TODO: use name
                to_entity="mobilebot_1",
            ),
        )

    def drop_box(self, to_entity: str) -> rclpy.Future:
        """Drop a box on an entity."""
        if self.task is None:
            raise RuntimeError("Can't drop a box without any task")

        return self.box_transfer.call_async(
            wa_srvs.BoxTransfer.Request(
                box_id=self.task.box_id,
                # TODO: use name
                from_entity="mobilebot_1",
                to_entity=to_entity,
            ),
        )


def main() -> None:
    """Run the mobilebot."""
    rclpy.init()
    node = Mobilebot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
