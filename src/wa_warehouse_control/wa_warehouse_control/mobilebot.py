from __future__ import annotations

from math import atan2, cos, sin
from typing import TYPE_CHECKING, ClassVar, Final, Literal, cast

import rclpy
from geometry_msgs import msg as geometry_msgs
from nav_msgs import msg as nav_msgs
from rcl_interfaces import msg as rcl_msgs
from rclpy.node import Node
from std_msgs import msg as std_msgs
from wa_interfaces import msg as wa_msgs
from wa_interfaces import srv as wa_srvs

from wa_warehouse_control.utils.navigator import Navigator

if TYPE_CHECKING:
    from rclpy.client import Client
    from rclpy.timer import Timer

NAVIGATION_FRAME_ID: Final[str] = "map"
"""Relative frame for the automatic navigation."""

CLOSE_RADIUS: Final[float] = 0.5
"""Radius to consider the robot able to pick up a box."""

CONVEYOR_BELT_Y_DELTA: Final[float] = 0.8
"""Delta for the pick up point of a conveyor belt."""


class Mobilebot(Node):
    """A mobile robot in the warehouse automation project.

    The mobile robots are the entities responsible for moving the
    boxes (the tasks) between startpoints and endpoints. This node
    is responsible for accepting tasks and managing the robot.
    """

    name: ClassVar[str] = "mobilebot_1"
    """Default node name."""

    namespace: ClassVar[str] = "/wa"
    """Default node namespace."""

    confirmation: Client
    """Task confirmation client."""

    box_transfer: Client
    """Box transfer client (pick & drop box)."""

    pose: geometry_msgs.Pose
    """The mobilebot's current pose."""

    task: wa_msgs.Task | None
    """The task the robot is currently executing."""

    navigator: Navigator
    """Robot automatic navigation."""

    # TODO: automaton
    goal_status: Literal["idle", "to_start", "to_end", "pick_box", "drop_box"]
    """Current goal status."""

    goal_check_timer: Timer
    """Timer for checking if the robot reached the goal."""

    def __init__(
        self,
        goal_check_period: float = 0.2,
        initial_position: tuple[float, float] = (0.0, 0.0),
        close_radius: float = CLOSE_RADIUS,
    ) -> None:
        super().__init__(  # type: ignore[reportArgumentType]
            node_name=self.name,
            namespace=self.namespace,
        )

        # Parameters
        self.declare_parameter("goal_check_period", goal_check_period)
        self.declare_parameter("initial_position", initial_position)
        self.declare_parameter("close_radius", close_radius)
        self.create_subscription(  # Monitor set parameter
            rcl_msgs.ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            10,
        )

        # Clients
        self.confirmation = self.create_client(
            wa_srvs.TaskConfirmation,
            "task_transmitter/task_confirmation",
        )
        self.box_transfer = self.create_client(
            wa_srvs.BoxTransfer,
            "box/transfer",
        )
        while not (
            self.confirmation.wait_for_service(5.0)
            and self.box_transfer.wait_for_service(5.0)
        ):
            self.get_logger().info(
                "Waiting for task_transmitter and box_order services",
            )

        # Initialize robot tracker
        self.pose = geometry_msgs.Pose()
        self.task = None
        self.goal_status = "idle"

        # Robot navigator
        self.navigator = Navigator(namespace=self.get_fully_qualified_name())
        self.set_initial_position(self.initial_position)
        self.navigator.waitUntilNav2Active()

        # Timers
        self.goal_check_timer = self.create_timer(
            self.goal_check_period,
            self.goal_check_callback,
        )

        # Subscriptions
        self.create_subscription(
            std_msgs.UInt8,
            "task_transmitter/broadcast",
            self.broadcast_callback,
            1,
        )
        self.create_subscription(
            nav_msgs.Odometry,
            f"{self.get_name()}/odom",
            self.pose_tracker_callback,
            10,
        )

        # Outputs
        self.task_started = self.create_publisher(
            std_msgs.UInt8,
            "task/started",
            10,
        )
        self.task_midpoint = self.create_publisher(
            std_msgs.UInt8,
            "task/midpoint",
            10,
        )
        self.task_completed = self.create_publisher(
            std_msgs.UInt8,
            "task/completed",
            10,
        )

    @property
    def goal_check_period(self) -> float:
        """Goal check period of the robot."""
        return self.get_parameter("goal_check_period").value  # type: ignore[reportIncompatibleType]

    @property
    def initial_position(self) -> tuple[float, float]:
        """Initial position of the robot."""
        return self.get_parameter("initial_position").value  # type: ignore[reportIncompatibleType]

    @property
    def close_radius(self) -> float:
        """How close must the robot get to the goal."""
        return self.get_parameter("close_radius").value  # type: ignore[reportIncompatibleType]

    def parameter_event_callback(
        self,
        message: rcl_msgs.ParameterEvent,
    ) -> None:
        """Monitor changes to this node's parameters for timer / position."""
        if message.node == self.get_fully_qualified_name():
            for parameter in message.changed_parameters:
                if parameter.name == "goal_check_period":
                    # Regenerate goal_check timer
                    self.get_logger().info(
                        "Updating timer to "
                        f"goal_check_period {self.goal_check_period}",
                    )
                    self.goal_check_timer.destroy()
                    self.goal_check_timer = self.create_timer(
                        self.goal_check_period,
                        self.goal_check_callback,
                    )
                elif parameter.name == "initial_position":
                    # Update initial position
                    self.get_logger().info(
                        f"Updating initial position to "
                        f"{self.initial_position[:2]}",
                    )
                    self.set_initial_position(self.initial_position[:2])

    def set_initial_position(
        self,
        initial_position: tuple[float, float],
    ) -> None:
        """Set the robot's initial position."""
        x, y = initial_position
        self.navigator.setInitialPose(
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
                ),
            ),
        )

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
                self.task.start.pose.position.y + CONVEYOR_BELT_Y_DELTA,
                ndigits=2,
            )
        if self.task.end.name.startswith("conveyor_belt"):
            self.task.end.pose.position.y = round(
                self.task.end.pose.position.y - CONVEYOR_BELT_Y_DELTA,
                ndigits=2,
            )

    def goal_check_callback(self) -> None:
        """Check if the robot has reached the goal."""
        if self.task is None:
            # Return to idle position on timer
            # if not self.is_close(0.0, 0.0, close_radius=2.0):
            #     self.go_to(0.0, 0.0)
            return

        if self.goal_status == "idle":
            self.goal_status = "to_start"
            self.task_started.publish(std_msgs.UInt8(data=self.task.uid))
            self.go_to(
                self.task.start.pose.position.x,
                self.task.start.pose.position.y,
            )
            return

        if self.goal_status == "to_start":
            if not self.is_close(
                self.task.start.pose.position.x,
                self.task.start.pose.position.y,
            ):
                # Prevent robot getting lost during navigation
                if self.navigator.isTaskComplete():
                    # Try again
                    self.get_logger().warn(
                        "Robot got lost going to "
                        "("
                        f"x: {self.task.start.pose.position.x}, "
                        f"y: {self.task.start.pose.position.y}"
                        ")",
                    )
                    self.go_to(
                        self.task.start.pose.position.x,
                        self.task.start.pose.position.y,
                    )
            else:

                def done_callback(_: rclpy.Future) -> None:
                    """Move to second goal."""
                    if self.task is None:
                        self.get_logger().error(
                            "No task to set midpoint, something went wrong",
                        )
                        return
                    self.task_midpoint.publish(
                        std_msgs.UInt8(data=self.task.uid),
                    )
                    self.go_to(
                        self.task.end.pose.position.x,
                        self.task.end.pose.position.y,
                    )
                    self.goal_status = "to_end"

                self.goal_status = "pick_box"
                self.pick_box(self.task.start.name).add_done_callback(
                    done_callback,
                )
        elif self.goal_status == "to_end":
            if not self.is_close(
                self.task.end.pose.position.x,
                self.task.end.pose.position.y,
            ):
                # Prevent robot getting lost during navigation
                if self.navigator.isTaskComplete():
                    # Try again
                    self.get_logger().warn(
                        "Robot got lost going to "
                        "("
                        f"x: {self.task.end.pose.position.x}, "
                        f"y: {self.task.end.pose.position.y}"
                        ")",
                    )
                    self.go_to(
                        self.task.end.pose.position.x,
                        self.task.end.pose.position.y,
                    )
            else:

                def done_callback(_: rclpy.Future) -> None:
                    """Idle goal."""
                    if self.task is None:
                        self.get_logger().error(
                            "No task to set complete, something went wrong",
                        )
                        return
                    self.task_completed.publish(
                        std_msgs.UInt8(data=self.task.uid),
                    )
                    self.goal_status = "idle"
                    self.task = None

                self.goal_status = "drop_box"
                self.drop_box(self.task.end.name).add_done_callback(
                    done_callback,
                )

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
        ) ** 2 <= self.close_radius**2

    def pick_box(self, from_entity: str) -> rclpy.Future:
        """Pick a box from an entity."""
        if self.task is None:
            self.get_logger().error("Can't pick a box without any task")
            raise RuntimeError("Can't pick a box without any task")

        self.get_logger().info(f"Picking box '{self.task.box_id}'")
        return self.box_transfer.call_async(
            wa_srvs.BoxTransfer.Request(
                box_id=self.task.box_id,
                from_entity=from_entity,
                to_entity=self.get_name(),
            ),
        )

    def drop_box(self, to_entity: str) -> rclpy.Future:
        """Drop a box on an entity."""
        if self.task is None:
            self.get_logger().error("Can't drop a box without any task")
            raise RuntimeError("Can't drop a box without any task")

        self.get_logger().info(f"Dropping box '{self.task.box_id}'")
        return self.box_transfer.call_async(
            wa_srvs.BoxTransfer.Request(
                box_id=self.task.box_id,
                from_entity=self.get_name(),
                to_entity=to_entity,
            ),
        )


def main() -> None:
    """Run the mobilebot."""
    rclpy.init()
    node = Mobilebot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
