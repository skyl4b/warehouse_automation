from __future__ import annotations

from typing import TYPE_CHECKING

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import UInt32
from wa_interfaces.srv import TaskConfirmation

from src.robot import Robot
from src.task_queue import TaskQueue

if TYPE_CHECKING:
    from rclpy.client import Future


class MobileBot(Robot):
    def __init__(self, action_period: float) -> None:
        super().__init__(action_period)

        self._task_queue = TaskQueue(size=2)
        self.confirming = None
        self.goal = 0

        # Inputs
        self.confirmation_client = self.create_client(
            TaskConfirmation,
            "wa/task_transmitter_1/confirmation",
        )
        self.create_subscription(
            UInt32,
            "wa/task_transmitter_1/broadcast",
            self.task_callback,
            1,
        )

        # self.create_service(
        #     String,
        #     f"wa/{self.name}/task_override",
        #     self.override_task,
        # )

        # Outputs
        self.task_started_pub = self.create_publisher(
            UInt32,
            f"wa/{self.name}/task/started",
            10,
        )
        self.task_completed_pub = self.create_publisher(
            UInt32,
            f"wa/{self.name}/task/completed",
            10,
        )

        # Wait for the task transmitter to come online
        while not self.confirmation_client.wait_for_service():
            self.get_logger().warn("Waiting for task_transmitter server")

    @classmethod
    def basename(cls) -> str:
        return "mobile_bot"

    def action_callback(self) -> None:
        if (task_uid := self._task_queue.get()) is None:
            return

        # Execution logic
        if self.goal == 5:  # noqa: PLR2004
            self.get_logger().info(f"Task {task_uid} done")
            msg = UInt32()
            msg.data = task_uid
            self.task_completed_pub.publish(msg)
            self.goal = 0
            self._task_queue.done()
        else:
            self.get_logger().info(f"Executing task {task_uid}")
            self.goal += 1

    def task_callback(self, msg: UInt32) -> None:
        # Use https://app.gazebosim.org/OpenRobotics/fuel/models/Depot
        if self.should_accept_task():
            task_uid = msg.data
            self.get_logger().info(f"Requesting task {task_uid}")
            self.confirming = task_uid

            request = TaskConfirmation.Request()
            request.task_uid = msg.data
            request.entity = self.name
            self.confirmation_client.call_async(request).add_done_callback(
                self.handle_task_confirmation,
            )

    def handle_task_confirmation(self, future: Future) -> None:
        if (task_uid := self.confirming) is not None:
            response: TaskConfirmation.Response = future.result()  # type: ignore
            if response.task_confirmed:
                self.get_logger().info(f"Task {task_uid} confirmed")
                self._task_queue.put(task_uid)
                self.task_started_pub.publish(UInt32(data=task_uid))
            else:
                self.get_logger().info(f"Task {task_uid} denied")
                self.confirming = None

    def should_accept_task(self) -> bool:
        # Decision logic
        return not self._task_queue.full()

    def override_task(self):
        # Override logic
        pass


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()

    robots = 3
    action_period = 1.0

    nodes = [MobileBot(action_period) for _ in range(robots)]

    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
