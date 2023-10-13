from rclpy.node import Node
from std_msgs.msg import String


class TaskTransmitter(Node):
    def __init__(self) -> None:
        super().__init__("task_transmitter")
        self._task_stack = []
        self.task_pub = self.create_publisher(String, "task_broadcast", 10)
        self.confirmation_pub = self.create_publisher(
            String, "task_confirmation", 10
        )
        self.subscription = self.create_subscription(
            String, "robot_response", self.robot_response_callback, 10
        )

    def robot_response_callback(self, msg):
        if self.task_accepted:
            return
        if msg.data == "accepted":
            self.task_accepted = True
            confirm_msg = String(data="confirmed")
            self.confirmation_pub.publish(confirm_msg)

    def broadcast_task(self):
        self.task_accepted = False
        task_msg = String(data="task details")
        self.task_pub.publish(task_msg)
        # Retry if necessary
