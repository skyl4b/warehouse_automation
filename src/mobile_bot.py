from robot import Robot
from std_msgs.msg import String


class MobileBot(Robot):
    def __init__(self) -> None:
        super().__init__("mobilebot")

        self.response_pub = self.create_publisher(String, "robot_response", 10)
        self.create_subscription(
            String, "task_broadcast", self.task_callback, 10
        )
        self.create_subscription(
            String, "task_confirmation", self.task_confirmation_callback, 10
        )
        self._task = None

    def task_callback(self, msg):
        if self.should_accept_task():
            response_msg = String(data="accepted")
            self.response_pub.publish(response_msg)

    def task_confirmation_callback(self, msg):
        if msg.data == "confirmed":
            self.execute_task()

    def should_accept_task(self):
        # Decision logic
        return True

    def execute_task(self):
        # Execution logic
        pass
