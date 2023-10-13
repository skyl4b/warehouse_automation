import rclpy
from task_transmitter import TaskTransmitter


def main() -> None:
    rclpy.init()
    transmitter = TaskTransmitter()
    rclpy.spin(transmitter)


if __name__ == "__main__":
    main()
