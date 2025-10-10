#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TimerNode(Node):
    def __init__(self):
        super().__init__("timer_node")

        self.timer_pub = self.create_publisher(String, "/timer", 10)

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.counter = 0


    def timer_callback(self):
        msg = String()
        msg.data = "sending constant control..."
        self.timer_pub.publish(msg)
        self.counter += 1
        self.get_logger().info(f"Published message {self.counter}: {msg.data}")


if __name__ == "__main__":
    rclpy.init()
    node = TimerNode()
    rclpy.spin(node)
    rclpy.shutdown()
