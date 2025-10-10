#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TimerNode(Node):
    def __init__(self):
        super().__init__("timer_node")

        # self.timer_pub = self.create_publisher(String, "/timer", 10)
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.2, self.timer_callback)


        self.counter = 0


    def timer_callback(self):

        # timer_msg = String()
        # timer_msg.data = f"sending constant control..."

        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Constant forward velocity
        twist_msg.angular.z = 0.0  # No rotation

        self.twist_pub.publish(twist_msg)
        # self.timer_pub.publish(timer_msg)

        self.counter += 1
        # self.get_logger().info(f"Published message {self.counter}: {timer_msg.data}")


if __name__ == "__main__":
    rclpy.init()
    node = TimerNode()
    rclpy.spin(node)
    rclpy.shutdown()
