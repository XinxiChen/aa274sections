#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


class TimerNode(Node):
    def __init__(self):
        super().__init__("timer_node")

        # self.timer_pub = self.create_publisher(String, "/timer", 10)
        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.kill_sub = self.create_subscription(Bool, "/kill", self.kill_callback, 10)


        self.timer = self.create_timer(0.2, self.timer_callback)

        self.killed = False

        self.counter = 0


    def timer_callback(self):

        # timer_msg = String()
        # timer_msg.data = f"sending constant control..."

        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Constant forward velocity
        twist_msg.angular.z = 0.0  # No rotation

        if self.killed:
            twist_msg.linear.x=0.0
            twist_msg.linear.y=0.0
            twist_msg.linear.z=0.0
            twist_msg.angular.x=0.0
            twist_msg.angular.y=0.0
            twist_msg.angular.z=0.0


        self.twist_pub.publish(twist_msg)
        # self.timer_pub.publish(timer_msg)

        self.counter += 1
        # self.get_logger().info(f"Published message {self.counter}: {timer_msg.data}")

        if self.killed:
            self.timer.cancel()


    def kill_callback(self, msg: Bool):
        self.killed = msg.data


if __name__ == "__main__":
    rclpy.init()
    node = TimerNode()
    rclpy.spin(node)
    rclpy.shutdown()
