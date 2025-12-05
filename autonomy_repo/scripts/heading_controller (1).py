#!/usr/bin/env python3

import numpy as np
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_msgs.msg import TurtleBotState

class HeadingController(BaseHeadingController):
    def __init__(self):
        super().__init__()
        self.kP=2.0
        self.declare_parameter("kp", self.kP)
        
    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        msg=TurtleBotControl()
        msg.omega=goal.theta-state.theta
        msg.omega=wrap_angle(msg.omega)
        msg.omega=msg.omega*self.kP
        return msg
    
    @property
    def kp(self) -> float:
        """ Get real-time parameter value of maximum velocity

        Returns:
            float: latest parameter value of maximum velocity
        """
        
        return self.get_parameter("kp").value

if __name__ == "__main__":
    rclpy.init()
    headControl=HeadingController()
    rclpy.spin(headControl)
    
    rclpy.shutdown()