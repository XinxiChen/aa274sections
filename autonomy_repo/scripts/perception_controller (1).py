#!/usr/bin/env python3

import numpy as np
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl
from asl_tb3_msgs.msg import TurtleBotState

from std_msgs.msg import Bool

class PerceptionController(BaseHeadingController):
    def __init__(self, node_name: str = "perception_controller"):
        super().__init__(node_name)
        self.kP=2.0
        self.declare_parameter("kp", self.kP)

        self.active2=True
        self.declare_parameter("active", self.active2)

        self.image_detected = False
        self.create_subscription(Bool, "/detector_bool", self.image_callback, 10)

    
    def image_callback(self, msg: Bool) -> None:
        """ callback triggered when receiving latest image detection status

        Args:
            msg (Bool): latest image detection status
        """
        if msg.data is True:
            self.image_detected = True
    

    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        msg=TurtleBotControl()
        if self.active:
            msg.omega=0.2
        else:
            msg.omega= 0.0

        
        if self.image_detected is False:
            msg.omega=0.2
        else:
            msg.omega= 0.0
        

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
    
    @property
    def active(self) -> bool:
        """ Get real-time parameter value of active status

        Returns:
            bool: latest parameter value of active status
        """
        
        return self.get_parameter("active").value

if __name__ == "__main__":
    rclpy.init()
    perception_controller=PerceptionController()
    rclpy.spin(perception_controller)
    
    rclpy.shutdown()