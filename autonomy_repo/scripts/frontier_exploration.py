#!/usr/bin/env python3

# code to build a ROS node

# import all necessary packages
import math
import numpy as np
import typing as T

# from utils import plot_line_segments

import rclpy
from rclpy.node import Node
from scipy import linalg
from scipy.interpolate import splrep, splev

from nav_msgs.msg import OccupancyGrid, Path

from asl_tb3_lib.control import BaseController
from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from asl_tb3_lib.grids import StochOccupancyGrid2D

from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw

import numpy as np
import typing as T

from enum import Enum
from dataclasses import dataclass
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from scipy.interpolate import splev
from std_msgs.msg import Bool

from asl_tb3_msgs.msg import TurtleBotState, TurtleBotControl
from asl_tb3_lib.control import BaseController
from asl_tb3_lib.grids import snap_to_grid, StochOccupancyGrid2D
from asl_tb3_lib.math_utils import wrap_angle, distance_linear, distance_angular

from scipy.signal import convolve2d


# create Node that inherits from BaseNavigator
class FrontierExplorer(Node):

    def __init__(self) -> None:
        super().__init__("FrontierExplorer")

        self.get_logger().info("Frontier Explorer started.")

        self.state: T.Optional[TurtleBotState] = None
        self.occupancy: T.Optional[StochOccupancyGrid2D] = None

        # create relevant node variables
        # TODO
        self.reached_target_pose = False
        self.image_detected = False
        self.timer = self.create_timer(5.0, self.explore)

        # create various subscribers and publishers to interface with the navigator
        self.cmd_nav_pub = self.create_publisher(TurtleBotState, "/cmd_nav", 10)

        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.nav_success_sub = self.create_subscription(Bool, "/nav_success", self.explore_callback, 10)
        self.state_sub = self.create_subscription(TurtleBotState, "/state", self.state_callback, 10)

        self.detector_sub = self.create_subscription(Bool, "/detector_bool", self.image_callback, 10)

    def state_callback(self, msg: TurtleBotState) -> None:
        """ callback triggered when receiving latest turtlebot state

        Args:
            msg (TurtleBotState): latest turtlebot state
        """
        self.state = msg

    def explore_callback(self,msg):
        print("Reached target pose")
        self.explore()

    def image_callback(self, msg: Bool) -> None:
        """ callback triggered when receiving latest image detection status

        Args:
            msg (Bool): latest image detection status
        """
        #if msg.data is True:
        self.image_detected = msg.data

    def explore(self):
        """ returns potential states to explore
        Args:
            occupancy (StochasticOccupancyGrid2D): Represents the known, unknown, occupied, and unoccupied states. See class in first section of notebook.

        Returns:
            frontier_states (np.ndarray): state-vectors in (x, y) coordinates of potential states to explore. Shape is (N, 2), where N is the number of possible states to explore.

        HINTS:
        - Function `convolve2d` may be helpful in producing the number of unknown, and number of occupied states in a window of a specified cell
        - Note the distinction between physical states and grid cells. Most operations can be done on grid cells, and converted to physical states at the end of the function with `occupancy.grid2state()`
        """

        if not self.occupancy:
            self.get_logger().warn("No occupancy")
            return

        if not self.state:
            self.get_logger().warn("No state")
            return

        window_size = 13    # defines the window side-length for neighborhood of cells to consider for heuristics
        ########################### Code starts here ###########################
        # grouping by probabilities
        unknown_cell = (self.occupancy.probs < 0)
        occupied_cell = (self.occupancy.probs >= 0.5)
        free_cell = (self.occupancy.probs < 0.5) & (self.occupancy.probs >=0)
        # kernel with given window side-length
        kernel = np.ones((window_size, window_size))

        # apply convolutions
        num_unknown_in_window = convolve2d(unknown_cell, kernel, mode='same', boundary = "fill", fillvalue = 0)
        num_occupied_in_window = convolve2d(occupied_cell, kernel, mode='same',boundary = "fill", fillvalue = 0)
        num_free_in_window = convolve2d(free_cell, kernel, mode='same',boundary = "fill", fillvalue = 0)

        # apply heuristics
        mask = (num_unknown_in_window >= 0.2*window_size*window_size) & (num_occupied_in_window == 0) & (num_free_in_window >= 0.3*window_size*window_size) & free_cell
        frontier_inds = np.argwhere(mask)

        # swap (y, x) to (x, y) and compute frontier states
        frontier_states = self.occupancy.grid2state(np.column_stack((frontier_inds[:, 1], frontier_inds[:, 0])))
        
        if(frontier_states.shape[0]==0):
            self.get_logger().warn("No frontier states found")
            return frontier_states # avoid error of argmin on empty array
        state_distances = np.linalg.norm(frontier_states-np.array([self.state.x, self.state.y]),axis=1)
        closest_frontier_pt = frontier_states[np.argmin(state_distances)] # identify closest frontier state

        if self.image_detected:
            self.get_logger().warn("IMAGE DETECTED")
            self.cmd_nav_pub.publish(TurtleBotState(
            x=self.state.x,
            y=self.state.y,
            theta=self.state.theta)
            )
            return frontier_states

        self.get_logger().warn("NO IMAGE")
        # publish to cmd_nav_pub
        goal = TurtleBotState(
            x=float(closest_frontier_pt[0]),
            y=float(closest_frontier_pt[1]),
            theta=float(0),
        )

        print(f"Robot pose: {self.state.x, self.state.y}")
        print(f"Goal frontier pose: {goal.x, goal.y}")
        

        self.cmd_nav_pub.publish(goal)
        self.get_logger().info(f"Published goal: {goal}")

        ########################### Code ends here ###########################
        return frontier_states

    def map_callback(self, msg: OccupancyGrid) -> None:
        """ Callback triggered when the map is updated

        Args:
            msg (OccupancyGrid): updated map message
        """
        self.occupancy = StochOccupancyGrid2D(
            resolution=msg.info.resolution,
            size_xy=np.array([msg.info.width, msg.info.height]),
            origin_xy=np.array([msg.info.origin.position.x, msg.info.origin.position.y]),
            window_size=9,
            probs=msg.data,
        )

def main():
    rclpy.init()
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
