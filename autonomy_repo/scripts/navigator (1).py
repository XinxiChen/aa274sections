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

from asl_tb3_lib.control import BaseController
from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState
from asl_tb3_lib.grids import StochOccupancyGrid2D

from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.tf_utils import quaternion_to_yaw

# create Node that inherits from BaseNavigator
class Navigator(BaseNavigator):

    def __init__(self) -> None:
        # give it a default node name
        super().__init__("Navigator")
        print("Navigator node started.")
        # needed for code from HW2 P2
        # change these values??
        self.kp = 2.0
        self.kpx = 2.0
        self.kdx = 2.0
        self.kpy = 2.0
        self.kdy = 2.0

        self.V_PREV_THRESH = 0.001
        self.t_prev = 0.0
        self.V_prev = 0.0
        self.om_prev = 0.0

    def compute_heading_control(self,
        state: TurtleBotState,
        goal: TurtleBotState
    ) -> TurtleBotControl:
        """ Compute only orientation target (used for NavMode.ALIGN and NavMode.Park)

        Returns:
            TurtleBotControl: control target
        """
        # raise NotImplementedError("You need to implement this!")

        heading_error = wrap_angle(goal.theta - state.theta)
        correction_vel = self.kp * (heading_error)
        
        control_message = TurtleBotControl()
        control_message.omega = correction_vel
        
        return control_message

    def compute_trajectory_tracking_control(self,
        state: TurtleBotState,
        plan: TrajectoryPlan,
        t: float,
    ) -> TurtleBotControl:
        """ Compute control target using a trajectory tracking controller

        Args:
            state (TurtleBotState): current robot state
            plan (TrajectoryPlan): planned trajectory
            t (float): current timestep

        Returns:
            TurtleBotControl: control command
        """
        # edit t
        t_trimmed = np.clip(t, 0, plan.duration)

        # compute desired x_d, xd_d, xdd_d, y_d, yd_d, ydd_d
        x_d = splev(t_trimmed, plan.path_x_spline, der = 0)
        xd_d = splev(t_trimmed, plan.path_x_spline, der = 1)
        xdd_d = splev(t_trimmed, plan.path_x_spline, der = 2)

        y_d = splev(t_trimmed, plan.path_y_spline, der = 0)
        yd_d = splev(t_trimmed, plan.path_y_spline, der = 1)
        ydd_d = splev(t_trimmed, plan.path_y_spline, der = 2)

        # use these for HW2 code below
        x = state.x 
        y = state.y
        th = state.theta
        dt = t - self.t_prev

        ############### from HW2, P2_trajectory_tracking.py: #################

        # avoid singularity
        if abs(self.V_prev) < self.V_PREV_THRESH:
            self.V_prev = self.V_PREV_THRESH

        xd = self.V_prev*np.cos(th)
        yd = self.V_prev*np.sin(th)

        # compute virtual controls
        u = np.array([xdd_d + self.kpx*(x_d-x) + self.kdx*(xd_d-xd),
                      ydd_d + self.kpy*(y_d-y) + self.kdy*(yd_d-yd)])

        # compute real controls
        J = np.array([[np.cos(th), -self.V_prev*np.sin(th)],
                          [np.sin(th), self.V_prev*np.cos(th)]])
        a, om = linalg.solve(J, u)
        V = self.V_prev + a*dt
        ######################## HW 2 Code ends here ###########################

        # save the commands that were applied and the time
        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        print("V:", V, " omega:", om)

        return  TurtleBotControl(v = V, omega = om)

        # raise NotImplementedError("You need to implement this!")

    def compute_trajectory_plan(self,
        state: TurtleBotState,
        goal: TurtleBotState,
        occupancy: StochOccupancyGrid2D,
        resolution: float,
        horizon: float,
    ) -> T.Optional[TrajectoryPlan]:
        """ Compute a trajectory plan using A* and cubic spline fitting

        Args:
            state (TurtleBotState): state
            goal (TurtleBotState): goal
            occupancy (StochOccupancyGrid2D): occupancy
            resolution (float): resolution
            horizon (float): horizon

        Returns:
            T.Optional[TrajectoryPlan]:
        """
        # raise NotImplementedError("You need to implement this!")

        # T.Optional means function can also return none

        # initialize A*
        # taken from parameters passed into compute_trajectory_plan
        new_Astar = AStar(
            statespace_hi = (state.x + horizon, state.y + horizon),
            statespace_lo = (state.x - horizon, state.y - horizon),
            x_init = (state.x, state.y),
            x_goal = (goal.x, goal.y),
            occupancy = occupancy,
            resolution = resolution
        )
        if new_Astar is None:
            self.get_logger().warn("IM NONE")

        # if A* problem is not solvable or length of path < 4 then return None
        if not new_Astar.solve() or len(new_Astar.path) < 4:
            self.get_logger().warn("Plan failed")
            return None

        # reset class variables for previous velocity and time
        self.t_prev = 0
        self.V_prev = 0
        self.om_prev = 0

        # compute planned time stamps using constant velocity heuristics
        v_desired = 0.15
        # spline_alpha = 0.2
        new_path = np.asarray(new_Astar.path)

        # array of timestamps and cubic spline fitting, adapted from compute_smooth_plan in sim_astar.ipynb
        ts = [0]
        x_spline = None
        y_spline = None 

        for current_node, next_node in zip(new_path[:-1], new_path[1:]):
            # using euclidian norm
            distance = np.linalg.norm(next_node - current_node, ord = 2)

            t_segment = distance / v_desired
            ts.append(ts[-1] + t_segment)
        
        # fit cubic splines to x coords 
        x_spline = splrep(ts, new_path[:,0], s = 0)

        # fit cubic splines to y coords
        y_spline = splrep(ts, new_path[:,1], s = 0)

        print("Planned trajectory with", len(new_path), "waypoints.")

        # return trajectory plan
        return TrajectoryPlan(
            path = new_path,
            path_x_spline = x_spline,
            path_y_spline = y_spline,
            duration = ts[-1]
        )


# Create A_star class, from HW1 

class AStar(object):
    """Represents a motion planning problem to be solved using A*"""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1):
        self.statespace_lo = statespace_lo         # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = statespace_hi         # state space upper bound (e.g., [5, 5])
        self.occupancy = occupancy                 # occupancy grid (a DetOccupancyGrid2D object)
        self.resolution = resolution               # resolution of the discretization of state space (cell/m)
        self.x_offset = x_init                     
        self.x_init = self.snap_to_grid(x_init)    # initial state
        self.x_goal = self.snap_to_grid(x_goal)    # goal state

        self.closed_set = set()    # the set containing the states that have been visited
        self.open_set = set()      # the set containing the states that are condidate for future expension

        self.est_cost_through = {}  # dictionary of the estimated cost from start to goal passing through state (often called f score)
        self.cost_to_arrive = {}    # dictionary of the cost-to-arrive at state from start (often called g score)
        self.came_from = {}         # dictionary keeping track of each state's parent to reconstruct the path

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.distance(self.x_init,self.x_goal)

        self.path = None        # the final path as a list of states

    def is_free(self, x):
        """
        Checks if a given state x is free, meaning it is inside the bounds of the map and
        is not inside any obstacle.
        Inputs:
            x: state tuple
        Output:
            Boolean True/False
        Hint: self.occupancy is a DetOccupancyGrid2D object, take a look at its methods for what might be
              useful here
        """
        ########## Code starts here ##########
        # raise NotImplementedError("is_free not implemented")
    
        # if x_coordinate is outside x_bounds of map, return false
        if x[0] < self.statespace_lo[0] or x[0] > self.statespace_hi[0]:
            return False

        # if y is outside bounds of map, return false
        if x[1] < self.statespace_lo[1] or x[1] > self.statespace_hi[1]:
            return False
        
        # if x is inside an obstacle, return false, otherwise return true
        return self.occupancy.is_free(np.array(x))
    
        ########## Code ends here ##########

    def distance(self, x1, x2):
        """
        Computes the Euclidean distance between two states.
        Inputs:
            x1: First state tuple
            x2: Second state tuple
        Output:
            Float Euclidean distance

        HINT: This should take one line. Tuples can be converted to numpy arrays using np.array().
        """
        ########## Code starts here ##########
        # raise NotImplementedError("distance not implemented")

        x1_array = np.array(x1)
        x2_array = np.array(x2)

        # calculate Euclidian distance (ord = 2), or Manhattan (ord = 1), or Chevyshev (ord = infinity)
        return np.linalg.norm(x2_array - x1_array, ord=2)
        ########## Code ends here ##########

    def snap_to_grid(self, x):
        """ Returns the closest point on a discrete state grid
        Input:
            x: tuple state
        Output:
            A tuple that represents the closest point to x on the discrete state grid
        """
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        """
        Gets the FREE neighbor states of a given state x. Assumes a motion model
        where we can move up, down, left, right, or along the diagonals by an
        amount equal to self.resolution.
        Input:
            x: tuple state
        Ouput:
            List of neighbors that are free, as a list of TUPLES

        HINTS: Use self.is_free to check whether a given state is indeed free.
               Use self.snap_to_grid (see above) to ensure that the neighbors
               you compute are actually on the discrete grid, i.e., if you were
               to compute neighbors by adding/subtracting self.resolution from x,
               numerical errors could creep in over the course of many additions
               and cause grid point equality checks to fail. To remedy this, you
               should make sure that every neighbor is snapped to the grid as it
               is computed.
        """
        neighbors = []
        ########## Code starts here ##########
        # raise NotImplementedError("get_neighbors not implemented")

        # move up. snap to grid. if free, add to neighbors
        # snap_to_grid returns tuple on grid; is_free returns boolean
        up = self.snap_to_grid((x[0], x[1] + self.resolution))
        if self.is_free(up):
            neighbors.append(up)        

        # move down. snap to grid. if free, add to neighbors
        down = self.snap_to_grid((x[0], x[1] - self.resolution))
        if self.is_free(down):
            neighbors.append(down)

        # move left 
        left = self.snap_to_grid((x[0] - self.resolution, x[1]))
        if self.is_free(left):
            neighbors.append(left)

        # move right
        right = self.snap_to_grid((x[0] + self.resolution, x[1]))
        if self.is_free(right):
            neighbors.append(right)

        # move NW
        nw = self.snap_to_grid((x[0] - self.resolution, x[1] + self.resolution))
        if self.is_free(nw):
            neighbors.append(nw)        

        # move SW
        sw = self.snap_to_grid((x[0] - self.resolution, x[1] - self.resolution))
        if self.is_free(sw):
            neighbors.append(sw)

        # move NE
        ne = self.snap_to_grid((x[0] + self.resolution, x[1] + self.resolution))
        if self.is_free(ne):
            neighbors.append(ne)

        # move SE 
        se = self.snap_to_grid((x[0] + self.resolution, x[1] - self.resolution))
        if self.is_free(se):
            neighbors.append(se)
    
        ########## Code ends here ##########
        return neighbors
    
    def find_best_est_cost_through(self):
        """
        Gets the state in open_set that has the lowest est_cost_through
        Output: A tuple, the state found in open_set that has the lowest est_cost_through
        """
        return min(self.open_set, key=lambda x: self.est_cost_through[x])
    
    def reconstruct_path(self):
        """
        Use the came_from map to reconstruct a path from the initial location to
        the goal location
        Output:
        A list of tuples, which is a list of the states that go from start to goal
        """
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))
        
    def plot_path(self, fig_num=0, show_init_label=True):
        """Plots the path found in self.path and the obstacles"""
        if not self.path:
            return
        
        self.occupancy.plot(fig_num)
        
        solution_path = np.asarray(self.path)
        plt.plot(solution_path[:,0],solution_path[:,1], color="green", linewidth=2, label="A* solution path", zorder=10)
        plt.scatter([self.x_init[0], self.x_goal[0]], [self.x_init[1], self.x_goal[1]], color="green", s=30, zorder=10)
        if show_init_label:
            plt.annotate(r"$x_{init}$", np.array(self.x_init) + np.array([.2, .2]), fontsize=16)
            plt.annotate(r"$x_{goal}$", np.array(self.x_goal) + np.array([.2, .2]), fontsize=16)
            plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)
        
        plt.axis([0, self.occupancy.width, 0, self.occupancy.height])
    
    def plot_tree(self, point_size=15):
        plot_line_segments([(x, self.came_from[x]) for x in self.open_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        plot_line_segments([(x, self.came_from[x]) for x in self.closed_set if x != self.x_init], linewidth=1, color="blue", alpha=0.2)
        px = [x[0] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
        py = [x[1] for x in self.open_set | self.closed_set if x != self.x_init and x != self.x_goal]
        plt.scatter(px, py, color="blue", s=point_size, zorder=10, alpha=0.2)
        
    def solve(self):
        """
        Solves the planning problem using the A* search algorithm. It places
        the solution as a list of tuples (each representing a state) that go
        from self.x_init to self.x_goal inside the variable self.path
        Input:
        None
        Output:
        Boolean, True if a solution from x_init to x_goal was found
        
        HINTS: We're representing the open and closed sets using python's built-in
        set() class. This allows easily adding and removing items using
        .add(item) and .remove(item) respectively, as well as checking for
        set membership efficiently using the syntax "if item in set".
        """
        ########## Code starts here ##########
        # raise NotImplementedError("solve not implemented")
        
        # while open set is not empty (will evaluate true if set contains elements)
        while self.open_set:
            
            # set x_current to the lowest estimated cost through O?
            x_current = self.find_best_est_cost_through()
            
            # if we have found the goal, return path
            if x_current == self.x_goal:
                self.path = self.reconstruct_path() 
                return True
            # remove x_current from open set
            self.open_set.remove(x_current)
            
            # add x_current to closed set
            self.closed_set.add(x_current)
            
            for x_neigh in self.get_neighbors(x_current):
                # if neighbor has already been explored
                if x_neigh in self.closed_set:
                    continue # move on to next neighbor
                
                # f = g + h, where g is cost to get to x_current, h is euclidian distance to goal
                tentative_cost_to_arrive = self.cost_to_arrive[x_current] + self.distance(x_current, x_neigh)
                
                # add x_neigh to open set if new
                if x_neigh not in self.open_set:
                    self.open_set.add(x_neigh)
                # if we had already found a better path to x_neigh, move on
                elif tentative_cost_to_arrive >= self.cost_to_arrive[x_neigh]:
                    continue
                
                # assign parent to x_neigh in dictionary
                self.came_from[x_neigh] = x_current
                
                # assign cost to arrive to x_neigh, put in dictionary
                self.cost_to_arrive[x_neigh] = tentative_cost_to_arrive
                
                # assign estimated cost (new) to goal via x_neigh, put in dictionary
                self.est_cost_through[x_neigh] = tentative_cost_to_arrive + self.distance(x_neigh, self.x_goal)
        # otherwise, return false for path not found
        return False 
    
    ########## Code ends here ##########



if __name__ == "__main__":
    rclpy.init() # initialize ROS client library
    node = Navigator() # create the node instance
    rclpy.spin(node) # call ROS2 default scheduler
    rclpy.shutdown() # clean up after node exits
