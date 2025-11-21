#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from asl_tb3_lib.grids import StochOccupancyGrid2D
import numpy as np

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        # Publishers
        self.nav_goal_pub = self.create_publisher(
            PoseStamped, 
            '/nav_goal',  # Check navigator topic name
            10
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.state_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/state',
            self.state_callback,
            10
        )
        
        self.nav_success_sub = self.create_subscription(
            Bool,
            '/nav_success',
            self.nav_success_callback,
            10
        )
        
        # State variables
        self.current_map = None
        self.occupancy_grid = None
        self.robot_state = None
        self.exploring = False
        self.exploration_complete = False
        
        # Timer for periodic frontier detection
        self.create_timer(2.0, self.exploration_loop)
        
        self.get_logger().info("Frontier Explorer initialized")
    
    def map_callback(self, msg):
        """Process incoming map data"""
        self.current_map = msg
        # Convert to StochOccupancyGrid2D for processing
        self.occupancy_grid = StochOccupancyGrid2D(
            msg.data,
            msg.info.resolution,
            msg.info.width,
            msg.info.height,
            msg.info.origin
        )
    
    def state_callback(self, msg):
        """Update robot's current pose"""
        self.robot_state = msg
    
    def nav_success_callback(self, msg):
        """Handle navigation success/failure"""
        if msg.data:
            self.get_logger().info("Navigation goal reached")
            self.exploring = False
        else:
            self.get_logger().warn("Navigation failed, replanning...")
            self.exploring = False
    
    def find_frontiers(self):
        """
        Find frontier points (boundaries between known and unknown space)
        Port your code from Problem 2 here
        
        Returns:
            List of frontier points [(x1, y1), (x2, y2), ...]
        """
        if self.occupancy_grid is None:
            return []
        
        # TODO: Implement frontier detection
        # - Find cells that are free and adjacent to unknown cells
        # - Cluster frontier cells
        # - Return frontier centroids
        
        frontiers = []
        # Your frontier detection logic here
        
        return frontiers
    
    def select_best_frontier(self, frontiers):
        """
        Choose the best frontier to explore next
        
        Consider:
        - Distance from robot
        - Size of frontier
        - Information gain
        """
        if not frontiers or self.robot_state is None:
            return None
        
        # TODO: Implement frontier selection strategy
        # Simple approach: closest frontier
        robot_x = self.robot_state.pose.pose.position.x
        robot_y = self.robot_state.pose.pose.position.y
        
        best_frontier = min(
            frontiers,
            key=lambda f: np.sqrt((f[0]-robot_x)**2 + (f[1]-robot_y)**2)
        )
        
        return best_frontier
    
    def send_navigation_goal(self, x, y):
        """Send navigation goal to navigator"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0  # Face forward
        
        self.nav_goal_pub.publish(goal)
        self.exploring = True
        self.get_logger().info(f"Sending goal: ({x:.2f}, {y:.2f})")
    
    def check_exploration_complete(self):
        """Determine if exploration is complete"""
        if self.occupancy_grid is None:
            return False
        
        # TODO: Check if enough of the map is explored
        # Example: count unknown cells vs total cells
        
        return False
    
    def exploration_loop(self):
        """Main exploration logic"""
        if self.exploration_complete:
            return
        
        if self.exploring:
            # Currently navigating to a goal
            return
        
        if self.check_exploration_complete():
            self.get_logger().info("Exploration complete!")
            self.exploration_complete = True
            return
        
        # Find and navigate to next frontier
        frontiers = self.find_frontiers()
        
        if not frontiers:
            self.get_logger().warn("No frontiers found")
            return
        
        best_frontier = self.select_best_frontier(frontiers)
        
        if best_frontier:
            self.send_navigation_goal(best_frontier[0], best_frontier[1])

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()