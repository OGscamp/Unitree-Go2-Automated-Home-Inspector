import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose2D
from std_msgs.msg import Bool

import numpy as np
from scipy.ndimage import label
import math


class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')
        
        # Declare parameters
        self.declare_parameters()
        self.load_parameters()
        
        # Initialize variables
        self.costmap = None
        self.current_goal = None
        self.exploration_active = False
        self.robot_pose = None
        
        # Initialize ROS interfaces
        self.initialize_subscribers()
        self.initialize_publishers()
        self.initialize_timer()
        
        self.get_logger().info('Frontier Exploration Node initialized')
    
    def declare_parameters(self):
        """Declare all ROS parameters with default values"""
        # Topics
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('goal_pose_topic', 'desired_pose')
        self.declare_parameter('exploration_status_topic', 'exploration_status')
        
        # Exploration parameters
        self.declare_parameter('exploration_rate', 1.0)  # Hz
        self.declare_parameter('min_frontier_size', 10)  # minimum cells
        self.declare_parameter('frontier_travel_point_ratio', 0.5)  # 0.0 to 1.0
        
        # Goal selection strategy: 'nearest', 'largest', 'information_gain'
        self.declare_parameter('goal_selection_strategy', 'nearest')
        
        # Safety parameters
        self.declare_parameter('min_distance_to_obstacle', 0.5)  # meters
        self.declare_parameter('max_goal_distance', 10.0)  # meters
        
        # Exploration completion
        self.declare_parameter('exploration_complete_threshold', 0.95)  # % of map explored
        self.declare_parameter('min_free_space_for_completion', 100)  # cells
        
    def load_parameters(self):
        """Load all parameters from ROS parameter server"""
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.exploration_status_topic = self.get_parameter('exploration_status_topic').value
        
        self.exploration_rate = self.get_parameter('exploration_rate').value
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.frontier_travel_point_ratio = self.get_parameter('frontier_travel_point_ratio').value
        
        self.goal_selection_strategy = self.get_parameter('goal_selection_strategy').value
        
        self.min_distance_to_obstacle = self.get_parameter('min_distance_to_obstacle').value
        self.max_goal_distance = self.get_parameter('max_goal_distance').value
        
        self.exploration_complete_threshold = self.get_parameter('exploration_complete_threshold').value
        self.min_free_space_for_completion = self.get_parameter('min_free_space_for_completion').value
        
    def initialize_subscribers(self):
        """Initialize all subscribers"""
        # QoS profile for costmap (transient local for map-like data)
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            costmap_qos
        )
        
        self.get_logger().info(f'Subscribed to costmap: {self.costmap_topic}')
    
    def initialize_publishers(self):
        """Initialize all publishers"""
        self.goal_pub = self.create_publisher(
            Pose2D,
            self.goal_pose_topic,
            10
        )
        
        self.status_pub = self.create_publisher(
            Bool,
            self.exploration_status_topic,
            10
        )
        
        self.get_logger().info(f'Publishing goals to: {self.goal_pose_topic}')
    
    def initialize_timer(self):
        """Initialize exploration timer"""
        timer_period = 1.0 / self.exploration_rate
        self.exploration_timer = self.create_timer(timer_period, self.exploration_callback)
        self.exploration_active = True
    
    def costmap_callback(self, msg):
        """Process incoming costmap data"""
        self.costmap = msg
        
        # Convert occupancy grid to numpy array
        width = msg.info.width
        height = msg.info.height
        self.costmap_data = np.array(msg.data).reshape((height, width))
        
        # Store costmap metadata
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        
    def exploration_callback(self):
        """Main exploration loop - called at exploration_rate Hz"""
        if not self.exploration_active:
            return
        
        if self.costmap is None:
            self.get_logger().warn('No costmap received yet', throttle_duration_sec=5.0)
            return
        
        # Check if exploration is complete
        if self.is_exploration_complete():
            self.get_logger().info('Exploration complete!')
            self.exploration_active = False
            self.publish_exploration_status(False)
            return
        
        # Find frontiers
        frontiers = self.find_frontiers()
        
        if len(frontiers) == 0:
            self.get_logger().warn('No frontiers found!', throttle_duration_sec=5.0)
            return
        
        # Select best frontier
        goal = self.select_best_frontier(frontiers)
        
        if goal is not None:
            self.publish_goal(goal)
            self.current_goal = goal
            self.get_logger().info(f'New goal sent: ({goal[0]:.2f}, {goal[1]:.2f})')
    
    def find_frontiers(self):
        """
        Find frontier cells (boundaries between free and unknown space)
        Returns list of frontier clusters
        """
        # Costmap values: -1 = unknown, 0 = free, 100 = occupied
        unknown = (self.costmap_data == -1)
        free = (self.costmap_data == 0)
        
        # Find cells that are free and adjacent to unknown
        frontier_mask = np.zeros_like(self.costmap_data, dtype=bool)
        
        # Check 8-connectivity
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                shifted_unknown = np.roll(np.roll(unknown, dy, axis=0), dx, axis=1)
                frontier_mask |= (free & shifted_unknown)
        
        # Label connected frontier regions
        labeled_frontiers, num_frontiers = label(frontier_mask)
        
        frontiers = []
        for i in range(1, num_frontiers + 1):
            frontier_cells = np.argwhere(labeled_frontiers == i)
            
            # Filter by minimum size
            if len(frontier_cells) < self.min_frontier_size:
                continue
            
            # Calculate frontier properties
            frontier_info = self.analyze_frontier(frontier_cells)
            if frontier_info is not None:
                frontiers.append(frontier_info)
        
        self.get_logger().info(f'Found {len(frontiers)} valid frontiers', throttle_duration_sec=2.0)
        return frontiers
    
    def analyze_frontier(self, frontier_cells):
        """
        Analyze a frontier cluster and return its properties
        Returns: dict with 'centroid', 'size', 'cells'
        """
        # Calculate centroid in map coordinates
        centroid_cell = np.mean(frontier_cells, axis=0)
        centroid_world = self.cell_to_world(centroid_cell[0], centroid_cell[1])
        
        # Check if centroid is safe (not too close to obstacles)
        if not self.is_safe_goal(centroid_world):
            return None
        
        # Calculate travel point (slightly inside free space)
        travel_point = self.calculate_travel_point(frontier_cells)
        
        return {
            'centroid': centroid_world,
            'travel_point': travel_point,
            'size': len(frontier_cells),
            'cells': frontier_cells
        }
    
    def calculate_travel_point(self, frontier_cells):
        """
        Calculate a point slightly inside free space from frontier
        This helps avoid goals right at the edge
        """
        # Use centroid for now (can be improved with more sophisticated methods)
        centroid_cell = np.mean(frontier_cells, axis=0)
        
        # Find direction towards free space (away from unknown)
        # Simple approach: use weighted average towards known free cells
        free_mask = (self.costmap_data == 0)
        
        # Get cells around frontier
        y_min = max(0, int(centroid_cell[0]) - 5)
        y_max = min(self.costmap_data.shape[0], int(centroid_cell[0]) + 5)
        x_min = max(0, int(centroid_cell[1]) - 5)
        x_max = min(self.costmap_data.shape[1], int(centroid_cell[1]) + 5)
        
        local_free = free_mask[y_min:y_max, x_min:x_max]
        
        if np.any(local_free):
            # Find centroid of free space
            free_cells = np.argwhere(local_free)
            free_centroid = np.mean(free_cells, axis=0) + np.array([y_min, x_min])
            
            # Move travel point between frontier and free space
            travel_cell = (centroid_cell * (1 - self.frontier_travel_point_ratio) + 
                          free_centroid * self.frontier_travel_point_ratio)
        else:
            travel_cell = centroid_cell
        
        return self.cell_to_world(travel_cell[0], travel_cell[1])
    
    def select_best_frontier(self, frontiers):
        """
        Select the best frontier based on the chosen strategy
        Returns: (x, y) world coordinates
        """
        if len(frontiers) == 0:
            return None
        
        # Get robot position (assume at origin if not available)
        robot_x, robot_y = 0.0, 0.0  # TODO: Get from TF if needed
        
        if self.goal_selection_strategy == 'nearest':
            # Select nearest frontier
            min_dist = float('inf')
            best_frontier = None
            
            for frontier in frontiers:
                goal = frontier['travel_point']
                dist = math.sqrt((goal[0] - robot_x)**2 + (goal[1] - robot_y)**2)
                
                # Skip if too far
                if dist > self.max_goal_distance:
                    continue
                
                if dist < min_dist:
                    min_dist = dist
                    best_frontier = goal
            
            return best_frontier
        
        elif self.goal_selection_strategy == 'largest':
            # Select largest frontier
            largest = max(frontiers, key=lambda f: f['size'])
            return largest['travel_point']
        
        elif self.goal_selection_strategy == 'information_gain':
            # Select frontier with best information gain (size/distance trade-off)
            best_score = -float('inf')
            best_frontier = None
            
            for frontier in frontiers:
                goal = frontier['travel_point']
                dist = math.sqrt((goal[0] - robot_x)**2 + (goal[1] - robot_y)**2)
                
                # Skip if too far
                if dist > self.max_goal_distance:
                    continue
                
                # Score = size / distance (favor large, nearby frontiers)
                score = frontier['size'] / (dist + 1.0)  # +1 to avoid division by zero
                
                if score > best_score:
                    best_score = score
                    best_frontier = goal
            
            return best_frontier
        
        else:
            self.get_logger().error(f'Unknown goal selection strategy: {self.goal_selection_strategy}')
            return frontiers[0]['travel_point']
    
    def is_safe_goal(self, world_pos):
        """Check if a goal position is safe (not too close to obstacles)"""
        cell_pos = self.world_to_cell(world_pos[0], world_pos[1])
        row, col = int(cell_pos[0]), int(cell_pos[1])
        
        # Check bounds
        if row < 0 or row >= self.costmap_data.shape[0] or col < 0 or col >= self.costmap_data.shape[1]:
            return False
        
        # Check surrounding cells for obstacles
        search_radius = int(self.min_distance_to_obstacle / self.resolution)
        
        for dr in range(-search_radius, search_radius + 1):
            for dc in range(-search_radius, search_radius + 1):
                r, c = row + dr, col + dc
                
                if r < 0 or r >= self.costmap_data.shape[0] or c < 0 or c >= self.costmap_data.shape[1]:
                    continue
                
                # Check if occupied (value > 50 is considered obstacle)
                if self.costmap_data[r, c] > 50:
                    return False
        
        return True
    
    def is_exploration_complete(self):
        """Check if exploration is complete based on map coverage"""
        total_cells = self.costmap_data.size
        unknown_cells = np.sum(self.costmap_data == -1)
        free_cells = np.sum(self.costmap_data == 0)
        
        # Calculate explored percentage
        explored_percentage = (total_cells - unknown_cells) / total_cells
        
        # Check if we have enough free space and high exploration percentage
        if (explored_percentage >= self.exploration_complete_threshold and 
            free_cells >= self.min_free_space_for_completion):
            return True
        
        return False
    
    def cell_to_world(self, row, col):
        """Convert cell coordinates to world coordinates"""
        x = col * self.resolution + self.origin_x
        y = row * self.resolution + self.origin_y
        return (x, y)
    
    def world_to_cell(self, x, y):
        """Convert world coordinates to cell coordinates"""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return (row, col)
    
    def publish_goal(self, goal):
        """Publish goal to navigation system"""
        goal_msg = Pose2D()
        goal_msg.x = goal[0]
        goal_msg.y = goal[1]
        goal_msg.theta = 0.0  # Let the planner handle orientation
        
        self.goal_pub.publish(goal_msg)
    
    def publish_exploration_status(self, active):
        """Publish exploration status"""
        status_msg = Bool()
        status_msg.data = active
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()