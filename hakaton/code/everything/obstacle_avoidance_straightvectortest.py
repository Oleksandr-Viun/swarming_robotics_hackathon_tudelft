import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from library.utils import clamp


class ObstacleAvoider:
    """
    Handles robust obstacle detection and avoidance using ultrasonic sensors.
    Integrates a 'consecutive hits' filter to handle sensor noise.
    """
    
    def __init__(self, node: Node, robot, config=None):
        self.node = node
        self.robot = robot
        
        # Configuration parameters
        self.config = {
            'obstacle_threshold': 0.45,     # meters - start avoiding when closer than this
            'emergency_threshold': 0.25,    # meters - stop immediately when this close
            'avoidance_angular': 0.8,       # rad/s - rotation speed when avoiding
            'avoidance_linear': 0.0,        # m/s - turn in place for safety
            'clearance_multiplier': 1.3,    # safety margin for resuming navigation
            'required_hits': 2,             # consecutive readings needed to trigger avoidance
            'left_sonar_topic': '/io/distance/front_left',
            'right_sonar_topic': '/io/distance/front_right',
        }
        
        if config:
            self.config.update(config)
        
        # Sensor readings
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # Filter counters (from Pioneer approach)
        self.left_hits = 0
        self.right_hits = 0
        
        # State
        self.is_avoiding = False
        self.avoid_direction = 0  # 1 for left, -1 for right
        self.last_emergency_time = 0
        
        # Create subscribers
        self._setup_subscribers()
        
        self.node.get_logger().info("Filtered ObstacleAvoider initialized")
    
    def _setup_subscribers(self):
        self.left_sub = self.node.create_subscription(
            Range, 
            self.config['left_sonar_topic'], 
            self._left_sonar_cb, 
            10
        )
        self.right_sub = self.node.create_subscription(
            Range, 
            self.config['right_sonar_topic'], 
            self._right_sonar_cb, 
            10
        )
    
    def _left_sonar_cb(self, msg: Range):
        self.left_distance = msg.range
        # Update hits counter
        if msg.range < self.config['obstacle_threshold']:
            self.left_hits += 1
        else:
            self.left_hits = 0

    def _right_sonar_cb(self, msg: Range):
        self.right_distance = msg.range
        # Update hits counter
        if msg.range < self.config['obstacle_threshold']:
            self.right_hits += 1
        else:
            self.right_hits = 0
    
    def check_obstacles(self):
        """
        Determine if avoidance is needed using the consecutive hits filter.
        Returns: (status, direction)
        """
        left = self.left_distance
        right = self.right_distance
        
        # 1. Emergency stop (immediate reaction)
        if left < self.config['emergency_threshold'] or right < self.config['emergency_threshold']:
            return 'emergency', 0
        
        # 2. Filtered avoidance check
        left_blocked = self.left_hits >= self.config['required_hits']
        right_blocked = self.right_hits >= self.config['required_hits']
        
        if left_blocked and right_blocked:
            # Both blocked - turn towards whichever side is currently more open
            return 'avoid', 1 if left < right else -1
        elif left_blocked:
            return 'avoid', -1 # Turn right to avoid left obstacle
        elif right_blocked:
            return 'avoid', 1  # Turn left to avoid right obstacle
        
        return 'clear', 0
    
    def get_avoidance_command(self, current_heading):
        """Get drive commands for obstacle avoidance."""
        status, direction = self.check_obstacles()
        
        if not self.is_avoiding and status == 'avoid':
            self.is_avoiding = True
            self.avoid_direction = direction
            self.node.get_logger().warn(f"Obstacle detected! Avoiding (Dir: {self.avoid_direction})")
        
        # If we are in avoidance mode, keep turning in the chosen direction
        angular_speed = clamp(
            self.config['avoidance_angular'] * self.avoid_direction,
            -self.robot.max_angular_speed, 
            self.robot.max_angular_speed
        )
        
        return self.config['avoidance_linear'], angular_speed
    
    def check_cleared(self):
        """Check if obstacles have been cleared for enough time."""
        # Must be clear on both sides and have no pending hits
        left_clear = self.left_distance > self.config['obstacle_threshold'] * self.config['clearance_multiplier']
        right_clear = self.right_distance > self.config['obstacle_threshold'] * self.config['clearance_multiplier']
        no_pending_hits = (self.left_hits == 0 and self.right_hits == 0)
        
        if self.is_avoiding and left_clear and right_clear and no_pending_hits:
            self.node.get_logger().info("Obstacle cleared, resuming path.")
            self.is_avoiding = False
            self.avoid_direction = 0
            return True
        
        return not self.is_avoiding
    
    def apply_speed_reduction(self, linear_speed):
        """Slow down when getting close to an obstacle."""
        min_dist = min(self.left_distance, self.right_distance)
        if min_dist < self.config['obstacle_threshold'] * 1.5:
            factor = clamp(min_dist / (self.config['obstacle_threshold'] * 1.5), 0.2, 1.0)
            return linear_speed * factor
        return linear_speed

    def emergency_stop(self):
        current_time = time.time()
        if current_time - self.last_emergency_time > 1.0:
            self.node.get_logger().error("EMERGENCY STOP - Obstacle too close!")
            self.robot.drive(0.0, 0.0, 0.0)
            self.last_emergency_time = current_time
            return True
        return False

    def reset(self):
        self.is_avoiding = False
        self.avoid_direction = 0
        self.left_hits = 0
        self.right_hits = 0
