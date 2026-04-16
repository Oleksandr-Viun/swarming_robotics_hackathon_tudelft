import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from library.utils import clamp


class ObstacleAvoider:
    """Handles obstacle detection and avoidance using ultrasonic sensors."""
    
    def __init__(self, node: Node, robot, config=None):
        """
        Initialize obstacle avoider.
        
        Args:
            node: ROS2 node for creating subscribers
            robot: Robot instance for driving commands
            config: Dictionary of configuration parameters (optional)
        """
        self.node = node
        self.robot = robot
        
        # Configuration parameters with defaults
        self.config = {
            'obstacle_threshold': 0.5,      # meters - start avoiding when closer than this
            'emergency_threshold': 0.25,    # meters - stop immediately when this close
            'avoidance_angular': 0.8,       # rad/s - rotation speed when avoiding
            'avoidance_linear': 0.3,        # m/s - linear speed when avoiding
            'clearance_multiplier': 1.2,    # multiplier for obstacle clearance check
            'left_sonar_topic': '/ultrasonic_left',
            'right_sonar_topic': '/ultrasonic_right',
        }
        
        if config:
            self.config.update(config)
        
        # Sensor readings
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # State
        self.is_avoiding = False
        self.avoid_direction = 0  # 1 for left, -1 for right
        self.last_emergency_time = 0
        
        # Create subscribers
        self._setup_subscribers()
        
        self.node.get_logger().info("ObstacleAvoider initialized")
    
    def _setup_subscribers(self):
        """Setup ROS2 subscribers for ultrasonic sensors."""
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
        """Callback for left ultrasonic sensor."""
        self.left_distance = msg.range
    
    def _right_sonar_cb(self, msg: Range):
        """Callback for right ultrasonic sensor."""
        self.right_distance = msg.range
    
    def get_sensor_readings(self):
        """Get current sensor readings."""
        return {
            'left': self.left_distance,
            'right': self.right_distance
        }
    
    def check_obstacles(self):
        """
        Check sensor readings and determine if avoidance is needed.
        
        Returns:
            tuple: (status, direction)
                status: 'emergency', 'avoid', or 'clear'
                direction: 1 (turn left), -1 (turn right), or 0
        """
        # Get current distances with sanity check
        left = self.left_distance if self.left_distance < 10 else float('inf')
        right = self.right_distance if self.right_distance < 10 else float('inf')
        
        # Emergency stop check
        if left < self.config['emergency_threshold'] or right < self.config['emergency_threshold']:
            return 'emergency', 0
        
        # Check if we need to avoid obstacles
        if left < self.config['obstacle_threshold'] and right < self.config['obstacle_threshold']:
            # Both sides blocked - turn towards clearer side
            return 'avoid', 1 if left < right else -1
        elif left < self.config['obstacle_threshold']:
            # Left side blocked - turn right
            return 'avoid', -1
        elif right < self.config['obstacle_threshold']:
            # Right side blocked - turn left
            return 'avoid', 1
        else:
            return 'clear', 0
    
    def get_avoidance_command(self, current_heading):
        """
        Get drive commands for obstacle avoidance.
        
        Args:
            current_heading: Current robot heading in radians
        
        Returns:
            tuple: (linear_speed, angular_speed)
        """
        if not self.is_avoiding:
            self.is_avoiding = True
            status, direction = self.check_obstacles()
            self.avoid_direction = direction if status == 'avoid' else 0
        
        # Calculate avoidance speeds
        angular_speed = clamp(
            self.config['avoidance_angular'] * self.avoid_direction,
            -self.robot.max_angular_speed, 
            self.robot.max_angular_speed
        )
        
        linear_speed = self.config['avoidance_linear']
        
        return linear_speed, angular_speed
    
    def check_cleared(self):
        """
        Check if obstacles have been cleared.
        
        Returns:
            bool: True if no obstacles detected, False otherwise
        """
        left_clear = self.left_distance > self.config['obstacle_threshold'] * self.config['clearance_multiplier']
        right_clear = self.right_distance > self.config['obstacle_threshold'] * self.config['clearance_multiplier']
        
        if left_clear and right_clear and self.is_avoiding:
            self.node.get_logger().debug("Obstacle cleared")
            self.is_avoiding = False
            return True
        
        return not self.is_avoiding
    
    def apply_speed_reduction(self, linear_speed):
        """
        Apply speed reduction based on obstacle proximity.
        
        Args:
            linear_speed: Desired linear speed
        
        Returns:
            float: Adjusted linear speed
        """
        min_distance = min(self.left_distance, self.right_distance)
        
        if min_distance < self.config['obstacle_threshold'] * 1.5:
            # Reduce speed proportionally to distance
            speed_factor = min_distance / (self.config['obstacle_threshold'] * 1.5)
            adjusted_speed = linear_speed * max(0.3, speed_factor)
            self.node.get_logger().debug(
                f"Speed reduction: {linear_speed:.2f} -> {adjusted_speed:.2f} "
                f"(distance: {min_distance:.2f}m)"
            )
            return adjusted_speed
        
        return linear_speed
    
    def emergency_stop(self):
        """Execute emergency stop and return time since last stop."""
        current_time = time.time()
        
        # Prevent spamming emergency stops
        if current_time - self.last_emergency_time > 0.5:
            self.node.get_logger().warn("Emergency stop - obstacle too close!")
            self.robot.drive(0.0, 0.0, 0.0)
            self.last_emergency_time = current_time
            return True
        
        return False
    
    def recovery_maneuver(self):
        """Execute a recovery maneuver when stuck."""
        self.node.get_logger().warn("Executing recovery maneuver")
        
        # Back up
        self.robot.drive(linear_x=-0.3, linear_y=0.0, angular_z=0.0)
        time.sleep(1.0)
        
        # Turn around
        self.robot.drive(linear_x=0.0, linear_y=0.0, angular_z=0.8)
        time.sleep(1.5)
        
        # Stop
        self.robot.drive(0.0, 0.0, 0.0)
        time.sleep(0.5)
        
        # Reset avoidance state
        self.is_avoiding = False
        self.avoid_direction = 0
    
    def reset(self):
        """Reset avoidance state."""
        self.is_avoiding = False
        self.avoid_direction = 0
    
    def get_obstacle_status(self):
        """
        Get human-readable obstacle status.
        
        Returns:
            dict: Status information
        """
        status, direction = self.check_obstacles()
        return {
            'status': status,
            'avoid_direction': direction,
            'is_avoiding': self.is_avoiding,
            'left_distance': self.left_distance,
            'right_distance': self.right_distance,
            'left_clear': self.left_distance > self.config['obstacle_threshold'],
            'right_clear': self.right_distance > self.config['obstacle_threshold']
        }