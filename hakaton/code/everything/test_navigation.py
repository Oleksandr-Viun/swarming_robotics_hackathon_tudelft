import time
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from library.robot_big import Robot
from library.utils import angle_difference, clamp
from obstacle_avoidance_straightvectortest import ObstacleAvoider

ODOM_TOPIC = "/mirte_base_controller/odom"

def yaw_from_quaternion(q) -> float:
    """Extract yaw from a ROS quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class LocalNavigator(Node):
    """
    A standalone navigator for testing movement and obstacle avoidance.
    Uses internal odometry (/odom) instead of server ground-truth.
    """
    def __init__(self, robot: Robot):
        super().__init__("local_navigator_test")
        self.robot = robot
        
        # State
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0
        self.odom_received = False
        
        # Initialize obstacle avoider
        self.avoider = ObstacleAvoider(self, robot)
        
        # Odometry subscriber
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)
        
        self.get_logger().info("Local Navigator initialized. Waiting for odometry...")

    def _odom_cb(self, msg: Odometry):
        """Update self-position and heading from odometry."""
        # Note: Depending on the robot's setup, /odom might start at (0,0) 
        # when the node launches, or it might be absolute to the world frame.
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.heading = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def wait_for_odom(self):
        """Block until the first odometry message is received."""
        while rclpy.ok() and not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Record the starting position AND heading so we can compute relative targets
        self.start_x = self.pos_x
        self.start_y = self.pos_y
        self.start_heading = self.heading
        self.get_logger().info(f"Odom received! Starting at: X={self.start_x:.2f}, Y={self.start_y:.2f}, Heading={self.start_heading:.2f} rad")

    def drive_relative(self, rel_x: float, rel_y: float):
        """
        Drive to a coordinate relative to the STARTING position of the robot.
        e.g., rel_x=2.0 means drive 2 meters 'forward' from exactly where it started.
        """
        # Transform the relative (forward/left) coordinates into global odom coordinates
        # based on the direction the robot was facing when the script started.
        target_x = self.start_x + (rel_x * math.cos(self.start_heading)) - (rel_y * math.sin(self.start_heading))
        target_y = self.start_y + (rel_x * math.sin(self.start_heading)) + (rel_y * math.cos(self.start_heading))
        
        self.get_logger().info(f"Target set: X={target_x:.2f}, Y={target_y:.2f} (Relative: {rel_x}, {rel_y})")
        self.avoider.reset()
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            
            # Check distance to target
            dx = target_x - self.pos_x
            dy = target_y - self.pos_y
            distance = math.hypot(dx, dy)
            
            if distance < self.robot.linear_threshold:
                self.get_logger().info("Reached target!")
                break
            
            # Check for obstacles
            obstacle_status, _ = self.avoider.check_obstacles()
            
            if obstacle_status == 'emergency':
                self.avoider.emergency_stop()
                time.sleep(0.1)
                continue
                
            elif obstacle_status == 'avoid':
                # Obstacle avoidance mode
                linear_speed, angular_speed = self.avoider.get_avoidance_command(self.heading)
                self.robot.drive(linear_x=linear_speed, linear_y=0.0, angular_z=angular_speed)
                time.sleep(self.robot.poll_freq)
                
                # Check if we've cleared the obstacle
                if self.avoider.check_cleared():
                    self.get_logger().info("Obstacle cleared, recalculating path to target.")
                continue
                
            else:
                # Clear path - normal navigation
                target_heading = math.atan2(dy, dx)
                error = angle_difference(target_heading, self.heading)
                
                # Calculate speeds
                linear_speed = self.robot.clamp_linear_speed(
                    self.robot.kp_linear * distance, 0.0, self.robot.max_linear_speed
                )
                angular_speed = clamp(
                    self.robot.kp_angular * error,
                    -self.robot.max_angular_speed, self.robot.max_angular_speed
                )
                
                # Slow down if pointing away from target
                if abs(error) > self.robot.stop_driving_forward_angle:
                    linear_speed = 0.0
                elif abs(error) > self.robot.slower_driving_forward_angle:
                    linear_speed *= 0.2
                
                # Apply speed reduction based on obstacles
                linear_speed = self.avoider.apply_speed_reduction(linear_speed)
                
                self.robot.drive(linear_x=linear_speed, linear_y=0.0, angular_z=angular_speed)
                time.sleep(self.robot.poll_freq)
        
        # Stop at target position
        self.robot.drive(0.0, 0.0, 0.0)

def main():
    rclpy.init()
    
    robot = Robot()
    time.sleep(2.0) # Let hardware settle
    
    navigator = LocalNavigator(robot)
    
    try:
        navigator.wait_for_odom()
        
        # =====================================================================
        # TEST SCENARIO
        # Place the robot facing forward.
        # Place an obstacle 1 meter directly in front of it.
        # Place the "bucket" 2.5 meters in front of the robot.
        # =====================================================================
        print("\n--- Starting Test ---")
        print("Driving 2.5 meters forward (relative +X)...")
        print("Please place an obstacle in the way to test avoidance.")
        
        # Drive 2.5m forward, 0.0m sideways
        navigator.drive_relative(rel_x=2.5, rel_y=0.0)
        
        print("\n--- Test Complete ---")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        robot.drive(0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
