import time
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from library.robot_big import Robot
from library.utils import angle_difference, clamp

def yaw_from_quaternion(q) -> float:
    """Extract yaw from a ROS quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class OdomHelper(Node):
    """Helper class to track current odometry for alignment."""
    def __init__(self):
        super().__init__("align_odom_helper")
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0
        self.received = False
        self.create_subscription(Odometry, "/mirte_base_controller/odom", self._cb, 10)

    def _cb(self, msg: Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.heading = yaw_from_quaternion(msg.pose.pose.orientation)
        self.received = True

def align_to_objective(robot: Robot, initial_dist: float, initial_yaw_deg: float, target_distance: float = 0.2):
    """
    Aligns the robot to an objective based on a single initial reading and then 
    uses odometry to finish the maneuver.
    
    Args:
        robot: The Robot instance.
        initial_dist: Initial distance to the objective in meters.
        initial_yaw_deg: Initial yaw angle to the objective in degrees.
        target_distance: Final desired distance to stop at (meters).
    """
    # 1. Initialize Odom Helper to get current state
    helper = OdomHelper()
    robot.node.get_logger().info("Waiting for odometry for alignment...")
    while rclpy.ok() and not helper.received:
        rclpy.spin_once(helper, timeout_sec=0.1)

    if initial_dist is None or initial_yaw_deg is None:
        robot.node.get_logger().error("No objective detected! Alignment aborted.")
        return

    # Convert degrees to radians
    initial_yaw_rad = math.radians(initial_yaw_deg)
    
    robot.node.get_logger().info(
        f"Objective localized: Dist={initial_dist:.2f}m, Yaw={initial_yaw_deg:.1f}deg. "
        "Transitioning to odometry-based alignment..."
    )

    # 3. Calculate the target world coordinate relative to the current pose
    # The objective is at (initial_dist) distance and (initial_yaw_rad) angle from current heading
    world_target_heading = helper.heading + initial_yaw_rad
    target_x = helper.pos_x + (initial_dist * math.cos(world_target_heading))
    target_y = helper.pos_y + (initial_dist * math.sin(world_target_heading))
    
    # Adjust target_x and target_y to stop 'target_distance' AWAY from the object
    # The new distance to travel is (initial_dist - target_distance)
    travel_dist = initial_dist - target_distance
    
    # Recalculate actual stop point (so we don't hit the bucket)
    stop_x = helper.pos_x + (travel_dist * math.cos(world_target_heading))
    stop_y = helper.pos_y + (travel_dist * math.sin(world_target_heading))

    # 4. Navigation Loop (Odometry only)
    while rclpy.ok():
        rclpy.spin_once(helper, timeout_sec=0)
        
        # Calculate distance error
        dx = stop_x - helper.pos_x
        dy = stop_y - helper.pos_y
        distance_error = math.hypot(dx, dy)
        
        if distance_error < robot.linear_threshold:
            break
            
        # Calculate heading to stop point
        # We actually want to face the OBJECT (world_target_heading), 
        # not necessarily the stop point
        heading_error = angle_difference(world_target_heading, helper.heading)
        
        # Check if we are close enough to stop
        if distance_error < 0.05 and abs(heading_error) < robot.angular_threshold:
            break
            
        # Calculate speeds
        angular_speed = clamp(
            robot.kp_angular * heading_error, 
            -robot.max_angular_speed, 
            robot.max_angular_speed
        )
        
        linear_speed = robot.clamp_linear_speed(
            robot.kp_linear * distance_error, 
            0.0, 
            robot.max_linear_speed
        )
        
        # Priority on turning
        if abs(heading_error) > robot.stop_driving_forward_angle:
            linear_speed = 0.0
        elif abs(heading_error) > robot.slower_driving_forward_angle:
            linear_speed *= 0.2
            
        robot.drive(linear_x=linear_speed, linear_y=0.0, angular_z=angular_speed)
        time.sleep(robot.poll_freq)
        
    # 5. Final fine-tuning rotation (Ensure we face it perfectly)
    while rclpy.ok():
        rclpy.spin_once(helper, timeout_sec=0)
        heading_error = angle_difference(world_target_heading, helper.heading)
        if abs(heading_error) < robot.angular_threshold:
            break
        angular_speed = clamp(robot.kp_angular * heading_error, -0.4, 0.4) # slower fine-tuning
        robot.drive(0.0, 0.0, angular_speed)
        time.sleep(robot.poll_freq)

    robot.drive(0.0, 0.0, 0.0)
    robot.node.get_logger().info("Odometry alignment complete.")

def main():
    rclpy.init()
    robot = Robot()
    
    try:
        # Mock reading for demonstration: 1.0m away, 30 degrees to the left
        align_to_objective(robot, initial_dist=1.0, initial_yaw_deg=30.0, target_distance=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        robot.drive(0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
