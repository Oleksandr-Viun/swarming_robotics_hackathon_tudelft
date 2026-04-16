import time
import math
import rclpy
from rclpy.node import Node
from library.robot_big import Robot
from library.communication import Communication
from library.utils import angle_difference, clamp
from obstacle_avoidance_straightvectortest import ObstacleAvoider

class WaypointNavigator(Node):
    """
    Navigates the Master robot to global waypoints using server location data
    and integrated local obstacle avoidance (ultrasonic sensors).
    """
    def __init__(self, robot: Robot, comm: Communication):
        super().__init__("waypoint_navigator")
        self.robot = robot
        self.comm = comm
        
        # State
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0
        self.visible = False
        
        # Mirroring (Set to True if your target coordinates are mirrored)
        self.mirror_x = False 
        self.arena_width = 6.0
        
        # Initialize obstacle avoidance
        self.avoider = ObstacleAvoider(self, robot)
        
        # Register the location callback
        self.comm.register_callback_location(self._location_cb)
        
        self.get_logger().info("Waypoint Navigator with Obstacle Avoidance initialized.")

    def _location_cb(self, x, y, angle, visible, last_seen):
        """Processes raw camera data and handles coordinate mirroring."""
        if self.mirror_x:
            self.pos_x = self.arena_width - x
            self.heading = math.pi - angle
        else:
            self.pos_x = x
            self.heading = angle
        
        self.pos_y = y
        self.visible = visible

    def wait_for_location(self):
        """Block until the overhead camera sees the robot."""
        self.get_logger().info("Waiting for camera tracking...")
        while rclpy.ok() and not self.visible:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Location locked: ({self.pos_x:.2f}, {self.pos_y:.2f})")

    def navigate_to(self, target_x, target_y, timeout=180.0):
        """
        Drives to a target coordinate using holonomic movement while checking for obstacles.
        """
        start_time = time.time()
        self.get_logger().info(f"--- Starting Navigation to ({target_x:.2f}, {target_y:.2f}) ---")
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            now = time.time()
            
            # 1. Timeout & Visibility Checks
            if now - start_time > timeout:
                self.get_logger().error("Navigation Timed Out!")
                break
                
            if not self.visible:
                self.robot.drive(0.0, 0.0, 0.0)
                time.sleep(0.1)
                continue
            
            # 2. Obstacle Avoidance Check
            status, _ = self.avoider.check_obstacles()
            
            if status == 'emergency':
                self.avoider.emergency_stop()
                time.sleep(0.1)
                continue
                
            elif status == 'avoid':
                # OBSTACLE MODE: The avoider takes control of rotation
                vx_avoid, wz_avoid = self.avoider.get_avoidance_command(self.heading)
                self.robot.drive(linear_x=vx_avoid, linear_y=0.0, angular_z=wz_avoid)
                time.sleep(self.robot.poll_freq)
                continue

            # 3. Pathing Logic (Target following)
            dx = target_x - self.pos_x
            dy = target_y - self.pos_y
            distance = math.hypot(dx, dy)
            
            if distance < 0.15: # Reached Target
                self.get_logger().info("Target Reached!")
                break
            
            # Holonomic direction calculation
            global_angle = math.atan2(dy, dx)
            # Use -self.heading because camera orientation is typically inverted for drive
            relative_angle = angle_difference(global_angle, -self.heading)
            
            # Speeds
            speed = self.robot.clamp_linear_speed(0.6 * distance, 0.0, 0.3)
            # Apply reduction if close to obstacle even if not in 'avoid' mode yet
            speed = self.avoider.apply_speed_reduction(speed)
            
            vx = speed * math.cos(relative_angle)
            vy = speed * math.sin(relative_angle)
            
            # Rotation lock (maintain 0 rad mirrored or whatever the start heading was)
            # To face target instead, use relative_angle here.
            angular_err = angle_difference(0.0, -self.heading)
            wz = clamp(1.5 * angular_err, -0.6, 0.6)
            
            self.robot.drive(linear_x=vx, linear_y=vy, angular_z=wz)
            time.sleep(self.robot.poll_freq)
            
        self.robot.drive(0.0, 0.0, 0.0)

def main():
    rclpy.init()
    
    robot = Robot()
    time.sleep(2.0)
    
    # Credentials
    team_id, robot_id = 3, 0
    password = "stagnant-attractor-companion"
    host = "172.18.0.2:8000"
    
    comm = Communication(host, team_id, robot_id, password)
    nav = WaypointNavigator(robot, comm)
    
    try:
        nav.wait_for_location()
        
        # TARGET: Move to 1 meter in front of the mirrored starting position
        # Or set to absolute coordinates like (4.5, 3.0)
        nav.navigate_to(target_x=4.5, target_y=3.0)
        
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        robot.drive(0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
