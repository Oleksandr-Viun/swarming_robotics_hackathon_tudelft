import time
import math
import rclpy
from rclpy.node import Node
from library.robot_big import Robot
from library.utils import angle_difference, clamp
from library.communication import Communication

class GlobalMover(Node):
    """
    A bare-bones movement script that uses the overhead camera server 
    for absolute ground-truth positioning instead of odometry.
    """
    def __init__(self, robot: Robot, comm: Communication):
        super().__init__("global_mover")
        self.robot = robot
        self.comm = comm
        
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0
        self.visible = False
        
        # Register the location callback with the server
        self.comm.register_callback_location(self._location_cb)
        self.get_logger().info("Waiting for global location from overhead camera...")

    def _location_cb(self, x, y, angle, visible, last_seen):
        self.pos_x = x
        self.pos_y = y
        self.heading = angle
        self.visible = visible

    def wait_for_location(self):
        """Block until the overhead camera sees the robot."""
        while rclpy.ok() and not self.visible:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        self.get_logger().info("Global location locked in! The server sees the robot.")

    def drive_relative(self, forward_m: float, left_m: float):
        """
        Drives the robot a specific distance forward and left relative 
        to its CURRENT global position and orientation.
        """
        # 1. Snapshot the current global position and inverted heading
        start_x = self.pos_x
        start_y = self.pos_y
        start_heading = -self.heading
        
        self.get_logger().info(f"Current Pose -> X:{start_x:.2f}, Y:{start_y:.2f}, Yaw:{math.degrees(start_heading):.1f}°")
        
        # 2. Calculate the exact world coordinate we want to end up at
        # +X in robot frame is forward, +Y in robot frame is left
        target_x = start_x + (forward_m * math.cos(start_heading)) - (left_m * math.sin(start_heading))
        target_y = start_y + (forward_m * math.sin(start_heading)) + (left_m * math.cos(start_heading))
        
        self.get_logger().info(f"Driving to global Target -> X:{target_x:.2f}, Y:{target_y:.2f}")
        
        last_log_time = time.time()
        
        # 3. Drive Loop
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            
            # Print coordinates every 0.5s
            current_time = time.time()
            if current_time - last_log_time > 0.5:
                self.get_logger().info(f"LIVE POS -> X: {self.pos_x:.2f}, Y: {self.pos_y:.2f}, Yaw: {math.degrees(self.heading):.1f}°")
                last_log_time = current_time

            # Safety Check: If the camera loses us, stop and wait!
            if not self.visible:
                self.robot.drive(0.0, 0.0, 0.0)
                time.sleep(0.1)
                continue
                
            # Distance traveled from start
            dx = self.pos_x - start_x
            dy = self.pos_y - start_y
            traveled = math.hypot(dx, dy)
            
            distance_remaining = forward_m - traveled
            
            if distance_remaining < self.robot.linear_threshold: # Arrived! (within 10cm by default)
                self.get_logger().info(f"Reached Target! Total traveled: {traveled:.3f}m")
                break
                
            # Drive straight forward. No steering correction!
            linear_speed = self.robot.clamp_linear_speed(
                self.robot.kp_linear * distance_remaining, 0.0, self.robot.max_linear_speed
            )
            
            self.robot.drive(linear_x=linear_speed, linear_y=0.0, angular_z=0.0)
            time.sleep(self.robot.poll_freq)
            
        # 4. Full Stop
        self.robot.drive(0.0, 0.0, 0.0)


def main():
    rclpy.init()
    
    robot = Robot()
    time.sleep(2.0) # Let the hardware settle
    
    import library.utils as utils
    
    # Hardcode the known working credentials for this specific robot
    try:
        team_id = 3
        robot_id = 0
        password = "stagnant-attractor-companion"
        print(f"Loaded hardcoded credentials: Team {team_id}, Robot {robot_id}")
    except Exception as e:
        print(f"Failed to load credentials: {e}")
        return
        
    host = "172.18.0.2:8000"
    
    print(f"Connecting to server {host}...")
    comm = Communication(host, team_id, robot_id, password)
    
    mover = GlobalMover(robot, comm)
    
    try:
        mover.wait_for_location()
        
        print("\n--- Starting Global Movement Test ---")
        
        print("1. Driving 1.5 meters FORWARD...")
        mover.drive_relative(forward_m=1.5, left_m=0.0)
        
        print("\nPausing for 2 seconds...")
        time.sleep(2.0)
        
        # Uncomment this next block when you are ready to test the 1 meter left movement!
        # print("\n2. Driving 1.0 meter LEFT...")
        # mover.drive_relative(forward_m=0.0, left_m=1.0)
        
        print("\n--- Test Complete ---")
        
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        robot.drive(0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
