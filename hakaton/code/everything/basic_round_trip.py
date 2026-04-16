import time
import math
import rclpy
from rclpy.node import Node
from library.robot_big import Robot
from library.communication import Communication
from library.utils import angle_difference, clamp

class BasicRoundTrip(Node):
    """
    A simple script to test global positioning:
    1. Record starting position from camera.
    2. Drive forward relative to start.
    3. Return home using global camera coordinates.
    """
    def __init__(self, robot: Robot, comm: Communication):
        super().__init__("basic_round_trip")
        self.robot = robot
        self.comm = comm
        
        self.pos_x = 0.0
        self.pos_y = 0.0E
        self.heading = 0.0
        self.visible = False
        
        self.comm.register_callback_location(self._location_cb)

    def _location_cb(self, x, y, angle, visible, last_seen):
        self.pos_x = x
        self.pos_y = y
        self.heading = angle
        self.visible = visible

    def wait_for_location(self):
        print("[*] Waiting for camera to see robot...")
        while rclpy.ok() and not self.visible:
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f"[OK] Position locked: ({self.pos_x:.2f}, {self.pos_y:.2f})")
        return self.pos_x, self.pos_y

    def drive_to_global(self, target_x, target_y, label="Target"):
        print(f"\n--- Phase: {label} ---")
        print(f"Goal: ({target_x:.2f}, {target_y:.2f})")
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            
            if not self.visible:
                self.robot.drive(0.0, 0.0, 0.0)
                time.sleep(0.1)
                continue
                
            dx = target_x - self.pos_x
            dy = target_y - self.pos_y
            distance = math.hypot(dx, dy)
            
            if distance < 0.15: # Arrival threshold
                print(f"[SUCCESS] Reached {label}!")
                break
                
            # Holonomic Vector Calculation (Mecanum)
            global_target_angle = math.atan2(dy, dx)
            # Use -self.heading because camera orientation is typically inverted for drive
            relative_angle = angle_difference(global_target_angle, -self.heading)
            
            # Simple proportional speed
            speed = clamp(0.6 * distance, 0.1, 0.3)
            
            # Project to local robot axes
            vx = speed * math.cos(relative_angle)
            vy = speed * math.sin(relative_angle)
            
            # Maintain 0 rad heading (facing forward)
            angular_z = clamp(1.5 * angle_difference(0.0, -self.heading), -0.5, 0.5)
            
            self.robot.drive(linear_x=vx, linear_y=vy, angular_z=angular_z)
            time.sleep(0.01)
            
        self.robot.drive(0.0, 0.0, 0.0)

def main():
    rclpy.init()
    
    robot = Robot()
    time.sleep(2.0)
    
    # Credentials for Team 3, Robot 0
    team_id = 3
    robot_id = 0
    password = "stagnant-attractor-companion"
    host = "172.18.0.2:8000"
    
    comm = Communication(host, team_id, robot_id, password)
    mover = BasicRoundTrip(robot, comm)
    
    try:
        # 1. Record Start (Home)
        hx, hy = mover.wait_for_location()
        
        # 2. Drive 1.0m Forward (Global coordinates)
        # Note: We drive to hx + 1.0, hy
        mover.drive_to_global(hx + 1.0, hy, label="Away Point")
        
        time.sleep(1.0)
        
        # 3. Drive back to exact Home coordinates
        mover.drive_to_global(hx, hy, label="Home Base")
        
        print("\n[DONE] Successfully returned home using global coordinates.")
            
    except KeyboardInterrupt:
        print("\n[!] Stopped by user.")
    finally:
        robot.drive(0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    main()