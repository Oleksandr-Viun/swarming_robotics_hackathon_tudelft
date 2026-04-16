import time
import rclpy
from library.robot_big import Robot

def drive_for_duration(robot, linear_x, linear_y, angular_z, duration_seconds):
    print(f"Sending: x={linear_x}, y={linear_y}, z={angular_z} for {duration_seconds}s")
    start_time = time.time()
    while time.time() - start_time < duration_seconds:
        robot.drive(linear_x=linear_x, linear_y=linear_y, angular_z=angular_z)
        time.sleep(0.05) 
    robot.drive(0.0, 0.0, 0.0)

def main():
    rclpy.init()
    print("--- BALANCED DIRECTIONAL TEST ---")
    robot = Robot()
    time.sleep(2.0)
    
    try:
        # TURN TEST
        print("\n1. TURN LEFT (1.5)...")
        drive_for_duration(robot, 0.0, 0.0, 1.5, 2.0)
        time.sleep(1.0)
        
        print("\n2. TURN RIGHT (-1.5)...")
        drive_for_duration(robot, 0.0, 0.0, -1.5, 2.0)
        time.sleep(2.0)

        # STRAFE TEST (Mecanum only)
        print("\n3. STRAFE LEFT (y = 0.6)...")
        drive_for_duration(robot, 0.0, 0.6, 0.0, 2.0)
        time.sleep(1.0)
        
        print("\n4. STRAFE RIGHT (y = -0.6)...")
        drive_for_duration(robot, 0.0, -0.6, 0.0, 2.0)
        
        print("\nDone. Compare the Left vs Right movement for each.")
        
    except KeyboardInterrupt:
        pass
    finally:
        robot.drive(0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
