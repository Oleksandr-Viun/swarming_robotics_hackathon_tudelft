import time
import rclpy
from library.robot_big import Robot

def test_movement(robot: Robot, duration: float, linear_x: float, linear_y: float, angular_z: float, description: str):
    print(f"Executing: {description} (Vx: {linear_x}, Vy: {linear_y}, Wz: {angular_z})")
    
    # Send the movement command
    robot.drive(linear_x=linear_x, linear_y=linear_y, angular_z=angular_z)
    
    # Let it drive for the specified duration
    time.sleep(duration)
    
    # Stop the robot completely before the next movement
    robot.drive(0.0, 0.0, 0.0)
    time.sleep(1.0) # Pause to let momentum settle

def main():
    rclpy.init()
    
    # Initialize the master robot interface
    robot = Robot()
    time.sleep(2.0) # Wait for ROS publishers to establish connections

    print("\n=======================================================")
    print("           MECANUM KINEMATICS TEST SUITE               ")
    print("=======================================================")
    print("Please ensure the robot has ~2x2 meters of clear space.")
    print("Press Ctrl+C at any time to emergency stop.")
    print("Starting in 3 seconds...\n")
    time.sleep(3.0)
    
    try:
        # Phase 1: The Strafe Square (approx 2 seconds per side)
        # The robot should trace a square without rotating its body.
        print("\n--- Phase 1: The Strafe Square (Constant Heading) ---")
        test_movement(robot, 2.0, linear_x=0.2, linear_y=0.0, angular_z=0.0, description="Forward")
        test_movement(robot, 2.0, linear_x=0.0, linear_y=0.2, angular_z=0.0, description="Strafe Left")
        test_movement(robot, 2.0, linear_x=-0.2, linear_y=0.0, angular_z=0.0, description="Backward")
        test_movement(robot, 2.0, linear_x=0.0, linear_y=-0.2, angular_z=0.0, description="Strafe Right")

        # Phase 2: Diagonal Movement
        # The robot should move at a 45-degree angle.
        print("\n--- Phase 2: Diagonal Movement ---")
        test_movement(robot, 2.0, linear_x=0.2, linear_y=0.2, angular_z=0.0, description="Diagonal Forward-Left")
        test_movement(robot, 2.0, linear_x=-0.2, linear_y=-0.2, angular_z=0.0, description="Diagonal Backward-Right (Return)")

        # Phase 3: Orbiting (Strafe while rotating)
        # The robot should circle around a point while facing the center.
        print("\n--- Phase 3: Orbiting ---")
        # Moving left (Vy > 0) while turning right (Wz < 0) orbits a point to the right.
        test_movement(robot, 5.0, linear_x=0.0, linear_y=0.2, angular_z=-0.3, description="Orbiting a point to the right")

        print("\n=======================================================")
        print("                   TEST COMPLETE                       ")
        print("=======================================================")

    except KeyboardInterrupt:
        print("\n[!] Test interrupted by user. Stopping robot immediately.")
    except Exception as e:
        print(f"\n[!] An error occurred: {e}")
    finally:
        # Emergency stop ensure
        robot.drive(0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
