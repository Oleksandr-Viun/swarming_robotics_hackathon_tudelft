#!/usr/bin/env python3
"""
Safe test script for ObstacleAvoider on actual robot.
This script reads ultrasonic sensors and shows avoidance decisions without moving the robot.
"""

import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from library.robot_big import Robot
from obstacle_avoidance import ObstacleAvoider


class SafeRobotWrapper:
    """Wrapper for Robot that prevents actual movement."""
    
    def __init__(self, real_robot):
        self.real_robot = real_robot
        self.last_command = None
        self.command_history = []
        
        # Copy robot parameters
        self.max_linear_speed = real_robot.max_linear_speed
        self.max_angular_speed = real_robot.max_angular_speed
        self.kp_linear = real_robot.kp_linear
        self.kp_angular = real_robot.kp_angular
        self.linear_threshold = real_robot.linear_threshold
        self.angular_threshold = real_robot.angular_threshold
        self.stop_driving_forward_angle = real_robot.stop_driving_forward_angle
        self.slower_driving_forward_angle = real_robot.slower_driving_forward_angle
        self.poll_freq = real_robot.poll_freq
    
    def drive(self, linear_x, linear_y, angular_z):
        """Log drive commands but DON'T actually drive."""
        command = {
            'timestamp': time.time(),
            'linear_x': linear_x,
            'linear_y': linear_y,
            'angular_z': angular_z
        }
        self.last_command = command
        self.command_history.append(command)
        
        # Just print what would happen
        if linear_x != 0 or angular_z != 0:
            print(f"  → WOULD DRIVE: linear={linear_x:.2f} m/s, angular={angular_z:.2f} rad/s")
    
    def clamp_linear_speed(self, speed, min_speed, max_speed):
        """Clamp linear speed."""
        return max(min_speed, min(speed, max_speed))


class ObstacleAvoiderTestNode(Node):
    """Test node for ObstacleAvoider on actual robot without moving."""
    
    def __init__(self):
        super().__init__('obstacle_avoider_safe_test')
        
        # Create real robot but wrap it safely
        self.real_robot = Robot()
        self.safe_robot = SafeRobotWrapper(self.real_robot)
        
        # Configuration matching your robot's actual sensor topics
        config = {
            'obstacle_threshold': 0.5,      # meters
            'emergency_threshold': 0.25,    # meters
            'avoidance_angular': 0.8,       # rad/s
            'avoidance_linear': 0.3,        # m/s
            'clearance_multiplier': 1.2,
            'left_sonar_topic': '/ultrasonic_left',   # ADJUST THIS TO YOUR ACTUAL TOPIC
            'right_sonar_topic': '/ultrasonic_right', # ADJUST THIS TO YOUR ACTUAL TOPIC
        }
        
        # Create obstacle avoider with safe robot wrapper
        self.avoider = ObstacleAvoider(self, self.safe_robot, config)
        
        # Test control variables
        self.running = True
        self.manual_mode = False
        self.test_heading = 0.0  # Assume robot facing forward
        
        # Create a timer for periodic status updates
        self.create_timer(0.5, self.status_callback)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("OBSTACLE AVOIDER SAFE TEST MODE")
        self.get_logger().info("Robot will NOT move - only sensor readings and decisions shown")
        self.get_logger().info("=" * 60)
        self.get_logger().info("\nCommands:")
        self.get_logger().info("  Press 'q' + Enter to quit")
        self.get_logger().info("  Press 'm' + Enter to toggle manual testing mode")
        self.get_logger().info("  Press 'c' + Enter to clear command history")
        self.get_logger().info("  Press 'r' + Enter to reset avoider state")
        self.get_logger().info("  Press 't' + Enter to test avoidance command")
        self.get_logger().info("")
    
    def status_callback(self):
        """Print current sensor readings and avoidance status."""
        # Get current readings
        left = self.avoider.left_distance
        right = self.avoider.right_distance
        
        # Check obstacle status
        status, direction = self.avoider.check_obstacles()
        
        # Format status message
        status_icons = {
            'clear': '✅ CLEAR',
            'avoid': '⚠️ AVOID',
            'emergency': '🛑 EMERGENCY'
        }
        status_text = status_icons.get(status, status)
        
        direction_text = ""
        if direction > 0:
            direction_text = " (turn LEFT)"
        elif direction < 0:
            direction_text = " (turn RIGHT)"
        
        print("\n" + "=" * 60)
        print(f"SENSOR READINGS:")
        print(f"  Left:  {left:.3f} m" if left < 100 else f"  Left:  >2.0 m")
        print(f"  Right: {right:.3f} m" if right < 100 else f"  Right: >2.0 m")
        print(f"\nAVOIDANCE STATUS:")
        print(f"  Status: {status_text}{direction_text}")
        print(f"  Avoiding: {'YES' if self.avoider.is_avoiding else 'NO'}")
        
        # Show what action would be taken
        if status == 'clear':
            print(f"\nACTION: Drive normally")
        elif status == 'avoid':
            linear, angular = self.avoider.get_avoidance_command(self.test_heading)
            print(f"\nACTION: Avoid obstacle")
            print(f"  → Would turn at {abs(angular):.2f} rad/s")
            print(f"  → Would move forward at {linear:.2f} m/s")
        elif status == 'emergency':
            print(f"\nACTION: EMERGENCY STOP!")
        
        # Show last command if any
        if self.safe_robot.last_command:
            last = self.safe_robot.last_command
            if last['linear_x'] != 0 or last['angular_z'] != 0:
                print(f"\nLAST COMMAND (simulated):")
                print(f"  linear={last['linear_x']:.2f}, angular={last['angular_z']:.2f}")
        
        print("=" * 60)
    
    def run_manual_test(self):
        """Run manual testing with user input."""
        self.manual_mode = True
        print("\n--- MANUAL TEST MODE ---")
        print("You can manually test different obstacle scenarios:")
        print("Enter left and right distances (meters) separated by space")
        print("Example: '0.3 1.5' for left obstacle at 30cm, right clear")
        print("Type 'exit' to return to auto mode\n")
        
        while self.manual_mode and self.running:
            try:
                user_input = input("Enter distances (left right) or 'exit': ").strip()
                
                if user_input.lower() == 'exit':
                    self.manual_mode = False
                    print("Returning to auto mode...")
                    break
                
                # Parse distances
                parts = user_input.split()
                if len(parts) == 2:
                    left_dist = float(parts[0])
                    right_dist = float(parts[1])
                    
                    # Simulate sensor readings
                    print(f"\nSimulating: left={left_dist:.2f}m, right={right_dist:.2f}m")
                    
                    # Manually set sensor readings (for testing)
                    self.avoider.left_distance = left_dist
                    self.avoider.right_distance = right_dist
                    
                    # Show what would happen
                    status, direction = self.avoider.check_obstacles()
                    
                    if status == 'clear':
                        print("✅ PATH CLEAR - Would drive normally")
                    elif status == 'avoid':
                        linear, angular = self.avoider.get_avoidance_command(self.test_heading)
                        print(f"⚠️ OBSTACLE DETECTED - Would turn {'left' if direction > 0 else 'right'}")
                        print(f"   → Angular speed: {angular:.2f} rad/s")
                        print(f"   → Linear speed: {linear:.2f} m/s")
                    elif status == 'emergency':
                        print("🛑 EMERGENCY - Would stop immediately!")
                    
                    # Show if cleared
                    if self.avoider.check_cleared():
                        print("   → Obstacle cleared, would resume normal driving")
                
                else:
                    print("Please enter two numbers: left_distance right_distance")
                    
            except ValueError:
                print("Invalid input. Please enter numbers like: 0.3 1.5")
            except KeyboardInterrupt:
                break
    
    def run_auto_test(self):
        """Run automatic testing with real sensor readings."""
        print("\n--- AUTO TEST MODE ---")
        print("Reading real sensor data and showing avoidance decisions")
        print("Move obstacles in front of the robot to see reactions")
        print("Press Ctrl+C to stop\n")
        
        try:
            while self.running:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            pass
    
    def interactive_loop(self):
        """Main interactive loop."""
        print("\nStarting interactive test...")
        print("Type 'help' for commands\n")
        
        while self.running:
            try:
                # Handle user input
                if rclpy.ok():
                    # Non-blocking input check
                    import sys
                    import select
                    
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        cmd = sys.stdin.readline().strip().lower()
                        
                        if cmd == 'q' or cmd == 'quit':
                            self.running = False
                            break
                        elif cmd == 'm' or cmd == 'manual':
                            self.run_manual_test()
                        elif cmd == 'c' or cmd == 'clear':
                            self.safe_robot.clear_history()
                            print("Command history cleared")
                        elif cmd == 'r' or cmd == 'reset':
                            self.avoider.reset()
                            print("Avoider state reset")
                        elif cmd == 't' or cmd == 'test':
                            # Test avoidance command with current readings
                            status, direction = self.avoider.check_obstacles()
                            if status == 'avoid':
                                linear, angular = self.avoider.get_avoidance_command(self.test_heading)
                                print(f"Test command: linear={linear:.2f}, angular={angular:.2f}")
                            else:
                                print(f"Not in avoidance mode (status: {status})")
                        elif cmd == 'help':
                            print("\nCommands:")
                            print("  q     - Quit")
                            print("  m     - Manual test mode (simulate distances)")
                            print("  c     - Clear command history")
                            print("  r     - Reset avoider state")
                            print("  t     - Test avoidance command")
                            print("  help  - Show this help")
                        elif cmd:
                            print(f"Unknown command: {cmd}. Type 'help' for commands")
                
                # Keep spinning ROS
                rclpy.spin_once(self, timeout_sec=0.01)
                time.sleep(0.01)
                
            except KeyboardInterrupt:
                self.running = False
                break
    
    def cleanup(self):
        """Clean up resources."""
        self.get_logger().info("\nTest completed. Robot was never moved.")
        self.get_logger().info(f"Total simulated commands: {len(self.safe_robot.command_history)}")
        
        # Ensure robot is stopped (just in case)
        self.real_robot.drive(0.0, 0.0, 0.0)


def main():
    """Main entry point."""
    rclpy.init()
    
    test_node = None
    
    try:
        test_node = ObstacleAvoiderTestNode()
        print("\n" + "="*60)
        print("SAFE TEST MODE ACTIVE")
        print("The robot will NOT move during this test")
        print("="*60)
        
        # Start interactive loop
        test_node.interactive_loop()
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    finally:
        if test_node:
            test_node.cleanup()
        rclpy.shutdown()
        print("\nTest finished. Robot is safe and stationary.")


if __name__ == "__main__":
    main()