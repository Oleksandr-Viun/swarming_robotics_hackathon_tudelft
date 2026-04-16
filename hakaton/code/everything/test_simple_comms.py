import time
import rclpy
import struct
import math
import sys
import select
from library.robot_big import Robot
from library.communication import Communication
import library.utils as utils
from navigate_master import GlobalNavigator

def main():
    rclpy.init()
    
    # Master Credentials
    team_id = 3
    robot_id = 0
    password = "stagnant-attractor-companion"
    host = "172.18.0.2:8000"

    print("--- SIMPLE COMMS & NAV TEST ---")
    robot = Robot()
    comm = Communication(host, team_id, robot_id, password)
    
    # Initialize Navigator (handles mirroring and A* planner)
    nav = GlobalNavigator(robot, comm)
    
    # Storage for the last objective found by a Pioneer
    last_objective = None

    def on_map_update(from_team_id, from_robot_id, internal_type, data):
        if internal_type == 5:
            try:
                row, col = struct.unpack("<HH", data)
                # Update the Master's internal map
                nav.planner.set_cell_free(row, col)
                print(f"[MAP] Pioneer {from_robot_id} explored cell: ({row}, {col})")
            except:
                print(f"[ERROR] Failed to unpack map update from {from_robot_id}")

    def on_objective_update(team_id, robot_id, tag_id, x, y, angle, visible, last_seen):
        nonlocal last_objective
        if team_id == 3:
            # Mirror the X coordinate for the Master's map
            mx = 6.0 - x
            last_objective = (mx, y)
            print(f"\n[OBJ] Team 3 Objective Found! Robot {robot_id} is at mirrored ({mx:.2f}, {y:.2f})")
            print("      --> Type 'go' and press Enter to navigate there.")

    # Register callbacks
    comm.register_callback_custom(on_map_update)
    comm.register_callback_objective(on_objective_update)
    
    print("\n[*] Waiting for location and Pioneer data...")
    nav.wait_for_location()
    
    print("\nCommands:")
    print("  'go'    - Navigate to the last reported objective")
    print("  'test'  - Attempt to go to (3.0, 4.0) using whatever map data is currently known")
    print("  'q'     - Quit")
    
    try:
        while rclpy.ok():
            # Keep ROS spinning to receive callbacks
            rclpy.spin_once(nav, timeout_sec=0.1)
            
            # Check for terminal input
            if select.select([sys.stdin], [], [], 0.0)[0]:
                cmd = sys.stdin.readline().strip().lower()
                
                if cmd == 'go':
                    if last_objective:
                        print(f"[*] Starting Nav to Objective: {last_objective}")
                        success = nav.navigate_to(last_objective[0], last_objective[1], timeout=120)
                        print(f"[*] Result: {'SUCCESS' if success else 'FAILED'}")
                    else:
                        print("[!] No objective has been reported yet.")
                
                elif cmd == 'test':
                    print("[*] Starting Test Nav to (3.0, 4.0)")
                    success = nav.navigate_to(3.0, 4.0, timeout=120)
                    print(f"[*] Result: {'SUCCESS' if success else 'FAILED'}")
                
                elif cmd == 'q':
                    break

    except KeyboardInterrupt:
        pass
    finally:
        robot.drive(0.0, 0.0, 0.0)
        rclpy.shutdown()

if __name__ == "__main__":
    main()
