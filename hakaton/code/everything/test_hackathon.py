import time
import rclpy
import struct
import math
from rclpy.node import Node 
from library.robot_big import Robot
from library.detector import Detector
from library.communication import Communication
import library.utils as utils
from navigate_master import GlobalNavigator

# --- Constants for the Arm ---
ARM_PICKUP_POS = [0.0, -1.8, -1.3, 1.0]
ARM_CHECK_POS = [0.0, -0.4, -1.5, -2.0]
ARM_MOVE_POS = [1.5, 1.2, -1.0, -1.2]
ARM_DROP_POS = [0.0, 0.3, -0.8, -1.0]

#### 1. Setup ####
rclpy.init()

# Credentials for Team 3, Robot 0 (Master)
team_id = 3
robot_id = 0
password = "stagnant-attractor-companion"
host = "172.18.0.2:8000"

robot = Robot()
detector = Detector(True)
comm = Communication(host, team_id, robot_id, password)

# --- ESSENTIAL: Initialize Navigator once at the top level ---
# This ensures the safe-zone map stays persistent throughout the match.
# The AStarPlanner inside 'nav' starts with all cells blocked (True).
nav = GlobalNavigator(robot, comm)
objectives = []

###### 2. Callbacks (Essential for Scouring) #####

def on_receive_location(x, y, angle, visible, last_seen):
    # Required for the navigator to track itself
    nav._location_cb(x, y, angle, visible, last_seen)

def on_receive_objective(from_team_id, from_robot_id, tag_id, x, y, angle, visible, last_seen):
    # --- ESSENTIAL: Filter by Team 3 and Mirror the X coordinate ---
    if from_team_id == 3:
        print(f"\n[!] Team 3 Objective Found by Robot {from_robot_id}! Tag: {tag_id}")
        mirrored_x = 6.0 - x # Match the Master's mirrored map
        new_objective = {"x": mirrored_x, "y": y, "tag_id": tag_id}
        objectives.append(new_objective)

def on_receive_custom(from_team_id, from_robot_id, internal_type, data):
    # --- ESSENTIAL: Scouring logic (Type 50) ---
    # This allows Pioneers to "paint" the safe path for the Master.
    if internal_type == 50:
        try:
            # Unpack the 4-byte coordinate (row, col) from Pioneer
            row, col = struct.unpack("<HH", data)
            nav.planner.set_cell_free(row, col) # Mark cell as safe/explored in A*
        except:
            pass

# Register the essential callbacks
comm.register_callback_location(on_receive_location)
comm.register_callback_objective(on_receive_objective)
comm.register_callback_custom(on_receive_custom)

# Wait for camera
while detector.latest_image is None:
    print("no image in detector.. waiting")
    time.sleep(5)

class MainStateMachine:
    def __init__(self, robot, detector):
        self.robot = robot
        self.detector = detector
        self.state = "move_to_start" 

    def step(self):
        print(f"\n[FSM] Current State: {self.state}")
        if self.state == "move_to_start":
            self.state = move_to_start()
        elif self.state == "pickup":
            self.state = pickup()
        elif self.state == "move_to_obj":
            self.state == "dropoff" #debug
            # self.state = move_to_obj()
        elif self.state == "dropoff":
            self.state = dropoff()
        elif self.state == "exit":
            return True
        return False

##### FSM Functions #####

def pickup():
    from test_all import PickupStateMachine
    pickup_fsm = PickupStateMachine(robot, detector)
    start_time = time.time()
    while not pickup_fsm.is_ready():
        if time.time() - start_time > 120:
            print("[FSM] Pickup timed out. Returning to base.")
            return "move_to_start"
        pickup_fsm.step()
        time.sleep(0.1) 
    print("[FSM] Pickup done!")
    # Prepare arm for travel
    robot.move_arm_to(*ARM_MOVE_POS, 1)
    return "move_to_obj"

def move_to_obj():
    # --- ESSENTIAL: Wait for Pioneer report instead of hardcoding ---
    print("[FSM] Waiting for Team 3 Pioneers to report an objective location...")
    while len(objectives) == 0:
        # Keep spinning to allow the comm thread to populate the objectives list
        rclpy.spin_once(robot.node, timeout_sec=0.1)
        time.sleep(0.1)
    
    obj = objectives.pop(0)
    print(f"[FSM] Heading to objective reported by Pioneer at: mirrored ({obj['x']:.2f}, {obj['y']:.2f})")
    
    nav.wait_for_location()
    # Path is now calculated ONLY through explored/safe cells reported by Pioneers
    success = nav.navigate_to(obj["x"], obj["y"], timeout=180)
    
    if success:
        print("[FSM] Reached Objective location. Moving to dropoff.")
        return "dropoff"
    else:
        print("[FSM] Navigation failed or timed out. Returning home.")
        return "move_to_start"

def dropoff():
    from test_dropoff import DropOffStateMachine
    dropoff_fsm = DropOffStateMachine(robot, detector)
    start_time = time.time()
    while not dropoff_fsm.is_ready():
        if time.time() - start_time > 60:
            print("[FSM] Dropoff timed out.")
            return "move_to_start"
        dropoff_fsm.step()
        time.sleep(0.1)
    print("[FSM] Dropoff done!")
    # Prepare arm for travel
    robot.move_arm_to(*ARM_MOVE_POS, 1)
    return "move_to_start"

def move_to_start():
    print("[FSM] Returning to Home Base...")
    nav.wait_for_location()
    # Home base at (3.0, 0.2) in mirrored coordinates
    success = nav.navigate_to(3.0, 0.2, timeout=120)
    
    if success:
        print("[FSM] Back at base. Ready for next pickup.")
        return "pickup"
    else:
        print("[FSM] Failed to reach base. Trying again.")
        return "move_to_start"

# Run FSM
fsm = MainStateMachine(robot, detector)
try:
    while rclpy.ok():
        if fsm.step():
            break
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n[!] Interrupted by user.")
finally:
    # Shutdown nicely
    robot.drive(0.0, 0.0, 0.0)
    time.sleep(0.5)
    rclpy.shutdown()
