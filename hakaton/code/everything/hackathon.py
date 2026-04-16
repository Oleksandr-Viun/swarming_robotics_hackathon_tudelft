import time
import rclpy
from rclpy.node import Node 
import math
from sensor_msgs.msg import Image
from enum import Enum
import struct
import numpy as np
import threading

from library.robot_big import Robot
from library.detector import Detector
from library.communication import Communication
import library.utils as utils
from PIL import Image
from library.utils import clamp


####  Setup ####
rclpy.init()
team_id, robot_id = utils.get_team_robot_id()
# password = utils.get_password()
# Credentials for Team 3, Robot 0 (Master)
team_id = 3
robot_id = 0
password = "stagnant-attractor-companion"
host = "172.18.0.2:8000"

# --- Константы камеры (откалиброваны экспериментально) ---
IMAGE_W = 640
IMAGE_H = 480
H_CAMERA = 13.0          # высота камеры над полом (см)
TILT_ANGLE = 0.0         # наклон камеры (градусы)
V_FOV = 40.0             # вертикальный угол обзора (градусы)
H_FOV = 62.0             # горизонтальный угол обзора (градусы)

# --- Константы руки ---
ARM_PICKUP_POS = [0.0, -1.8, -1.3, 1.0]   # позиция для захвата
ARM_CHECK_POS = [0.0, -0.4, -1.5, -2.0]   # позиция для проверки захвата
ARM_MOVE_POS = [1.5, 1.2, -1.0, -1.2]     # позиция для транспортировки
ARM_DROP_POS = [0.0, 0.3, -0.8, -1.0]     # позиция для сброса
ARM_PICK_MIDDLE_POS_AFTER_CHECK = [0.0, -0.4, -0.5, 0.0]     # позиция для сброса

# --- Пороги для верификации захвата ---
AREA_THRESHOLD_PX = 0.25 * 640 * 480        # 40% от площади кадра


comm = Communication(host, team_id, robot_id, password)

robot = Robot()
detector = Detector(True)
while detector.latest_image is None:
    print("no image in detector.. waiting")
    time.sleep(5)

objectives = []

###### Callbacks #####
def on_receive_location(x, y, angle, visible, last_seen):
    # This is called whenever the server sends the location of the robot
    # print("location received!")
    robot_position = {"x": x, "y": y, "angle": angle, "visible": visible, "last_seen": last_seen}

def on_receive_start():
    # This is called when the competition is started (and every 10 seconds during the competition)
    # print("start received :)")
    stopped = False

def on_receive_stop():
    # This is called when the competition is paused (and every 10 seconds while it is paused). You MUST stop your robot when this is called!
    # print("stop received :(")
    stopped = True
    robot.drive(0, 0)

def on_receive_objective(from_team_id, from_robot_id, tag_id, x, y, angle, visible, last_seen):
    # This is called when another robot tells you they discovered an objective (id tag_id). It includes the position of the other robot
    print("new objective?")
    new_objective = {
        "tag_id": tag_id,
        "x": y,
        "y": x,
        "found_by_team": team_id,
        "found_by_robot": robot_id,
        "angle": angle,
        "visible": visible,
        "last_seen": last_seen
    }
    if tag_id == 3:
        objectives.append(new_objective)

def on_receive_custom(from_team_id, from_robot_id, internal_type, bytes):
    # This is called whenever another robot sends a custom message
    print("custom received!", internal_type)
    if internal_type == 12:
        try:
            a, b = struct.unpack("<BB", bytes)
            print(a,b)
        except Exception as e:
            print("Could not parse message")

comm.register_callback_location(on_receive_location)
comm.register_callback_start(on_receive_start)
comm.register_callback_stop(on_receive_stop)
comm.register_callback_objective(on_receive_objective)
comm.register_callback_custom(on_receive_custom)

class MainStateMachine:
    def __init__(self, robot, detector):
        self.robot = robot
        self.detector = detector
        self.state = "move_to_obj"
        # self.retry_count = 0
        # self.max_retries = 3
        # self.robot.move_arm_to(*ARM_MOVE_POS, 1)

    def step(self):
        if self.state == "move_to_start":
            self.state = move_to_start()
        elif self.state == "pickup":
            self.state = pickup()
        elif self.state == "dropoff":
            self.state = dropoff()
        elif self.state == "move_to_obj":
            self.state = move_to_obj()
        elif self.state == "exit":
            return True
        else:
            return False
    
    def is_ready(self):
        return self.state == "ready"


##### Pick up ##### 
def pickup():
    from test_all import PickupStateMachine, angle_difference, align_to_objective, OdomHelper, compute_dist_angle_from_contour, detect_contours, get_dist_and_angle, clamp

    pickup_fsm = PickupStateMachine(robot, detector)
    start_time = time.time()
    while not pickup_fsm.is_ready():
        if time.time() - start_time > 120:
            return "move_to_start"
        pickup_fsm.step()
        time.sleep(0.1) 
    print("Pickup done!")
    robot.move_arm_to(*ARM_MOVE_POS, 1)
    
    # return "dropoff" # debug
    return "move_to_obj"


##### Move to location ##### 
def move_to_obj():
    from navigate_master import GlobalNavigator
    from path_planner import AStarPlanner
    
    nav = GlobalNavigator(robot, comm)
    try:
        if len(objectives) > 0: 
            objective = objectives.pop()
            # TARGET coordinates (Mirrored Space)
            target_x = objective["x"]
            target_y = objective["y"]
            
        
            nav.navigate_to(target_x, target_y, timeout=180) # this eventually returns
            print("movement done!")
            return "dropoff"
        
    except KeyboardInterrupt:
        print("\nInterrupted.")


#### Drop off sequence ####
def dropoff():
    from test_dropoff import get_dist_and_angle, DropOffStateMachine
    dropoff_fsm = DropOffStateMachine(robot, detector)
    start_time = time.time()
    
    while not dropoff_fsm.is_ready():
        if time.time() - start_time > 60:
            return "move_to_start"
        dropoff_fsm.step()
        time.sleep(0.1)
    print("Dropoff done!")
    robot.move_arm_to(*ARM_MOVE_POS, 1)
    return "move_to_start"

#### Go back to pick up ####
def move_to_start():
    from navigate_master import GlobalNavigator
    from path_planner import AStarPlanner
    
    print(f"av: {type(robot)}, comm: {type(comm)}")
    nav = GlobalNavigator(robot, comm)
    try:

        target_x = 3.0
        target_y = 0.2
            
        print(f"\n--- Moving Home to ({target_x}, {target_y}) ---")
        
        nav.navigate_to(target_x, target_y, timeout=180) # this eventually returns
        print("movement done!")
        return "pickup"
        
    except KeyboardInterrupt:
        print("\nInterrupted.")

fsm = MainStateMachine(robot, detector)
while not fsm.is_ready():
    fsm.step()
    time.sleep(0.1)   

##### Shutdown nicely #####
robot.drive(0.0, 0.0, 0.0)
time.sleep(0.5)
rclpy.shutdown()

