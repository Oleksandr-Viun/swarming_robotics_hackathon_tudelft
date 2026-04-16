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


###### Setup ######
rclpy.init()

team_id, robot_id = utils.get_team_robot_id()
password = utils.get_password()

robot = Robot()

# communication = Communication("172.18.0.2:8000", team_id, robot_id, password)
detector = Detector(True)

stopped = False
robot_position = None
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
        "x": x,
        "y": y,
        "found_by_team": team_id,
        "found_by_robot": robot_id,
        "angle": angle,
        "visible": visible,
        "last_seen": last_seen
    }
    objectives.append(new_objective)
1
def on_receive_custom(from_team_id, from_robot_id, internal_type, bytes):
    # This is called whenever another robot sends a custom message
    print("custom received!", internal_type)
    if internal_type == 12:
        try:
            a, b = struct.unpack("<BB", bytes)
            print(a,b)
        except Exception as e:
            print("Could not parse message")

# communication.register_callback_location(on_receive_location)
# communication.register_callback_start(on_receive_start)
# communication.register_callback_stop(on_receive_stop)
# communication.register_callback_objective(on_receive_objective)
# communication.register_callback_custom(on_receive_custom)


##### Main loop #####
# Send a custom message to itself (internal type 99, contents is two 8-bit integers (BB): 123 and 45)
# You should see the robot receive this message as well
# communication.send_custom_msg(team_id, robot_id, 99, struct.pack("<BB", 123, 45))

# # Drive forward for 2 seconds
driving_time = 4.0
speed = 0.3
t_end = time.time() + driving_time


while time.time() < t_end:
    # Drive by specifying the speed of the left and right motor
    robot.drive(speed, speed, 0.0)
    print("loop")
    # time.sleep(0.1)
    # Print the detected april tags
    tags = detector.detect_objective_tags()
    for tag in tags:
        print(f"Tag {tag.tag_id}, within distance: {utils.is_tag_within_distance(tag)}")

robot.drive(0.0, 0.0, 0.0)
time.sleep(1.0)
robot.open_gripper()
time.sleep(2.0)
# robot.open_gripper()
# time.sleep(1.0)
# robot.close_gripper()

time.sleep(1.0)
robot.move_arm_to(-0.5, 0.0, 0.0, 0.0, 2)
time.sleep(0.3)

##### Shutdown nicely #####
robot.drive(0.0, 0.0, 0.0)
time.sleep(0.5)
rclpy.shutdown()

img = Image.fromarray(detector.latest_image)
# 3. Save the file
img.save("my_image.png")
