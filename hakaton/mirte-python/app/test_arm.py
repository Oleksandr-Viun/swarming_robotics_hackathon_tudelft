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

driving_time = 0.0
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

robot.move_arm_to(0.0, 1.2, -1.0, -1.2, 1)
time.sleep(0.3)
robot.close_gripper(10.0)

robot.drive(0.0, 0.0, 0.0)
time.sleep(1.0)
robot.move_arm_to(0.0, 0.3, -0.8, -1.0, 1)
robot.open_gripper(10.0)
time.sleep(2.0)

# robot.open_gripper()
# time.sleep(1.0)
# robot.close_gripper()


##### Shutdown nicely #####
robot.drive(0.0, 0.0, 0.0)
time.sleep(0.5)
rclpy.shutdown()


### Pick up
### Drop
### Locate


def drop():
    robot.move_arm_to(0.0, 0.3, -0.8, -1.0, 1)
    time.sleep(0.5)
    robot.open_gripper(10.0)
    time.sleep(0.5)


# def locate():
