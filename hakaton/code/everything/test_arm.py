import time
import cv2
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

ARM_PICKUP_POS = [0.0, -1.6, -1.2, 0.9]
ARM_CHECK_POS = [0.0, -0.4, -1.5, -2.0]
ARM_MOVE_POS = [0.0, 1.2, -1.0, -1.2]
ARM_DROP_POS = [0.0, 0.3, -0.8, -1.0]

AREA_THRESHOLD_PX = 0.4 * 640 * 480 # use to check if green blob is large enough in image

###### Setup ######
rclpy.init()

team_id, robot_id = utils.get_team_robot_id()
password = utils.get_password()

robot = Robot()

# communication = Communication("172.18.0.2:8000", team_id, robot_id, password)
detector = Detector(True)
while detector.latest_image is None:
    print("no image in detector.. waiting")
    time.sleep(5)
    # detector = Detector(True)

stopped = False
robot_position = None
objectives = []

def pickup_item(position, duration=1):

    robot.open_gripper()
    time.sleep(3.0)

    robot.move_arm_to(0.0, -1.6, 0.0, 0.9, 1)
    time.sleep(3.0)

    robot.move_arm_to(*ARM_PICKUP_POS, 1)
    time.sleep(3.0)

    robot.close_gripper(15.0)
    time.sleep(3.0)

def check_item_in_gripper():
    """call this to check item with camera. if false, run pickup again."""
    robot.move_arm_to(*ARM_CHECK_POS, 1)
    time.sleep(3)
    # check green in center of image
    success = _check_green_in_image()
    return True if success else False

def _check_green_in_image():
    """check if largest green blob in image is larger than a threshold"""
    img = detector.latest_image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([37, 41, 19])
    upper = np.array([90, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return False

    c = max(contours, key=cv2.contourArea)

    _, _, w, h = cv2.boundingRect(c)
    print(f"{w*h/480/640*100:.1f}% green. w,h: {w},{h}")
    if w*h > AREA_THRESHOLD_PX:
        return True
    else:
        return False


def drop():
    robot.move_arm_to(0.0, 0.3, -0.8, -1.0, 1)
    time.sleep(3.0)
    robot.open_gripper(10.0)
    time.sleep(0.5)

# 1. First try to pick up
# move_to_item_on_ground()
pickup_item(ARM_DROP_POS, 1)

# 2. If fail to pick up, keep trying
while not check_item_in_gripper():
    print("nothing in gripper!")
    # move_to_item_on_ground()
    pickup_item(ARM_DROP_POS, 1)
    time.sleep(0.5)
print("pick up done")

# 3. drive to location
# DRIVE
robot.move_arm_to(*ARM_MOVE_POS, 1)
time.sleep(3)

# driving_time = 1.0
# speed = 0.3
# t_end = time.time() + driving_time
# while time.time() < t_end:
#     robot.drive(0.0, 0.0, -1.0)
# robot.drive(0.0, 0.0, 0.0)

# 4. locate obj near location

# 5. Drop in obj
# drop()


##### Shutdown nicely #####
robot.drive(0.0, 0.0, 0.0)
time.sleep(0.5)
rclpy.shutdown()

