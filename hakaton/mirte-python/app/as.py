import time
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from enum import Enum
import struct
import threading
from nav_msgs.msg import Odometry

from library.robot_big import Robot
from library.detector import Detector
from library.communication import Communication
import library.utils as utils
from library.utils import clamp


def get_dist_and_angle_original(img):
    """Вычисляет расстояние (м) и угол (град) до зелёного предмета."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([37, 41, 19])
    upper = np.array([90, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    y_bottom = y + h
    x_center = x + w / 2
    offset_x = x_center - (IMAGE_W / 2)
    angle_to_target = (offset_x / IMAGE_W) * H_FOV
    relative_pos = y_bottom / IMAGE_H
    ray_angle = (0.5 - relative_pos) * V_FOV
    total_angle = TILT_ANGLE - ray_angle
    if total_angle > 0:
        distance = H_CAMERA / math.tan(math.radians(total_angle)) / 100.0  # в метры
    else:
        return None
    if not (-20 < angle_to_target < 20) or distance > 1.5:
        print(f"More than 10 degrees, {distance=}, {angle_to_target=}")
        return None
    return {"dist": distance, "angle": angle_to_target}


GET_DIST_FUNC = get_dist_and_angle_original


def get_dist_and_angle_first(img) -> dict[str, float] | None:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([37, 41, 19])
    upper = np.array([90, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    c = max(contours, key=cv2.contourArea)

    x, y, w, h = cv2.boundingRect(c)
    print(f"pixels w:{w} h:{h}")
    y_bottom = y + h
    x_center = x + w / 2

    offset_x = x_center - (IMAGE_W / 2)
    angle_to_target = (offset_x / IMAGE_W) * H_FOV

    relative_pos = y_bottom / IMAGE_H

    ray_angle = (0.5 - relative_pos) * V_FOV

    total_angle = TILT_ANGLE - ray_angle

    if total_angle > 0:
        distance = H_CAMERA / math.tan(math.radians(total_angle))
    else:
        distance = 999

    return {
        "dist": round(distance, 1),
        "angle": round(angle_to_target, 1),
    }
