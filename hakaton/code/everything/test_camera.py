import time
import rclpy
from library.detector import Detector
import numpy as np
import cv2
import math


H_CAMERA = 13.0  # Высота линзы камеры над полом (в см)
TILT_ANGLE = 0.0  # Угол наклона камеры вниз (в градусах)
IMAGE_H = 480  # Высота кадра в пикселях
IMAGE_W = 640  # Ширина кадра в пикселях
V_FOV = 45.0  # Вертикальный угол обзора камеры (примерно)
H_FOV = 62.0  # Горизонтальный угол обзора


def get_dist_and_angle(img) -> dict[str, float] | None:
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

rclpy.init()
detector = Detector(True)
print("starting...")
time.sleep(5)
for i in range(1):
    print("sleeping...")
    time.sleep(3.0)
    print(f"Go on measurment {i}")
    res = get_dist_and_angle(detector.latest_image)
    print(f"measure #{i}: {res}")

    with open(f'image_green_{i}.npy', 'wb') as f:
        np.save(f, detector.latest_image)
rclpy.shutdown()