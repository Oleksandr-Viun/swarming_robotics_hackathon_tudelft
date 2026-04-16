import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node 
import math

from library.robot_big import Robot
from library.detector import Detector
from library.communication import Communication
import library.utils as utils
import json

# Константы (подставьте ваши)
IMAGE_W, IMAGE_H = 640, 480
H_CAMERA = 13.0
TILT_ANGLE = 0.0
V_FOV = 40.0
H_FOV = 62.0
ARM_MOVE_POS = [2.0, 1.2, -1.0, -1.2]     # позиция для транспортировки

def compute_dist_angle_from_contour(cnt, img_shape):
    """
    Возвращает (distance_см, angle_град) для контура или None, если расчёт невозможен.
    """
    x, y, w, h = cv2.boundingRect(cnt)
    y_bottom = y + h
    x_center = x + w / 2

    offset_x = x_center - (img_shape[1] / 2)
    angle_to_target = (offset_x / img_shape[1]) * H_FOV

    relative_pos = y_bottom / img_shape[0]
    ray_angle = (0.5 - relative_pos) * V_FOV
    total_angle = TILT_ANGLE - ray_angle

    if total_angle > 0:
        distance_cm = H_CAMERA / math.tan(math.radians(total_angle))
        return distance_cm, angle_to_target
    else:
        return None

# Функция детекции (без фильтров, просто возвращает контуры)
def detect_contours(img):    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([37, 41, 19])
    upper = np.array([85, 255, 220])
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((9,9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Возвращаем контуры в координатах исходного изображения
    result = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 50:  # очень мелкие шумы не сохраняем
            continue
        x, y, wc, hc = cv2.boundingRect(cnt)
        x_abs = x
        y_abs = y
        result.append({
            'area': area,
            'x': x_abs,
            'y': y_abs,
            'width': wc,
            'height': hc,
            'solidity': area / (wc * hc)
        })
    return result

def calibrate(robot, detector, num_steps=100):
    """
    robot – объект с методами drive(lin_vel, ang_vel) и get_image()
    """
    noise_samples = []   # каждый элемент: (dist_cm, angle_deg)

    for step in range(num_steps):
        # Поворачиваем робота с угловой скоростью 0.5 рад/с (пример)
        robot.drive(0.0, 0.0, -0.5)
        time.sleep(0.3)   # ждём стабилизации кадра

        img = detector.latest_image
        # Детекция без фильтра шума
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([37, 41, 19])
        upper = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            # Можно отфильтровать слишком мелкие контуры (опционально)
            if cv2.contourArea(cnt) < 100:
                continue
            res = compute_dist_angle_from_contour(cnt, img.shape)
            if res is not None:
                dist_cm, angle_deg = res
                noise_samples.append((dist_cm, angle_deg))

    # Убираем явные дубликаты (для экономии места)
    noise_samples = list(set(noise_samples))   # если пары чисел могут повторяться

    with open('noise_model.json', 'w') as f:
        json.dump(noise_samples, f)
    print(f"Сохранено {len(noise_samples)} шумовых образцов")

# Пример использования:
rclpy.init()

team_id, robot_id = utils.get_team_robot_id()
password = utils.get_password()

robot = Robot()
detector = Detector(True)
robot.move_arm_to(*ARM_MOVE_POS, 1)
time.sleep(2.5)

calibrate(robot, detector)

##### Shutdown nicely #####
robot.drive(0.0, 0.0, 0.0)
time.sleep(0.5)
rclpy.shutdown()
