# import rclpy
# from rclpy.node import Node
# import math
# from sensor_msgs.msg import Image
# from enum import Enum
# import struct
# import numpy as np
# import threading
#
# from library.robot_big import Robot
# from library.detector import Detector
# from library.communication import Communication
# import library.utils as utils

# Our deps
import time
import numpy as np
import cv2

# Предположим, используешь mirte-python-sdk или ROS2 интерфейс
from build.lib.mirte_robot.robot import Robot


class MirtePicker:
    def __init__(self):
        self.robot = Robot()
        self.camera = self.robot.get_camera("orbbec_astra")
        self.arm = self.robot.get_arm()
        self.gripper = self.robot.get_gripper()

        # Калибровочные константы
        self.GRAB_HEIGHT = 0.05  # Высота захвата над полом
        self.SAFE_HEIGHT = 0.2  # Высота переноса

    def detect_chainsaw(self):
        """Поиск зеленой бензопилы через HSV и контуры."""
        img = self.camera.get_color_frame()
        if img is None:
            return None

        # 1. Переходим в HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 2. Задаем диапазон зеленого (настрой под освещение в RoboHouse)
        # H: 35-90 (зеленый), S: 50-255 (насыщенный), V: 50-255 (не слишком темный)
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([90, 255, 255])

        # 3. Создаем маску и чистим шум
        mask = cv2.inRange(hsv, lower_green, upper_green)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # убирает мелкие точки

        # 4. Ищем контуры
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Берем самый большой контур (предполагаем, что это бензопила)
            c = max(contours, key=cv2.contourArea)

            # Если объект слишком маленький, игнорируем (защита от шума)
            if cv2.contourArea(c) < 500:
                return None

            # Получаем координаты центра (x, y)
            M = cv2.moments(c)
            if M["m00"] != 0:
                target_x = int(M["m10"] / M["m00"])
                target_y = int(M["m01"] / M["m00"])
            else:
                return None

            # Получаем угол поворота (minAreaRect дает (центр, размер, угол))
            rect = cv2.minAreaRect(c)
            target_angle = rect[2]

            # Важно: OpenCV возвращает угол от -90 до 0.
            # Возможно, понадобится коррекция в зависимости от длинной стороны:
            (w, h) = rect[1]
            if w < h:
                target_angle += 90

            return target_x, target_y, target_angle

        return None

    def align_to_target(self, x, y, angle):
        """Шаги 2 & 3: Rotate & Pos Adjust."""
        print(f"Aligning to: {x}, {y} with angle {angle}")
        # Поворот основания или кисти под бензопилу
        self.arm.set_joint_position("joint_waist", np.arctan2(y, x))
        # Подъезд к точке, если манипулятор не дотягивается
        # self.robot.drive_to(x - offset, y)

    def grab_sequence(self):
        """Шаг 4: Grab."""
        self.gripper.open()
        current_pos = self.arm.get_current_pose()
        # Опускаемся
        self.arm.move_to(current_pos.x, current_pos.y, self.GRAB_HEIGHT)
        self.gripper.close()
        time.sleep(1)
        # Поднимаем
        self.arm.move_to(current_pos.x, current_pos.y, self.SAFE_HEIGHT)

    def verify_grasp(self):
        """Шаг 5: Verify. Проверка по датчикам тока или камере."""
        return (
            self.gripper.get_effort() > 0.5
        )  # Пример: если сопротивление есть, значит схватили

    def run_dag(self):
        """Основной цикл исполнения."""
        print("Starting DAG...")

        # 1. Detect
        target = self.detect_chainsaw()
        if not target:
            print("Chainsaw not found")
            return

        x, y, angle = target

        # 2. & 3. Rotate & Adjust
        self.align_to_target(x, y, angle)

        # 4. Grab
        self.grab_sequence()

        # 5. Verify
        if self.verify_grasp():
            print("Success! Moving to 'Ready' state.")
            # 6. Ready
            self.arm.go_to_named_pose("home")
        else:
            print("Failed to grab. Resetting.")
            self.gripper.open()


if __name__ == "__main__":
    picker = MirtePicker()
    while True:
        picker.run_dag()
