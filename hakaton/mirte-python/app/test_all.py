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

# --- Константы камеры (откалиброваны экспериментально) ---
IMAGE_W = 640
IMAGE_H = 480
H_CAMERA = 13.0  # высота камеры над полом (см)
TILT_ANGLE = 0.0  # наклон камеры (градусы)
V_FOV = 40.0  # вертикальный угол обзора (градусы)
H_FOV = 62.0  # горизонтальный угол обзора (градусы)

# --- Константы руки ---
ARM_PICKUP_POS = [0.0, -1.8, -1.3, 1.0]  # позиция для захвата
ARM_CHECK_POS = [0.0, -0.4, -1.5, -2.0]  # позиция для проверки захвата
ARM_MOVE_POS = [0.0, 1.2, -1.0, -1.2]  # позиция для транспортировки
ARM_DROP_POS = [0.0, 0.3, -0.8, -1.0]  # позиция для сброса

# --- Пороги для верификации захвата ---
AREA_THRESHOLD_PX = 0.4 * 640 * 480  # 40% от площади кадра


def get_dist_and_angle(img):
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
    return {"dist": distance, "angle": angle_to_target}


class OdomHelper(Node):
    """Подписчик на одометрию для получения текущего положения."""

    def __init__(self):
        super().__init__("odom_helper")
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0
        self.received = False
        self.create_subscription(Odometry, "/mirte_base_controller/odom", self._cb, 10)

    def _cb(self, msg: Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.heading = math.atan2(siny_cosp, cosy_cosp)
        self.received = True


def align_to_objective(robot, initial_dist, initial_yaw_deg, target_distance=0.2):
    """
    Подъезжает к объекту на target_distance (м), используя начальные измерения.
    """
    helper = OdomHelper()
    robot.node.get_logger().info("Waiting for odometry...")
    while rclpy.ok() and not helper.received:
        rclpy.spin_once(helper, timeout_sec=0.1)

    if initial_dist is None or initial_yaw_deg is None:
        robot.node.get_logger().error("No initial measurement, abort alignment")
        return False

    initial_yaw_rad = math.radians(initial_yaw_deg)
    world_target_heading = helper.heading + initial_yaw_rad
    travel_dist = initial_dist - target_distance
    if travel_dist <= 0:
        robot.node.get_logger().warn("Already closer than target, no movement needed")
        return True

    stop_x = helper.pos_x + (travel_dist * math.cos(world_target_heading))
    stop_y = helper.pos_y + (travel_dist * math.sin(world_target_heading))

    while rclpy.ok():
        rclpy.spin_once(helper, timeout_sec=0)
        dx = stop_x - helper.pos_x
        dy = stop_y - helper.pos_y
        distance_error = math.hypot(dx, dy)
        heading_error = angle_difference(world_target_heading, helper.heading)

        if (
            distance_error < robot.linear_threshold
            and abs(heading_error) < robot.angular_threshold
        ):
            break

        angular_speed = clamp(
            robot.kp_angular * heading_error,
            -robot.max_angular_speed,
            robot.max_angular_speed,
        )
        linear_speed = robot.clamp_linear_speed(
            robot.kp_linear * distance_error, 0.0, robot.max_linear_speed
        )

        if abs(heading_error) > robot.stop_driving_forward_angle:
            linear_speed = 0.0
        elif abs(heading_error) > robot.slower_driving_forward_angle:
            linear_speed *= 0.2

        robot.drive(linear_x=linear_speed, linear_y=0.0, angular_z=angular_speed)
        time.sleep(robot.poll_freq)

    robot.drive(0.0, 0.0, 0.0)
    robot.node.get_logger().info("Alignment complete")
    return True


def angle_difference(a, b):
    """Разница углов в радианах."""
    diff = a - b
    return math.atan2(math.sin(diff), math.cos(diff))


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


class PickupStateMachine:
    """Конечный автомат для автономного захвата предмета."""

    def __init__(self, robot, detector):
        self.robot = robot
        self.detector = detector
        self.state = "rotate"
        self.retry_count = 0
        self.max_retries = 3
        self.robot.move_arm_to(*ARM_MOVE_POS, 1)

    def step(self):
        """Выполняет один шаг автомата. Возвращает True, если захват успешен."""
        if self.state == "rotate":
            return self._rotate()
        elif self.state == "detect":
            return self._detect()
        elif self.state == "pos_adjust":
            return self._pos_adjust()
        elif self.state == "grab":
            return self._grab()
        elif self.state == "verify":
            return self._verify()
        elif self.state == "ready":
            return True
        else:
            return False

    def _rotate(self):
        """Вращается, пока не найдёт зелёный объект."""
        self.robot.node.get_logger().info("State: rotate")
        # Вращаемся с небольшой угловой скоростью
        self.robot.drive(0.0, 0.0, -0.5)  # скорость поворота
        time.sleep(0.2)  # даём время на получение нового изображения
        img = self.detector.latest_image
        if img is not None and get_dist_and_angle(img) is not None:
            self.robot.drive(0.0, 0.0, 0.0)  # остановка
            self.state = "detect"
        else:
            # продолжаем вращение
            pass
        return False

    def _detect(self):
        """Делает измерение расстояния и угла."""
        self.robot.node.get_logger().info("State: detect")
        img = self.detector.latest_image
        if img is None:
            self.state = "rotate"
            return False
        meas = get_dist_and_angle(img)
        if meas is None:
            self.robot.node.get_logger().warn("No object detected, retry rotate")
            self.state = "rotate"
            return False
        self.dist = meas["dist"]
        self.angle = meas["angle"]
        self.robot.node.get_logger().info(
            f"Measured: dist={self.dist:.2f}m, angle={self.angle:.1f}deg"
        )
        self.state = "pos_adjust"
        print(self.dist, self.angle, self.state)
        return False

    def _pos_adjust(self):
        """Подъезжает на 25 см от объекта."""
        self.robot.node.get_logger().info("State: pos_adjust (drive to 0.25m)")
        success = align_to_objective(
            self.robot, self.dist, -self.angle, target_distance=0.25
        )
        if success:
            self.state = "grab"
        else:
            self.state = "rotate"
        return False

    def _grab(self):
        """Опускает руку, подъезжает ещё на 5 см и захватывает."""
        self.robot.node.get_logger().info("State: grab")
        # 1. Опускаем руку в позицию захвата
        self.robot.open_gripper()
        self.robot.move_arm_to(*ARM_PICKUP_POS, 2)
        time.sleep(3.0)
        # 2. Подъезжаем вперёд на 5 см (используем одометрию)
        helper = OdomHelper()
        while rclpy.ok() and not helper.received:
            rclpy.spin_once(helper, timeout_sec=0.1)
        start_x, start_y = helper.pos_x, helper.pos_y
        # Целевое расстояние 0.05 м в направлении текущей ориентации
        target_dx = 0.05 * math.cos(helper.heading)
        target_dy = 0.05 * math.sin(helper.heading)
        target_x = start_x + target_dx
        target_y = start_y + target_dy
        # Двигаемся пока не приблизимся к целевой точке
        while rclpy.ok():
            rclpy.spin_once(helper, timeout_sec=0)
            dx = target_x - helper.pos_x
            dy = target_y - helper.pos_y
            dist_error = math.hypot(dx, dy)
            if dist_error < 0.02:
                break
            # Простая П-регуляция линейной скорости
            speed = self.robot.clamp_linear_speed(0.3 * dist_error, 0.0, 0.1)
            self.robot.drive(linear_x=speed, linear_y=0.0, angular_z=0.0)
            time.sleep(0.1)
        self.robot.drive(0.0, 0.0, 0.0)
        # 3. Захват
        self.robot.close_gripper(20.0)
        time.sleep(3.0)
        # 4. Поднимаем руку в транспортное положение
        self.robot.move_arm_to(*ARM_MOVE_POS, 1)
        time.sleep(3.0)
        self.state = "verify"
        return False

    def _verify(self):
        """Проверяет, что предмет действительно в клешне."""
        self.robot.node.get_logger().info("State: verify")
        # Перемещаем руку в положение проверки
        self.robot.move_arm_to(*ARM_CHECK_POS, 1)
        time.sleep(3.0)
        # Проверяем зелёный цвет в центре кадра
        img = self.detector.latest_image
        if img is None:
            self.state = "rotate"
            return False
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([37, 41, 19])
        upper = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.robot.node.get_logger().warn("No green in gripper")
            self.retry_count += 1
            if self.retry_count < self.max_retries:
                # Сбросить руку и попробовать снова
                self.robot.move_arm_to(*ARM_MOVE_POS, 1)
                time.sleep(2.0)
                self.state = "detect"  # повторить захват
            else:
                self.state = "rotate"  # слишком много попыток, начать сначала
            return False
        # Зелёный есть -> успех
        self.robot.node.get_logger().info("Gripper verified: item present")
        self.state = "ready"
        return True

    def is_ready(self):
        return self.state == "ready"


# Пример использования:
rclpy.init()

team_id, robot_id = utils.get_team_robot_id()
password = utils.get_password()

robot = Robot()
detector = Detector(True)

fsm = PickupStateMachine(robot, detector)
while not fsm.is_ready():
    fsm.step()
    time.sleep(0.1)  # небольшая пауза между шагами
print("Pickup done!")

##### Shutdown nicely #####
robot.drive(0.0, 0.0, 0.0)
time.sleep(0.5)
rclpy.shutdown()
