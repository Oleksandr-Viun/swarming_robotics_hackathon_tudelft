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

ARM_PICKUP_POS = [0.0, -1.6, -1.2, 0.9]
ARM_CHECK_POS = [0.0, -0.4, -1.5, -2.0]
ARM_MOVE_POS = [0.0,0.0,0.0,0.0] #[0.0, 1.2, -1.0, -1.2]
ARM_DROP_POS = [0.0, -0.7, -0.8, -1.0]

AREA_THRESHOLD_PX = 0.3*640*480

IMAGE_W = 640
IMAGE_H = 480
H_CAMERA = 13.0          # высота камеры над полом (см)
TILT_ANGLE = 0.0         # наклон камеры (градусы)
V_FOV = 40.0             # вертикальный угол обзора (градусы)
H_FOV = 62.0             # горизонтальный угол обзора (градусы)


def get_dist_and_angle(img):
    """Вычисляет расстояние (м) и угол (град) до зелёного предмета."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([37, 41, 19])
    upper = np.array([90, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)

    # --- ОБРЕЗКА КРАЕВ (ROI) ---
    # Создаем черную рамку (убираем 15% сверху и по 10% по бокам и снизу)
    margin_sides = int(IMAGE_W * 0.10)
    margin_bottom = int(IMAGE_H * 0.05)

    # Зануляем края маски
    mask[IMAGE_H - margin_bottom : IMAGE_H, :] = 0  # Низ
    mask[:, 0:margin_sides] = 0  # Лево
    mask[:, IMAGE_W - margin_sides : IMAGE_W] = 0  # Право
    # ---------------------------

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

class DropOffStateMachine:
    def __init__(self, robot, detector):
        self.robot = robot
        self.detector = detector
        self.state = "rotate"
        self.retry_count = 0
        self.max_retries = 3
        self.robot.move_arm_to(*ARM_MOVE_POS, 1)

    def step(self):
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
        elif self.state == "drop":
            return self._drop()
        elif self.state == "ready":
            return True
        elif self.state == "pos_adjust_not_final":
            return self._pos_adjust_not_final()
        elif self.state == "exit":
            return True
        else:
            return False

    def _rotate(self):
        self.robot.node.get_logger().info("State: Rotate (until obj)")
        angle_to_target = None
        start_time = time.time()
        while angle_to_target is None or angle_to_target < -10 or angle_to_target > 10:
            self.robot.drive(0.0, 0.0, -0.7)
            time.sleep(1)
            img = self.detector.latest_image
            meas = get_dist_and_angle(img)
            if meas is not None:
                angle_to_target = meas["angle"]
            # self.robot.node.get_logger().info(f"Measured: angle={angle_to_target:.1f}deg")
            if time.time() - start_time > 10:
                self.robot.node.get_logger().warn(f"rotated and no target found")
                self.state = "abort"
                return False
                
        if angle_to_target > -5 or angle_to_target < 5:
            self.state = "detect"
            return True

    def _detect(self):
        """Делает измерение расстояния и угла."""
        self.robot.node.get_logger().info("State: detect")
        img = self.detector.latest_image
        # TODO: logic to check id here
        # success = check_if_our_objective()
        tags = self.detector.detect_objective_tags()
        if len(tags) > 0:
            tag_id = tags[0].tag_id
            tag_id = utils.get_team_from_tag_id(tag_id)
            if tag_id == 3:
                success = True
        if not success:
            self.robot.node.get_logger().warn("Did not detect our objective, retry rotate")
            self.state = "rotate"
            return False
        # there is an objective. move towards it
        self.state = "pos_adjust"
        return True

    def _pos_adjust(self):
        self.robot.node.get_logger().info("State: pos_adjust (drive towards obj)")
        img = self.detector.latest_image
        meas = get_dist_and_angle(img)
        angle = meas["angle"]
        self.robot.node.get_logger().info(f"Measured: angle={angle:.1f}deg")
        self.robot.drive(0.0, 0.0, 0.5 * np.sign(angle)) # rotate in the correct direction
        retry_num = 0
        
        while not(angle > -5 or angle < 5):
            retry_num += 1
            img = self.detector.latest_image
            meas = get_dist_and_angle(img)
            angle = meas["angle"]
            # TODO: angle needs to be relative?
            self.robot.node.get_logger().info(f"Adjust: angle={angle:.1f}deg")
            self.robot.drive(0.0, 0.0, 0.3 * np.sign(angle)) # rotate in the correct direction
            time.sleep(0.5)
            if retry_num > 10:
                self.robot.node.get_logger().warn(f"rotation alignment failed")
                self.state = "rotate"
                return False
        self.robot.node.get_logger().info(f"moving closer")
        
        # move forward until close enough
        size = self._get_green_in_image()
        DESIRED_THRESH = 22
        while not (DESIRED_THRESH-2 < size < DESIRED_THRESH+2):
            size = self._get_green_in_image()
            self.robot.node.get_logger().info(f"size is: {size}%")
            sign = np.sign(size - DESIRED_THRESH) # positive if too big: move backwards
            self.robot.drive(-sign*0.2, 0.0, 0.0)
            time.sleep(1)
            # self.robot.node.get_logger().warn(f"green not in image..")
        self.robot.drive(0.0, 0.0, 0.0)
        
        self.robot.node.get_logger().info(f"size is good {size}%")
        self.state = "drop"
        return True

    def _get_green_in_image(self):
        """get size of largest green blob in image"""
        img = self.detector.latest_image
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([37, 41, 19])
        upper = np.array([90, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return False

        c = max(contours, key=cv2.contourArea)

        _, _, w, h = cv2.boundingRect(c)
        size_percentage = w*h/480/640*100
        print(f"{size_percentage:.1f}% green. w,h: {w},{h}")
        return size_percentage
        # if w*h > AREA_THRESHOLD_PX:
        #     return True
        # else:
        #     return False

    def _drop(self):
        self.robot.node.get_logger().info("State: DROPPING")
        self.robot.move_arm_to(*ARM_DROP_POS, 1)
        time.sleep(5)
        self.robot.open_gripper()
        # Transition state
        self.state = "exit"


    def is_ready(self):
        return self.state == "ready"

    


if __name__ == "__main__":
    rclpy.init()

    team_id, robot_id = utils.get_team_robot_id()
    password = utils.get_password()

    robot = Robot()
    detector = Detector(True)
    while detector.latest_image is None:
        print("no image in detector.. waiting")
        time.sleep(5)

    robot.close_gripper()

    fsm = DropOffStateMachine(robot, detector)
    while not fsm.is_ready():
        fsm.step()
        time.sleep(0.1)   
    print("Dropoff done!")
    # robot.move_arm_to(*ARM_MOVE_POS, 1)

    ##### Shutdown nicely #####
    robot.drive(0.0, 0.0, 0.0)
    time.sleep(0.5)
    rclpy.shutdown()