import cv2
import numpy as np
import math

# --- ПАРАМЕТРЫ КАЛИБРОВКИ (замерить 1 раз) ---
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
    y_bottom = y + h
    x_center = x + w / 2
    print(i, y_bottom)

    # --- РАСЧЕТ УГЛА (Yaw) ---
    # Насколько объект смещен от центральной оси (в градусах)
    offset_x = x_center - (IMAGE_W / 2)
    angle_to_target = (offset_x / IMAGE_W) * H_FOV

    # --- РАСЧЕТ РАССТОЯНИЯ (Distance) ---
    # Вычисляем угол луча от горизонтали до нижней точки объекта
    # 0.5 - это центр кадра
    relative_y = (y_bottom / IMAGE_H) - 0.5
    # Угол луча относительно оптической оси камеры
    ray_angle = relative_y * V_FOV
    # Полный угол наклона к полу
    total_angle = TILT_ANGLE + ray_angle
    if total_angle > 0:
        distance = H_CAMERA / math.tan(math.radians(total_angle))
    else:
        distance = 999
    return {
        "dist": round(distance, 1),
        "angle": round(angle_to_target, 1),
    }


for i in range(1):
    img = np.load(f"../im{i}.npy")
    print(get_dist_and_angle(img))
