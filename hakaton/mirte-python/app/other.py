import cv2
import numpy as np
import json
import math
import glob

# Константы (подставьте ваши)
IMAGE_W, IMAGE_H = 640, 480
H_CAMERA = 13.0
TILT_ANGLE = 0.0
V_FOV = 40.0
H_FOV = 62.0


# Функция детекции (без фильтров, просто возвращает контуры)
def detect_contours(img):
    h, w = img.shape[:2]
    # ROI (по желанию)
    roi_top = int(h * 0.15)
    roi_bottom = int(h * 0.95)
    roi_left = int(w * 0.10)
    roi_right = int(w * 0.90)
    roi = img[roi_top:roi_bottom, roi_left:roi_right]
    if roi.size == 0:
        return []

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower = np.array([40, 70, 70])
    upper = np.array([85, 255, 220])
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Возвращаем контуры в координатах исходного изображения
    result = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 100:  # очень мелкие шумы не сохраняем
            continue
        x, y, wc, hc = cv2.boundingRect(cnt)
        x_abs = x + roi_left
        y_abs = y + roi_top
        result.append(
            {
                "area": area,
                "x": x_abs,
                "y": y_abs,
                "width": wc,
                "height": hc,
                "solidity": area / (wc * hc),
            }
        )
    return result


with open("../some.json", "r") as f:
    NOISE_MODEL = json.load(f)


def is_noise(detection, tolerance=0.0):
    """Проверяет, похожа ли детекция на шум из модели"""
    for noise in NOISE_MODEL:
        # Сравниваем площадь, высоту, ширину с допуском
        area_ratio = detection["area"] / noise["area"]
        if 1 - tolerance <= area_ratio <= 1 + tolerance:
            # Также можно сравнить позицию (x,y) если нужно
            return True
    return False


def get_dist_and_angle_calibrated(img):
    # Получаем все возможные детекции (без внутренних фильтров)
    detections = detect_contours(img)
    if not detections:
        return None

    # Выбираем лучшую (например, по площади) и проверяем на шум
    best = max(detections, key=lambda d: d["area"])
    if is_noise(best):
        print("noise got triggered")
        return None

    # Далее вычисляем расстояние и угол по координатам best
    x_center = best["x"] + best["width"] / 2
    y_bottom = best["y"] + best["height"]

    offset_x = x_center - (IMAGE_W / 2)
    angle_to_target = (offset_x / IMAGE_W) * H_FOV

    relative_pos = y_bottom / IMAGE_H
    ray_angle = (0.5 - relative_pos) * V_FOV
    total_angle = TILT_ANGLE - ray_angle

    if total_angle > 0:
        distance = H_CAMERA / math.tan(math.radians(total_angle)) / 100.0
    else:
        return None

    return {"dist": distance, "angle": angle_to_target}


# file_list = glob.glob("../floor/image_36cm_*.npy")
file_list = glob.glob("../green.npy")
# file_list = glob.glob("../im*.npy")

for file_path in file_list:
    try:
        img = np.load(file_path)

        # Check if loaded correctly
        if img is None:
            print(f"Skipping {file_path}: File is empty or invalid.")
            continue

        # Run your specific function
        result = get_dist_and_angle_calibrated(img=img)
        print(f"File: {file_path} | Result: {result}")

    except Exception as e:
        print(f"Error processing {file_path}: {e}")
