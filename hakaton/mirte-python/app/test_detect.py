import glob
import cv2
import math
import numpy as np

# --- Константы камеры (откалиброваны экспериментально) ---
IMAGE_W = 640
IMAGE_H = 480
H_CAMERA = 13.0  # высота камеры над полом (см)
TILT_ANGLE = 0.0  # наклон камеры (градусы)
V_FOV = 40.0  # вертикальный угол обзора (градусы)
H_FOV = 62.0  # горизонтальный угол обзора (градусы)


# def get_dist_and_angle(img):
#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#     lower = np.array([48, 47, 44])  # Поднял S и V, чтобы отсечь тусклые блики на полу
#     upper = np.array([58, 255, 255])
#     mask = cv2.inRange(hsv, lower, upper)
#     # [48, 47, 44], [58, 255, 255]
#     # 1. ЖЕСТКАЯ МОРФОЛОГИЯ: Склеиваем объект и убираем точки
#     kernel = np.ones((7, 7), np.uint8)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#
#     candidates = []
#     for c in contours:
#         area = cv2.contourArea(c)
#         if area < 400:
#             continue  # Слишком мелко
#
#         x, y, w, h = cv2.boundingRect(c)
#
#         # 2. ГЕОМЕТРИЧЕСКИЙ ФИЛЬТР (Solidity)
#         # У плотного объекта площадь контура ~70-90% от прямоугольника
#         # У россыпи бликов — < 30%
#         solidity = float(area) / (w * h)
#         if solidity < 0.4:
#             continue
#
#         # 3. ИГНОРИМ КРАЯ (Блики обычно по бокам от линзы)
#         center_dist = abs((x + w / 2) - IMAGE_W / 2)
#         if center_dist > (IMAGE_W * 0.4):  # Игнорим крайние 10% кадра
#             area *= 0.5  # Штрафуем объекты на периферии
#
#         candidates.append((area, c))
#
#     if not candidates:
#         print("no candidates")
#         return None
#
#     # Берем лучший по площади из прошедших фильтры
#     candidates.sort(key=lambda x: x[0], reverse=True)
#     c = candidates[0][1]
#
#     # --- Твоя математика (без изменений) ---
#     x, y, w, h = cv2.boundingRect(c)
#     y_bottom = y + h
#     x_center = x + w / 2
#
#     offset_x = x_center - (IMAGE_W / 2)
#     angle_to_target = (offset_x / IMAGE_W) * H_FOV
#     relative_pos = y_bottom / IMAGE_H
#     ray_angle = (0.5 - relative_pos) * V_FOV
#     total_angle = TILT_ANGLE - ray_angle
#
#     if total_angle > 0:
#         distance = H_CAMERA / math.tan(math.radians(total_angle)) / 100.0
#     else:
#         print("total_angle < 0")
#         return None
#
#     return {"dist": distance, "angle": angle_to_target}


def get_dist_and_angle_old(img):
    """Вычисляет расстояние (м) и угол (град) до зелёного предмета."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([53, 48, 40])  # Поднял S и V, чтобы отсечь тусклые блики на полу
    upper = np.array([57, 255, 255])
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


def get_dist_and_angle_new(img):
    # 1. ОБРЕЗКА ШУМА (ROI)
    # Блики по бокам и в самом низу нам не нужны.
    # Зануляем края маски (например, по 10% слева/справа и 15% снизу)
    h, w = img.shape[:2]

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([37, 60, 50])  # Подняли S и V, чтобы отсечь слабые блики
    upper = np.array([90, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)

    # Убираем края, где на фото мелкодисперсный мусор
    mask[:, : int(w * 0.1)] = 0  # Лево
    mask[:, int(w * 0.9) :] = 0  # Право
    mask[int(h * 0.85) :, :] = 0  # Низ (под самым носом робота)

    # 2. МОРФОЛОГИЯ (Смыкание)
    # Чтобы "склеить" части бензопилы и проигнорировать пыль
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None

    # 3. УМНЫЙ ВЫБОР (Вместо самого большого — самый "плотный")
    best_cnt = None
    max_score = -1

    for c in contours:
        area = cv2.contourArea(c)
        if area < 400:
            continue

        x, y, w_obj, h_obj = cv2.boundingRect(c)
        solidity = area / float(w_obj * h_obj)

        # Мы ищем объект, который достаточно "цельный" (solidity > 0.5)
        # Блики обычно разлапистые или дырявые
        score = area * solidity

        if score > max_score:
            max_score = score
            best_cnt = c

    if best_cnt is None:
        return None

    # --- Твоя математика ---
    x, y, w_obj, h_obj = cv2.boundingRect(best_cnt)
    y_bottom = y + h_obj
    x_center = x + w_obj / 2

    offset_x = x_center - (IMAGE_W / 2)
    angle_to_target = (offset_x / IMAGE_W) * H_FOV

    relative_pos = y_bottom / IMAGE_H
    ray_angle = (0.5 - relative_pos) * V_FOV
    total_angle = TILT_ANGLE - ray_angle

    if total_angle > 0:
        # Добавь abs() на случай если камера смотрит чуть ниже горизонта
        distance = H_CAMERA / math.tan(math.radians(total_angle)) / 100.0
    else:
        return None

    return {"dist": distance, "angle": angle_to_target}


def get_dist_and_angle_original(img):
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


def get_dist_and_angle_robust(img):
    # --- 1. Обрезка области интереса (ROI) ---
    h, w = img.shape[:2]
    # Убираем по 15% сверху (там обычно потолок/стена),
    # по 10% слева/справа, и 5% снизу (самый пол у робота)
    roi_top = int(h * 0.15)
    roi_bottom = int(h * 0.95)  # не трогаем самый низ, чтобы не потерять объект
    roi_left = int(w * 0.10)
    roi_right = int(w * 0.90)

    # Вырезаем ROI для обработки, но координаты потом пересчитаем
    roi = img[roi_top:roi_bottom, roi_left:roi_right]
    if roi.size == 0:
        return None

    # --- 2. Цветовая фильтрация (сужаем диапазон) ---
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # Более узкий диапазон: исключаем слишком тусклые и слишком яркие пиксели (блики)
    lower = np.array([40, 70, 70])  # повысили насыщенность и яркость
    upper = np.array([85, 255, 220])  # убрали слишком яркие (V > 220)
    mask = cv2.inRange(hsv, lower, upper)

    # --- 3. Морфология для связывания частей объекта ---
    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # убираем мелкий шум

    # --- 4. Поиск контуров ---
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # --- 5. Фильтрация по форме и плотности ---
    best_contour = None
    best_score = -1

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:  # минимальная площадь объекта в ROI
            continue

        # Прямоугольная рамка
        x, y, w_obj, h_obj = cv2.boundingRect(cnt)

        # Отношение площади к площади bounding box (solidity)
        # У настоящего объекта solidity > 0.6, у бликов часто < 0.5
        solidity = area / float(w_obj * h_obj)
        if solidity < 0.6:
            continue

        # Отношение сторон (ширина/высота). Блики часто вытянуты горизонтально
        aspect_ratio = w_obj / float(h_obj)
        if aspect_ratio > 3.0 or aspect_ratio < 0.5:
            continue

        # Проверка на "дырки": у замкнутого объекта площадь контура близка к площади заливки
        # Можно использовать cv2.contourArea(cnt) и cv2.contourArea(cnt, oriented=False) — они одинаковы.
        # Дополнительно: если объект имеет внутренние контуры, можно отсечь (но здесь не требуется)

        # Вычисляем "скоро" — комбинация площади и плотности
        score = area * solidity
        if score > best_score:
            best_score = score
            best_contour = cnt

    if best_contour is None:
        return None

    # --- 6. Пересчёт координат в исходное изображение ---
    x, y, w_obj, h_obj = cv2.boundingRect(best_contour)
    # Сдвигаем обратно с учётом ROI
    x_abs = x + roi_left
    y_abs = y + roi_top
    w_abs = w_obj
    h_abs = h_obj

    # --- 7. Расчёт расстояния и угла ---
    y_bottom = y_abs + h_abs
    x_center = x_abs + w_abs / 2

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


# file_list = glob.glob("../im*.npy")
# file_list = glob.glob("../green_last.npy")
file_list = glob.glob("../floor/image_36cm_*.npy")

for file_path in file_list:
    try:
        img = np.load(file_path)

        # Check if loaded correctly
        if img is None:
            print(f"Skipping {file_path}: File is empty or invalid.")
            continue

        # Run your specific function
        result = get_dist_and_angle_old(img=img)
        print(f"File: {file_path} | Result: {result}")

    except Exception as e:
        print(f"Error processing {file_path}: {e}")
