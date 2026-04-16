import cv2
import numpy as np

"""Final thresholds: [37, 41, 19], [90, 255, 255] """


def nothing(x):
    pass


# 1. Загрузка данных (замени на свой путь)
# Если используешь npy: img = np.load('frame.npy')
img = cv2.imread("../green_last.png")
# img = np.load("../floor/image_36cm_1774698404.7857857.npy")
if img is None:
    print("Ошибка: файл не найден")
    exit()

cv2.namedWindow("Settings", cv2.WINDOW_NORMAL)

# Создаем ползунки для настройки порогов в реальном времени
cv2.createTrackbar("H_low", "Settings", 37, 179, nothing)
cv2.createTrackbar("H_high", "Settings", 90, 179, nothing)
cv2.createTrackbar("S_low", "Settings", 41, 255, nothing)
cv2.createTrackbar("V_low", "Settings", 40, 255, nothing)
cv2.createTrackbar("Morph", "Settings", 5, 20, nothing)  # Размер ядра чистки шума

while True:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Считываем текущие значения из ползунков
    h_l = cv2.getTrackbarPos("H_low", "Settings")
    h_h = cv2.getTrackbarPos("H_high", "Settings")
    s_l = cv2.getTrackbarPos("S_low", "Settings")
    v_l = cv2.getTrackbarPos("V_low", "Settings")
    m_size = cv2.getTrackbarPos("Morph", "Settings")

    # lower = np.array([37, 41, 40])
    # lower = np.array([37, 41, 19])
    # upper = np.array([90, 255, 255])
    lower = np.array([h_l, s_l, v_l])
    upper = np.array([h_h, 255, 255])

    # Обработка
    mask = cv2.inRange(hsv, lower, upper)

    # Морфология (настройка ядра динамически)
    if m_size > 0:
        kernel = np.ones((m_size, m_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Визуализация детекции для проверки логики
    output = img.copy()
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # for c in contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 500:
            # 1. Вычисляем моменты для центра масс
            M = cv2.moments(c)
            if M["m00"] != 0:
                # Центр масс (Centroid)
                cx_mass = int(M["m10"] / M["m00"])
                cy_mass = int(M["m01"] / M["m00"])
            else:
                cx_mass, cy_mass = 0, 0

            # 2. Получаем геометрию прямоугольника (для угла)
            rect = cv2.minAreaRect(c)
            (cx_rect, cy_rect), (w, h), angle = rect
            box = np.array(cv2.boxPoints(rect), dtype=np.int32)

            # --- ОТРИСОВКА ---
            # Рисуем рамку
            cv2.drawContours(output, [box], 0, (0, 255, 0), 2)

            # Точка центра масс (СИНЯЯ)
            cv2.circle(output, (cx_mass, cy_mass), 7, (255, 0, 0), -1)

            # Точка центра рамки (КРАСНАЯ, для сравнения)
            cv2.circle(output, (int(cx_rect), int(cy_rect)), 4, (0, 0, 255), -1)

            # Текст с координатами центра масс
            cv2.putText(
                output,
                f"Mass Center: {cx_mass}, {cy_mass}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
            )

    # Показываем результат
    cv2.imshow("Original + Detection", output)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        # Выводим финальные значения в консоль, чтобы скопировать в основной код
        print(f"Final thresholds: [{h_l}, {s_l}, {v_l}], [{h_h}, 255, 255]")
        break

cv2.destroyAllWindows()
