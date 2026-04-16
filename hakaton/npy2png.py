import numpy as np
from PIL import Image
from pathlib import Path

source_dir = Path(".")
output_dir = Path("./preview")
output_dir.mkdir(exist_ok=True)

for npy_path in source_dir.rglob("*.npy"):
    try:
        data = np.load(npy_path)
    except ValueError:
        continue

    # Твои данные 480x640x3 uint8
    if data.shape == (480, 640, 3):
        # Считаем контрастность (std), чтобы найти "сочные" кадры
        contrast = np.std(data)

        img = Image.fromarray(data.astype("uint8"))

        # Сохраняем с префиксом контрастности, чтобы в папке они
        # отсортировались от скучных к интересным
        name = f"{int(contrast):03d}_{npy_path.stem}.png"
        img.save(output_dir / name)
