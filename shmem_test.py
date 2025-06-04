import numpy as np
import cv2
import mmap
import os
import time
import sys

SHARED_MEMORY_NAME = "/UE5_SharedMemory"
WIDTH = 1280
HEIGHT = 720
BYTES_PER_FLOAT = 4
SHARED_MEMORY_SIZE = WIDTH * HEIGHT * BYTES_PER_FLOAT

# Получаем полный путь к разделяемой памяти в Unix
SHM_PATH = f"/tmp{SHARED_MEMORY_NAME}"

# Проверяем наличие shm-файла
if not os.path.exists(SHM_PATH):
    print(f"[ERROR] Shared memory file not found: {SHM_PATH}")
    sys.exit(1)

# Открываем файл разделяемой памяти
with open(SHM_PATH, "rb") as f:
    # Отображаем его в память
    shm = mmap.mmap(f.fileno(), SHARED_MEMORY_SIZE, mmap.MAP_SHARED, mmap.PROT_READ)

    print("[INFO] Connected to shared memory.")
    try:
        while True:
            shm.seek(0)
            data = shm.read(SHARED_MEMORY_SIZE)

            # Преобразуем в float32-массив
            depth_array = np.frombuffer(data, dtype=np.float32).reshape((HEIGHT, WIDTH))

            # Нормализуем глубину для визуализации
            if depth_array.max() > 0:
                depth_vis = (depth_array / depth_array.max() * 255).astype(np.uint8)
            else:
                depth_vis = np.zeros_like(depth_array, dtype=np.uint8)

            cv2.imshow("Depth Image", depth_vis)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.01)  # Чтобы не грузить CPU

    finally:
        os.remove(SHM_PATH)
        # shm.close()
        print("[INFO] Shared memory unmapped.")
