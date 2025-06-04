import numpy as np
import imageio.v2 as imageio
import mmap
import time
import os

WIDTH = 1280
HEIGHT = 720
FLOAT_SIZE = 4
DEPTH_SHM = "/tmp/UE5_SharedMemory_Depth"
RGB_SHM = "/tmp/UE5_SharedMemory_RGB"

DEPTH_DIR = "depth"
RGB_DIR = "rgb"
TIMESTAMP_FILE = os.path.join(DEPTH_DIR, "timestamps.txt")

os.makedirs(DEPTH_DIR, exist_ok=True)
os.makedirs(RGB_DIR, exist_ok=True)

depth_buf_size = WIDTH * HEIGHT * FLOAT_SIZE
rgb_buf_size = WIDTH * HEIGHT * 4  # RGBA8

with open(TIMESTAMP_FILE, "w") as ts_file:
    while True:
        try:
            with open(DEPTH_SHM, "rb") as f_d, open(RGB_SHM, "rb") as f_rgb:
                shm_d = mmap.mmap(f_d.fileno(), depth_buf_size, access=mmap.ACCESS_READ)
                shm_rgb = mmap.mmap(f_rgb.fileno(), rgb_buf_size, access=mmap.ACCESS_READ)

                shm_d.seek(0)
                shm_rgb.seek(0)

                depth_data = shm_d.read(depth_buf_size)
                rgb_data = shm_rgb.read(rgb_buf_size)

                shm_d.close()
                shm_rgb.close()
        except FileNotFoundError:
            print("[WAIT] Shared memory not ready. Waiting...")
            time.sleep(1)
            continue

        if len(depth_data) != depth_buf_size:
            print(f"[ERROR] Invalid depth buffer size: {len(depth_data)}")
            continue

        if len(rgb_data) != rgb_buf_size:
            print(f"[ERROR] Invalid RGB buffer size: {len(rgb_data)}")
            continue

        # Чтение глубины (little-endian float32)
        depth = np.frombuffer(depth_data, dtype='<f4').reshape((HEIGHT, WIDTH))
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth[depth < 0.01] = 0.0

        max_depth = depth.max()
        if max_depth == 0:
            print("[SKIP] All-zero depth")
            time.sleep(0.2)
            continue

        print(f"[DEBUG] Depth sample: {depth.flat[:5]}  Max={max_depth:.2f}")

        normalized = np.clip(depth / max_depth, 0.0, 1.0)
        depth_uint16 = (normalized * 65535).astype(np.uint16)

        # Обработка RGB
        rgb_array = np.frombuffer(rgb_data, dtype=np.uint8).reshape((HEIGHT, WIDTH, 4))
        rgb_image = rgb_array[:, :, :3]  # Отбрасываем альфа-канал

        # Временная метка и сохранение
        timestamp = time.time()
        name = f"{timestamp:.3f}.png"

        imageio.imwrite(os.path.join(DEPTH_DIR, name), depth_uint16)
        imageio.imwrite(os.path.join(RGB_DIR, name), rgb_image)

        ts_file.write(f"{timestamp:.3f}\n")
        ts_file.flush()

        print(f"[SAVED] {name}   max={max_depth:.2f}m")
        time.sleep(0.2)
