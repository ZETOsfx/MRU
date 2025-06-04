import sqlite3
import mmap
import os
import struct
import math
import time
import re  # Для разбора timestamp из имени файла

# Путь до базы данных RTAB-Map
DB_PATH = "/Users/akko/Documents/RTAB-Map/rtabmap.tmp.db"
SHM_PATH = "/tmp/UE5_Ctrl"
# Размер памяти — 2 float значения: linear_speed и angular_speed
SHM_SIZE = 2 * 4
# Количество интерполируемых точек между узлами графа
STEPS = 20
# Ограничения по скорости
MAX_FORWARD = 20.0  # см/с
MAX_TURN = 20.0     # град/с
# Задержка между командами
DELAY = 0.05        # 50 мс

# Ожидание, пока RTAB-Map не сформирует минимальное число нод в начальной позиции
def wait_for_initial_map(conn, min_nodes = 5):
    print(f"[WAIT] Waiting for RTAB-Map to initialize with at least {min_nodes} nodes...")
    while True:
        cur = conn.cursor()
        cur.execute("SELECT COUNT(*) FROM Node")
        count = cur.fetchone()[0]
        if count >= min_nodes:
            print(f"[INFO] RTAB-Map initialized with {count} nodes.")
            break
        time.sleep(0.5)

# --- Извлечение координат позиции узла из бинарного поля RTAB-Map ---
def parse_pose_blob(blob):
    if not blob or len(blob) < 48:
        raise ValueError("Invalid pose BLOB")
    floats = struct.unpack('<12f', blob[:48])
    return floats[3], floats[7], floats[11]  # X, Y, Z

# --- Загрузка всех узлов и их позиций из базы RTAB-Map ---
def load_node_positions(conn):
    cur = conn.cursor()
    cur.execute("SELECT id, pose FROM Node")
    poses = {}
    for node_id, blob in cur.fetchall():
        try:
            x, y, z = parse_pose_blob(blob)
            poses[node_id] = (x, y, z)
        except:
            continue
    print(f"[INFO] Loaded {len(poses)} valid node poses")
    return poses

# --- Поиск ближайшего узла графа к указанной позиции (по XZ) ---
def find_nearest_node(poses, x, z):
    return min(poses.items(), key=lambda item: math.hypot(item[1][0] - x, item[1][2] - z))[0]

# --- Загрузка связей между узлами (граф карты) ---
def load_links(conn):
    cur = conn.cursor()
    cur.execute("SELECT from_id, to_id FROM Link")
    graph = {}
    for from_id, to_id in cur.fetchall():
        graph.setdefault(from_id, []).append(to_id)
    return graph

# --- Поиск пути между двумя узлами по графу (поиск в ширину) ---
def bfs_path(start, goal, graph):
    visited = set()
    queue = [(start, [start])]
    while queue:
        current, path = queue.pop(0)
        if current == goal:
            return path
        if current in visited:
            continue
        visited.add(current)
        for neighbor in graph.get(current, []):
            if neighbor not in visited:
                queue.append((neighbor, path + [neighbor]))
    return None

# --- Интерполяция пути между точками для сглаживания движения ---
def interpolate_path(points, steps=STEPS):
    path = []
    for i in range(len(points) - 1):
        p0, p1 = points[i], points[i + 1]
        for s in range(steps):
            t = s / steps
            x = p0[0] + (p1[0] - p0[0]) * t
            y = p0[1] + (p1[1] - p0[1]) * t
            z = p0[2] + (p1[2] - p0[2]) * t
            path.append((x, y, z))
    path.append(points[-1])
    return path

# --- Расчет угла поворота между двумя точками ---
def compute_yaw_angle(p0, p1):
    dx = p1[0] - p0[0]
    dz = p1[2] - p0[2]
    return math.degrees(math.atan2(dz, dx))

# --- Построение списка команд (скорость + поворот) для движения по пути ---
def compute_commands(path):
    commands = []
    last_yaw = compute_yaw_angle(path[0], path[1])
    for i in range(len(path) - 1):
        p0, p1 = path[i], path[i + 1]

        dx = p1[0] - p0[0]
        dz = p1[2] - p0[2]
        dist = math.hypot(dx, dz)
        forward = min(MAX_FORWARD, dist * 100)  # Преобразуем в см/с

        target_yaw = compute_yaw_angle(p0, p1)
        delta_yaw = (target_yaw - last_yaw + 180) % 360 - 180
        turn = max(min(delta_yaw / DELAY, MAX_TURN), -MAX_TURN)

        # Замедление при резком повороте
        if abs(turn) > 10:
            forward *= 0.75
        if abs(turn) > 15:
            forward *= 0.5

        last_yaw = target_yaw
        commands.append((forward, turn))
    return commands

# --- Подключение к Shared Memory. Ждем, пока создастся файл ---
def open_shared_memory():
    if not os.path.exists(SHM_PATH):
        print("[WAIT] Waiting for UE5 to start and create shared memory...")
        while not os.path.exists(SHM_PATH):
            time.sleep(0.2)

    fd = os.open(SHM_PATH, os.O_RDWR)
    return mmap.mmap(fd, SHM_SIZE, mmap.MAP_SHARED, mmap.PROT_WRITE)

# --- Отправка последовательности команд через Shared Memory ---
def send_commands(commands):
    shm = open_shared_memory()
    try:
        for forward, turn in commands:
            shm.seek(0)
            shm.write(struct.pack('<2f', forward, turn))
            print(f"Sent: speed = {forward:.2f} cm/s, turn = {turn:.2f} deg/s")
            time.sleep(DELAY)

        # Финальная команда — стоп
        shm.seek(0)
        shm.write(struct.pack('<2f', 0.0, 0.0))
    finally:
        shm.close()
        try:
            os.remove(SHM_PATH)
            print("[INFO] Shared memory removed.")
        except Exception as e:
            print(f"[WARN] Could not remove shared memory: {e}")

# --- Получение timestamp всех обработанных нод из RTAB-Map ---
def get_rtabmap_timestamps(conn):
    cur = conn.cursor()
    cur.execute("SELECT stamp FROM Node")
    timestamps = set()
    for (ts,) in cur.fetchall():
        if isinstance(ts, float) or isinstance(ts, int):
            timestamps.add(f"{ts:.3f}")
    return timestamps

# --- Удаление изображений с обработанными timestamp ---
def remove_consumed_images(used_timestamps, depth_dir="depth", rgb_dir="rgb"):
    removed_count = 0
    for directory in [depth_dir, rgb_dir]:
        for filename in os.listdir(directory):
            match = re.match(r"(\d+\.\d+)\.png", filename)
            if match:
                timestamp = match.group(1)
                if timestamp in used_timestamps:
                    file_path = os.path.join(directory, filename)
                    try:
                        os.remove(file_path)
                        removed_count += 1
                    except Exception as e:
                        print(f"[WARN] Could not delete {file_path}: {e}")
    print(f"[CLEANUP] Removed {removed_count} used image files.")


def main():
    conn = sqlite3.connect(DB_PATH)

    # Подождать, пока RTAB-Map не начнет строить карту
    wait_for_initial_map(conn)

    # Загрузка данных RTAB-Map
    poses = load_node_positions(conn)
    graph = load_links(conn)

    # Очистка обработанных изображений
    used_ts = get_rtabmap_timestamps(conn)
    remove_consumed_images(used_ts)

    # Выбор начальной и целевой точек (в глобальных координатах)
    start_xy = (-300, 0)
    goal_xy = (300, 0)

    start_node = find_nearest_node(poses, *start_xy)
    goal_node = find_nearest_node(poses, *goal_xy)

    print(f"Start Node: {start_node} → {poses[start_node]}")
    print(f"Goal Node:  {goal_node} → {poses[goal_node]}")

    # Построение пути по графу
    path_ids = bfs_path(start_node, goal_node, graph)
    if path_ids:
        path_positions = [poses[nid] for nid in path_ids]
        full_path = interpolate_path(path_positions)
        print(f"[INFO] Following graph path with {len(full_path)} steps.")
    else:
        print("[WARN] Graph disconnected — fallback to straight line.")
        full_path = interpolate_path([poses[start_node], poses[goal_node]])

    # Команды и передача
    commands = compute_commands(full_path)
    send_commands(commands)

if __name__ == "__main__":
    main()