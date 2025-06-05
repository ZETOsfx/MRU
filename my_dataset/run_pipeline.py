import subprocess
import time
import os
import sqlite3
import signal

DB_PATH = "/Users/akko/Documents/RTAB-Map/rtabmap.tmp.db"
SHM_PATH = "/tmp/UE5_Ctrl"
MIN_NODES = 5

def wait_for_rtabmap_nodes():
    print(f"[WAIT] RTAB-Map: ожидание минимум {MIN_NODES} узлов...")
    while True:
        try:
            conn = sqlite3.connect(DB_PATH)
            cur = conn.cursor()
            cur.execute("SELECT COUNT(*) FROM Node")
            count = cur.fetchone()[0]
            conn.close()
            if count >= MIN_NODES:
                print(f"[OK] RTAB-Map построил {count} узлов.")
                break
        except Exception:
            pass
        time.sleep(0.5)

def main():
    print("Запуск move_controller.py")
    move_proc = subprocess.Popen(["python3", "move_controller.py"])

    print("Вручную запусти UE5 проект.")
    input("▶ Когда UE5 полностью загрузится, нажми [Enter]...\n")

    print("[START] sm_saver.py")
    saver_proc = subprocess.Popen(["python3", "sm_saver.py"])

    print("[START] RTAB-Map")
    rtabmap_proc = subprocess.Popen([
        "rtabmap",
        "--RGBD", "images",
        "--delete_db_on_start",
        "--Rtabmap/DetectionRate", "5"
    ])

    wait_for_rtabmap_nodes()

    print("\nВсе процессы запущены корректно.")
    print("⏹ Для завершения — нажми Ctrl+C\n")

    try:
        move_proc.wait()
        rtabmap_proc.wait()
        saver_proc.wait()
    except KeyboardInterrupt:
        print("\n[SHUTDOWN] Завершаем все процессы...")
        for proc in [move_proc, rtabmap_proc, saver_proc]:
            proc.terminate()
        time.sleep(1)

if __name__ == "__main__":
    main()
