import os

shared_paths = [
    "/tmp/UE5_Ctrl",
    "/tmp/UE5_SharedMemory_Depth",
    "/tmp/UE5_SharedMemory_RGB"
]

for path in shared_paths:
    try:
        if os.path.exists(path):
            os.remove(path)
            print(f"Deleted: {path}")
        else:
            print(f"Not found: {path}")
    except Exception as e:
        print(f"Delete error {path}: {e}")