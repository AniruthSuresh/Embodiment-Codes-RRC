import os
import h5py
import numpy as np
import json

# Path to the list of folder numbers you want to process
folder_list_file = "folders.txt"   # <-- put your list here

# Source and target dirs
src_root = "../Actions-Observation/Droid-Extraction/data-final"
dst_root = "./lite-6-execute-traj"

# Path to the camera intrinsics JSON
camera_intrinsics_file = os.path.join(src_root, "camera_intrinsics.json")

# Load camera intrinsics JSON once
with open(camera_intrinsics_file, "r") as f:
    camera_intrinsics = json.load(f)

# Make target root if not exists
os.makedirs(dst_root, exist_ok=True)

# Read folder numbers from txt file
with open(folder_list_file, "r") as f:
    folder_list = [line.strip() for line in f if line.strip()]

print(f"Found {len(folder_list)} folders to process: {folder_list}")

# Iterate through specified folders
for folder in folder_list:
    folder_path = os.path.join(src_root, folder)
    traj_file = os.path.join(folder_path, "trajectory.h5")

    if not os.path.isfile(traj_file):
        print(f"Skipping {folder}: trajectory.h5 not found")
        continue

    print(f"Processing folder {folder}...")

    # Open h5 file and extract cartesian_position
    with h5py.File(traj_file, "r") as f:
        key = "action/cartesian_position"
        if key not in f:
            print(f"  Skipping {folder}: '{key}' not found")
            continue
        cart_pos = f[key][:]

    # Make destination folder
    dst_folder = os.path.join(dst_root, folder)
    os.makedirs(dst_folder, exist_ok=True)

    # Save cartesian positions
    dst_file = os.path.join(dst_folder, "cart_pos.txt")
    np.savetxt(dst_file, cart_pos, fmt="%.6f")
    print(f"  Saved cart_pos.txt to {dst_file}")

    # --- Save intrinsics and extrinsics ---
    if folder in camera_intrinsics:
        intrinsics = np.array(camera_intrinsics[folder]["intrinsics_matrix"])
        extrinsics = np.array(camera_intrinsics[folder]["extrinsics_pose"])

        np.savetxt(os.path.join(dst_folder, "intrinsics.txt"), intrinsics, fmt="%.6f")
        np.savetxt(os.path.join(dst_folder, "extrinsics.txt"), extrinsics.reshape(1, -1), fmt="%.6f")
        print(f"  Saved intrinsics.txt and extrinsics.txt for {folder}")
    else:
        print(f"  Warning: no intrinsics/extrinsics found in JSON for {folder}")
