import os
import h5py
import numpy as np

# Path to the list of folder numbers you want to process
folder_list_file = "folders.txt"   # <-- put your list here

# Source and target dirs
src_root = "../Actions-Observation/Droid-Extraction/data-final"
dst_root = "./lite-6-data/"

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

    # Open h5 file and extract joint_position
    with h5py.File(traj_file, "r") as f:
        joint_key = "action/joint_position"
        if joint_key in f:
            joint_pos = f[joint_key][:]
        else:
            print(f"  Warning: '{joint_key}' not found in {folder}")
            continue

    # Make destination folder
    dst_folder = os.path.join(dst_root, folder)
    os.makedirs(dst_folder, exist_ok=True)

    # Save joint positions
    dst_file = os.path.join(dst_folder, "joint_pos.txt")
    np.savetxt(dst_file, joint_pos, fmt="%.6f")
    print(f"  Saved joint_pos.txt to {dst_file}")
