import os
import h5py
import numpy as np
import subprocess
from pathlib import Path

# Function to write array rows as Python-like lists
def write_array_as_list(filename, array):
    array = np.atleast_2d(array)
    with open(filename, 'w') as f:
        for row in array:
            formatted = "[" + ", ".join(f"{val:.6f}" for val in row) + "]"
            f.write(formatted + "\n")

def get_svo_pose(relative_path,svo_file,output_base):

    rel_base = os.path.basename(relative_path)
    output_path = os.path.join(output_base,rel_base+svo_file[:-4])
    print(output_path)
    print(rel_base)
    os.makedirs(output_path,exist_ok=True) 

    traj_name = os.path.join("gs://gresearch/robotics/droid_raw/1.0.1/",relative_path,"trajectory.h5")
    traj_output = os.path.join(output_path,"trajectory.h5")

    svo_name = os.path.join("gs://gresearch/robotics/droid_raw/1.0.1/",relative_path,"recordings/SVO",svo_file)
    svo_output = os.path.join(output_path,svo_file)

    subprocess.run(["gsutil", "-m","cp","-r",traj_name,traj_output])
    subprocess.run(["gsutil", "-m","cp","-r",svo_name,svo_output])

    # Open the HDF5 file
    print(traj_output)

    with h5py.File(traj_output, 'r') as f:
        observation = f['observation']
        print(observation.keys())

        # Extract required data
        extrinsics_left = np.array(observation['camera_extrinsics'][svo_file[:-4]+'_left'][0])
        extrinsics_right = np.array(observation['camera_extrinsics'][svo_file[:-4]+'_right'][0])
        joint_positions = observation['robot_state']['joint_positions'][:]
        cartesian_position = observation['robot_state']['cartesian_position'][:]
        gripper_position = observation['robot_state']['gripper_position'][:]


    # Save arrays with brackets and commas
    write_array_as_list(os.path.join(output_path, "extrinsics_left.txt"), extrinsics_left)
    write_array_as_list(os.path.join(output_path, "extrinsics_right.txt"), extrinsics_right)
    write_array_as_list(os.path.join(output_path, "joint_ps.txt"), joint_positions)
    write_array_as_list(os.path.join(output_path, "cart_pos.txt"), cartesian_position)
    write_array_as_list(os.path.join(output_path, "gripper_pos.txt"), gripper_position)

if __name__ == "__main__":
    output_base = "/scratch/darshil/cross-emb-data"

    svo_list =   [
    ('WEIRD/success/2023-11-30/Thu_Nov_30_16:30:12_2023', '28834630.svo'),
    ('ILIAD/success/2023-07-12/Wed_Jul_12_21:35:14_2023', '23897859.svo'),
    ('IRIS/success/2023-06-06/Tue_Jun__6_11:52:15_2023', '29838012.svo'),
    ('TRI/success/2023-10-24/Tue_Oct_24_18:22:52_2023', '28451778.svo'),
    ('TRI/success/2023-11-28/Tue_Nov_28_14:05:05_2023', '20252535.svo'),
    ('TRI/success/2024-01-10/Wed_Jan_10_15:00:51_2024', '28451778.svo'),
    ('TRI/success/2024-01-10/Wed_Jan_10_15:12:19_2024', '28451778.svo'),
    ('TRI/success/2024-01-10/Wed_Jan_10_15:13:01_2024', '28451778.svo'),
    ('TRI/success/2024-01-10/Wed_Jan_10_15:20:58_2024', '28451778.svo'),
    ('TRI/success/2024-01-10/Wed_Jan_10_15:53:38_2024', '28451778.svo'),
    ('TRI/success/2024-01-10/Wed_Jan_10_15:54:03_2024', '28451778.svo'),
    ('TRI/failure/2023-11-28/Tue_Nov_28_14:02:40_2023', '20252535.svo')
    ]

    for relative_path,svo_file in svo_list:
        get_svo_pose(relative_path,svo_file,output_base)
