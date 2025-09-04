import os
import cv2
import time
import shutil
import pybullet as p
import pybullet_data
import numpy as np
import h5py
from scipy.spatial.transform import Rotation as R
import json
from tqdm import tqdm


# ============================================================
# PyBullet Setup
# ============================================================
def setup_simulation():
    """Initializes PyBullet, loads the plane, and sets gravity."""
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1.0 / 240.0)

# ============================================================
# Data Loading
# ============================================================
def load_trajectory_data_from_h5(filepath):
    """Loads arm and gripper trajectories from an HDF5 file."""
    cartesian_path = 'action/robot_state/cartesian_position'
    gripper_path = 'action/gripper_position'
    try:
        with h5py.File(filepath, 'r') as f:
            positions = f[cartesian_path][:, :3]
            orientations_euler = f[cartesian_path][:, 3:6]
            gripper_positions = f[gripper_path][:]
            return positions, orientations_euler, gripper_positions
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return None, None, None

def save_to_txt(filepath, data_log):
    with open(filepath, 'w') as f:
        for row in data_log:
            f.write(' '.join(map(str, row)) + '\n')

# ============================================================
# Robot Control
# ============================================================
def move_to_pose_and_get_action(robot_id, end_effector_link_index, gripper_joint_indices,
                                target_position, target_orientation, gripper_position):
    ik_joint_positions = p.calculateInverseKinematics(
        robot_id, end_effector_link_index, target_position, target_orientation
    )
    for i in range(len(ik_joint_positions)):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id, jointIndex=i + 1,
            controlMode=p.POSITION_CONTROL, targetPosition=ik_joint_positions[i]
        )
    for joint_index in gripper_joint_indices:
        joint_name = p.getJointInfo(robot_id, joint_index)[1].decode('utf-8')
        if 'right_inner_knuckle_joint' in joint_name:
            p.setJointMotorControl2(
                bodyUniqueId=robot_id, jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL, targetPosition=gripper_position
            )
    return ik_joint_positions

def update_intrinsic_matrix(K, old_dims, new_dims):
    old_h, old_w = old_dims
    new_h, new_w = new_dims
    scale_w, scale_h = new_w / old_w, new_h / old_h
    K_new = K.copy()
    K_new[0, 0] *= scale_w; K_new[1, 1] *= scale_h
    K_new[0, 2] *= scale_w; K_new[1, 2] *= scale_h

    return K_new

def cvK2BulletP(K_old, old_dims, new_dims, w=320, h=180, near=0.1, far=3.1):
    K = update_intrinsic_matrix(K_old, old_dims, new_dims)
    f_x, f_y, c_x, c_y = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    A = (near + far) / (near - far)
    B = 2 * near * far / (near - far)
    proj_matrix = [
        [2/w*f_x, 0, (w-2*c_x)/w, 0],
        [0, 2/h*f_y, (2*c_y-h)/h, 0],
        [0,0,A,B],
        [0,0,-1,0]
    ]
    return np.array(proj_matrix).T.reshape(16).tolist()

def capture_image(camera_position, camera_orientation, proj_matrix, file_name):
    rot_matrix = R.from_quat(camera_orientation).as_matrix()
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 1])
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=camera_target_position,
        cameraUpVector=[0, 0, 1]
    )
    h, w = 180, 320
    _, _, rgb_img, _, _ = p.getCameraImage(
        width=w, height=h, viewMatrix=view_matrix,
        projectionMatrix=proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL
    )
    rgb_array = np.reshape(rgb_img, (h, w, 4))[:, :, :3]  # drop alpha
    cv2.imwrite(file_name, cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR))

# ============================================================
# Main
# ============================================================
def run_episode(ep_info, traj_h5_path, episode_dir):
    positions, orientations_euler, gripper_positions = load_trajectory_data_from_h5(traj_h5_path)
    if positions is None:
        return

    setup_simulation()
    robot_id = p.loadURDF(
        "/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/airobot/src/airobot/urdfs/xarm7_robot.urdf",
        [0, 0, 0], useFixedBase=True
    )

    num_joints = p.getNumJoints(robot_id)
    gripper_joint_indices = []
    arm_joint_indices = list(range(1, 8))
    for i in range(num_joints):
        jn = p.getJointInfo(robot_id, i)[1].decode('utf-8')
        if 'left_' in jn or 'right_' in jn or 'finger_joint' in jn:
            gripper_joint_indices.append(i)

    actions_log, observations_log = [], []

    # --- Camera Setup from JSON ---
    cam_pos = np.array(ep_info["extrinsics_pose"][:3])
    cam_quat = R.from_euler('xyz', ep_info["extrinsics_pose"][3:6]).as_quat()
    K_old = np.array(ep_info["intrinsics_matrix"])
    old_dims = (ep_info["height"], ep_info["width"])

    old_h, old_w = ep_info.get("height", 0), ep_info.get("width", 0)

    if old_h == 0 or old_w == 0:
        print(f" Skipping episode {ep_info['episode_id']} due to invalid camera dims: ({old_h}, {old_w})")
        return

    proj_matrix = cvK2BulletP(K_old, old_dims=old_dims, new_dims=(180, 320))

    # --- Directories ---
    rgb_dir = os.path.join(episode_dir, "rgb_images")
    if os.path.exists(rgb_dir):
        shutil.rmtree(rgb_dir)
    os.makedirs(rgb_dir)

    # --- Simulation Loop ---
    end_effector_link_index = 7
    control_freq = 15.0; sim_freq = 240.0
    steps_per_action = int(sim_freq / control_freq)

    for i in range(len(positions)):
        target_pos = positions[i]
        target_ori_quat = p.getQuaternionFromEuler(orientations_euler[i])
        gripper_pos = gripper_positions[i]
        commanded_action = move_to_pose_and_get_action(
            robot_id, end_effector_link_index, gripper_joint_indices,
            target_pos, target_ori_quat, gripper_pos
        )
        actions_log.append(commanded_action)

        for _ in range(steps_per_action):
            p.stepSimulation()
            time.sleep(1.0 / sim_freq)

        joint_states = p.getJointStates(robot_id, arm_joint_indices)
        current_obs = [s[0] for s in joint_states]
        observations_log.append(current_obs)

        img_name = os.path.join(rgb_dir, f"image_{i:04d}.png")
        capture_image(cam_pos, cam_quat, proj_matrix, img_name)

    p.disconnect()
    save_to_txt(os.path.join(episode_dir, "actions.txt"), actions_log)
    save_to_txt(os.path.join(episode_dir, "observations.txt"), observations_log)


# ============================================================
# Entry Point
# ============================================================


def main():
    with open("../Actions-Observation/Droid-Extraction/data/camera_intrinsics.json", "r") as f:
        episodes_info = json.load(f)

    skipped_file = "skipped_episodes.txt"
    with open(skipped_file, "w") as sf:  # overwrite on each run
        for ep_id, ep_info in tqdm(episodes_info.items(), total=len(episodes_info), desc="Episodes"):
            episode_id = ep_info["episode_id"]
            traj_h5_path = os.path.join("../Actions-Observation/Droid-Extraction/data", ep_id, "trajectory.h5")

            if not os.path.exists(traj_h5_path):
                msg = f"Skipping {episode_id}, no trajectory.h5 found at {traj_h5_path}"
                print(msg)
                sf.write(msg + "\n")
                continue

            print(f"\n=== Running episode {episode_id} ===")
            episode_dir = os.path.join("../Actions-Observation/Droid-Extraction/data", ep_id)
            try:
                run_episode(ep_info, traj_h5_path, episode_dir)
            except Exception as e:
                msg = f"Skipping {episode_id} due to error: {e}"
                print(msg)
                sf.write(msg + "\n")
                continue

    print(f"\n Skipped episodes written to {skipped_file}")

if __name__ == "__main__":
    main()