import os
import cv2
import time
import shutil
import pybullet as p
import pybullet_data
import numpy as np
import h5py
from scipy.spatial.transform import Rotation as R

# ============================================================
# PyBullet Setup
# ============================================================
def setup_simulation():
    """Initializes PyBullet, loads the plane, and sets gravity."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1.0 / 240.0)

# ============================================================
# Data Loading and Saving
# ============================================================
# CORRECTED: This function now correctly separates position and euler angles
def load_trajectory_data_from_h5(filepath):
    """Loads arm and gripper trajectories from an HDF5 file."""
    print(f"Loading data from '{filepath}'...")
    cartesian_path = 'action/robot_state/cartesian_position'
    gripper_path = 'action/gripper_position'
    
    try:
        with h5py.File(filepath, 'r') as f:
            # Assuming data is [x, y, z, roll, pitch, yaw]
            positions = f[cartesian_path][:, :3]
            orientations_euler = f[cartesian_path][:, 3:6] # Reading 3 Euler angles
            gripper_positions = f[gripper_path][:]
            print("Data loaded successfully.")
            return positions, orientations_euler, gripper_positions
    except Exception as e:
        print(f"An error occurred while reading the HDF5 file: {e}")
        return None, None, None

def save_to_txt(filepath, data_log):
    """Saves a log to a text file."""
    print(f"Saving data to '{filepath}'...")
    with open(filepath, 'w') as f:
        for row in data_log:
            line = ' '.join(map(str, row))
            f.write(line + '\n')
    print(f"Successfully wrote {len(data_log)} lines.")

# ============================================================
# Robot Control
# ============================================================
def move_to_pose_and_get_action(robot_id, end_effector_link_index, gripper_joint_indices,
                                target_position, target_orientation, gripper_position):
    """
    Calculates IK, sets target positions for arm and gripper, and returns the IK solution.
    """
    ik_joint_positions = p.calculateInverseKinematics(
        robot_id, 
        end_effector_link_index, 
        target_position, 
        target_orientation # This now receives a proper quaternion
    )
    
    for i in range(len(ik_joint_positions)):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i + 1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=ik_joint_positions[i]
        )

    for joint_index in gripper_joint_indices:
        joint_name = p.getJointInfo(robot_id, joint_index)[1].decode('utf-8')
        if 'right_inner_knuckle_joint' in joint_name:
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=gripper_position
            )
    
    return ik_joint_positions


def update_intrinsic_matrix(K, old_dims, new_dims):
    old_height, old_width = old_dims
    new_height, new_width = new_dims
    scale_w, scale_h = new_width / old_width, new_height / old_height
    K_updated = K.copy()
    K_updated[0, 0] *= scale_w; K_updated[1, 1] *= scale_h
    K_updated[0, 2] *= scale_w; K_updated[1, 2] *= scale_h
    return K_updated

def cvK2BulletP(K_old, old_dims, new_dims, w=320, h=180, near=0.1, far=3.1):
    K = update_intrinsic_matrix(K=K_old, old_dims=old_dims, new_dims=new_dims)
    f_x, f_y, c_x, c_y = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    A = (near + far) / (near - far)
    B = 2 * near * far / (near - far)
    projection_matrix = [[2/w*f_x, 0, (w-2*c_x)/w, 0], [0, 2/h*f_y, (2*c_y-h)/h, 0], [0,0,A,B], [0,0,-1,0]]
    return np.array(projection_matrix).T.reshape(16).tolist()

def capture_image(robot_id, camera_position, camera_orientation, proj_matrix, file_name):
    rot_matrix = R.from_quat(camera_orientation).as_matrix()
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 1])
    view_matrix = p.computeViewMatrix(cameraEyePosition=camera_position, cameraTargetPosition=camera_target_position, cameraUpVector=[0, 0, 1])
    height, width = 180, 320
    _, _, _, _, seg_img = p.getCameraImage(width=width, height=height, viewMatrix=view_matrix, projectionMatrix=proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
    seg_array = np.reshape(seg_img, (height, width))
    arm_mask = np.isin(seg_array, [robot_id]).astype(np.uint8) * 255
    seg_mask_bgr = cv2.cvtColor(arm_mask, cv2.COLOR_GRAY2BGR)
    cv2.imwrite(file_name, seg_mask_bgr)


# ============================================================
# Main Execution
# ============================================================
def main():
    """Main function to run the simulation."""
    setup_simulation()
    
    robot_id = p.loadURDF(
        "/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/airobot/src/airobot/urdfs/xarm7_robot.urdf",
        [0, 0, 0],
        useFixedBase=True
    )
    
    # --- Load Trajectory Data from H5 ---
    positions, orientations_euler, gripper_positions = load_trajectory_data_from_h5('trajectory.h5')
    if positions is None:
        p.disconnect()
        return

    # --- Identify Joints ---
    num_joints = p.getNumJoints(robot_id)
    gripper_joint_indices = []
    arm_joint_indices = list(range(1, 8))
    for i in range(num_joints):
        joint_name = p.getJointInfo(robot_id, i)[1].decode('utf-8')
        if 'left_' in joint_name or "right_" in joint_name or 'finger_joint' in joint_name:
            gripper_joint_indices.append(i)

    # --- Initialize logs and directories ---
    actions_log, observations_log = [], []
    filtered_image_dir = "filtered_arm_pics_fin_rlds_xarm"
    if os.path.exists(filtered_image_dir): shutil.rmtree(filtered_image_dir)
    os.makedirs(filtered_image_dir)

    # --- Camera Setup ---
    camera_position = [0.085036, 0.563473, 0.416859]
    camera_orientation = p.getQuaternionFromEuler([-1.95721, -0.0233935, -2.11812])
    K_old = np.array([[524.24609375, 0., 639.77758789],[0., 524.24609375, 370.27789307],[0., 0., 1.]])
    proj_matrix = cvK2BulletP(K_old, old_dims=(720, 1280), new_dims=(180, 320))

    # --- Simulation Loop ---
    end_effector_link_index = 7
    control_freq = 15.0
    sim_freq = 240.0
    steps_per_action = int(sim_freq / control_freq)

    for i in range(len(positions)):
        target_pos = positions[i]
        # CORRECTED: Convert Euler angles from H5 file to a Quaternion for PyBullet
        target_ori_quat = p.getQuaternionFromEuler(orientations_euler[i])
        gripper_pos = gripper_positions[i] 

        print(f"Executing Step {i+1}/{len(positions)}")

        commanded_action = move_to_pose_and_get_action(
            robot_id, end_effector_link_index, gripper_joint_indices,
            target_pos, target_ori_quat, gripper_pos
        )
        actions_log.append(commanded_action)
        
        for _ in range(steps_per_action):
            p.stepSimulation()
            time.sleep(1.0 / sim_freq)

        joint_states = p.getJointStates(robot_id, arm_joint_indices)
        current_observation = [state[0] for state in joint_states]
        observations_log.append(current_observation)

        image_name = os.path.join(filtered_image_dir, f"image_{i:04d}.png")
        capture_image(robot_id, camera_position, camera_orientation, proj_matrix, image_name)

    print("\nTrajectory finished.")
    p.disconnect()

    # --- Save the recorded data ---
    save_to_txt("actions.txt", actions_log)
    save_to_txt("observations.txt", observations_log)
    print("\nProcess finished. Logs and images have been saved.")

if __name__ == "__main__":
    main()



