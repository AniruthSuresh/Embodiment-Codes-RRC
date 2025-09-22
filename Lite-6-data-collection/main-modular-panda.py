import os
import numpy as np
import pybullet as p
import pybullet_data
import cv2
import shutil
from scipy.spatial.transform import Rotation as R

def load_joint_positions(folder_path):
    joint_file = os.path.join(folder_path, "joint_pos.txt")
    traj = []
    with open(joint_file, "r") as f:
        for line in f:
            vals = [float(x) for x in line.strip().split()]
            traj.append(vals)
    return np.array(traj)

def load_extrinsics(folder_path):
    return np.loadtxt(os.path.join(folder_path, "extrinsics.txt"))

def load_intrinsics(folder_path):
    return np.loadtxt(os.path.join(folder_path, "intrinsics.txt"))

def cvK2BulletP(K, w, h, near=0.1, far=3.1):
    f_x, f_y, c_x, c_y = K[0,0], K[1,1], K[0,2], K[1,2]
    A = (near + far) / (near - far)
    B = 2 * near * far / (near - far)
    proj = [
        [2 / w * f_x, 0, (w - 2 * c_x) / w, 0],
        [0, 2 / h * f_y, (2 * c_y - h) / h, 0],
        [0, 0, A, B],
        [0, 0, -1, 0],
    ]
    return np.array(proj).T.reshape(16).tolist()

def update_intrinsic_matrix(K, old_dims, new_dims):
    old_height, old_width = old_dims
    new_height, new_width = new_dims
    scale_w = new_width / old_width
    scale_h = new_height / old_height
    K_updated = K.copy()
    K_updated[0, 0] *= scale_w  # fx
    K_updated[1, 1] *= scale_h  # fy
    K_updated[0, 2] *= scale_w  # cx
    K_updated[1, 2] *= scale_h  # cy
    return K_updated

def execute_and_record(folder_id, base_dir, panda_dir):
    folder_path = os.path.join(base_dir, str(folder_id))

    # Create subfolder for this folder_id
    out_folder = os.path.join(panda_dir, str(folder_id))
    rgb_out_dir = os.path.join(out_folder, "rgb-images")
    mask_out_dir = os.path.join(out_folder, "mask-images")

    # Remove old directories if they exist
    if os.path.exists(rgb_out_dir):
        shutil.rmtree(rgb_out_dir)
    if os.path.exists(mask_out_dir):
        shutil.rmtree(mask_out_dir)

    os.makedirs(rgb_out_dir)
    os.makedirs(mask_out_dir)

    # === Load data ===
    joint_traj = load_joint_positions(folder_path)
    extrinsics = load_extrinsics(folder_path)
    intrinsics = load_intrinsics(folder_path)
    width, height = 320, 180
    old_dims = (720, 1280)
    new_dims = (height, width)
    K_resized = update_intrinsic_matrix(intrinsics, old_dims, new_dims)

    # === Start PyBullet ===
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    robot_id = p.loadURDF(
        "../../../Exact_Panda/bullet3/examples/pybullet/gym/pybullet_data/franka_panda/panda_with_2F85_sec.urdf",
        [0, 0, 0], useFixedBase=True
    )

    # === Camera pose ===
    cam_pos = extrinsics[:3]
    cam_euler = extrinsics[3:]
    cam_quat = p.getQuaternionFromEuler(cam_euler)
    proj_matrix = cvK2BulletP(K_resized, width, height)

    # === Main loop ===
    for idx, joint_pos in enumerate(joint_traj):
        for j, val in enumerate(joint_pos):
            p.resetJointState(robot_id, j, val)
        for _ in range(50):
            p.stepSimulation()

        # Camera view
        rot_matrix = R.from_quat(cam_quat).as_matrix()
        cam_target = cam_pos + rot_matrix @ np.array([0, 0, 1])
        view_matrix = p.computeViewMatrix(cam_pos, cam_target, [0, 0, 1])

        # Render
        _, _, rgb, _, seg = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        # --- Save RGB ---
        rgb_array = np.reshape(rgb, (height, width, 4))
        bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGBA2BGR)
        rgb_img_path = os.path.join(rgb_out_dir, f"rgb_{idx:04d}.png")
        cv2.imwrite(rgb_img_path, bgr_array)

        # --- Save Mask ---
        seg_array = np.reshape(seg, (height, width))
        mask = np.isin(seg_array, [robot_id]).astype(np.uint8) * 255
        mask_img_path = os.path.join(mask_out_dir, f"mask_{idx:04d}.png")
        cv2.imwrite(mask_img_path, mask)

    p.disconnect()
    print(f"Folder {folder_id}: Saved RGB and mask images in {out_folder}.")

if __name__ == "__main__":
    base_dir = "/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/Follow_DROID/Franka_arm/Droid Mask Extraction/Lite-6-data-collection/lite-6-data"
    panda_dir = "./Panda-data"

    folder_ids = [f for f in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, f))]
    folder_ids.sort(key=lambda x: int(x))  # numeric sort

    for folder_id in folder_ids:
        print(f"Processing folder {folder_id} ...")
        try:
            execute_and_record(folder_id, base_dir, panda_dir)
        except Exception as e:
            print(f"Error in folder {folder_id}: {e}")
