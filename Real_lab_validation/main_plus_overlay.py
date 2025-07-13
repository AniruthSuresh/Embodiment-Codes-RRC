import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

import pandas as pd
import re
import shutil

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)
    
robot_id = p.loadURDF("/home/aniruth/Desktop/RRC/XARM7/xArm-Python-SDK/example/wrapper/xarm7/xarm7_robot.urdf", [0, 0,0], useFixedBase=True)


end_effector_link_index = 7

positions = []

csv_file_path = '../real_lab_bag_extract/cleaned_js.csv'  # Update with your actual file path

data = pd.read_csv(csv_file_path)


joint_states_array = np.array(data['joint_states'].str.split(', ').tolist(), dtype=float)

print(len(joint_states_array))

def move_to_joint_angles(joint_angles):

    current_joint_states = [p.getJointState(robot_id, i)[0] for i in range(len(joint_angles))]
    print(f"Current Joint States: {current_joint_states}")

    for i in range(len(joint_angles)):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i+1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=joint_angles[i]
        )

    for _ in range(500): 
        p.stepSimulation()
    
    actual_joint_states = [p.getJointState(robot_id, i)[0] for i in range(len(joint_angles))]
    # print(f"Actual Joint States: {actual_joint_states}")

def cvK2BulletP():
    """
    cvKtoPulletP converst the K interinsic matrix as calibrated using Opencv
    and ROS to the projection matrix used in openGL and Pybullet.

    :param K:  OpenCV 3x3 camera intrinsic matrix
    :param w:  Image width
    :param h:  Image height
    :near:     The nearest objects to be included in the render
    :far:      The furthest objects to be included in the render
    :return:   4x4 projection matrix as used in openGL and pybullet

    # https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12901
    """ 

    near = 0.1
    far = 3.1


    w = 1280 
    h = 720 



    # NOTE : Enter the K matrix from the camera_info.csv file here !! (IMPORTANT)
    K = np.array([
        [647.0324096679688,0.0,655.9091796875],
        [0.0,645.4093017578125,368.2861022949219],
        [0.0, 0.0, 1.0]
    ])

    f_x = K[0,0]
    f_y = K[1,1]
    c_x = K[0,2]
    c_y = K[1,2]

    A = (near + far)/(near - far)
    B = 2 * near * far / (near - far)

    projection_matrix = [
                        [2/w * f_x,  0,          (w - 2*c_x)/w,  0],
                        [0,          2/h * f_y,  (2*c_y - h)/h,  0],
                        [0,          0,          A,              B],
                        [0,          0,          -1,             0]]

    return np.array(projection_matrix).T.reshape(16).tolist()


def capture_and_filter_arm(camera_position, camera_orientation, file_name, arm_object_ids=None):
    if os.path.exists(file_name):
        os.remove(file_name)  

    rot_matrix = R.from_quat(camera_orientation).as_matrix()
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 1])

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=camera_target_position,
        cameraUpVector=[0, 0, 1]
    )

    """
    # https://reachpranjal19.medium.com/camera-calibration-in-ros-melodic-a0bf4742d636
    # https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/EPSRC_SSAZ/node3.html
    """




    proj_matrix = cvK2BulletP()

    """
    Scene 1
    """
    # width = 640 
    # height = 480

    """
    Scene 2
    """
    width = 1280 
    height = 720 

    
    _, _, _, _, seg_img = p.getCameraImage(
        width=width,           
        height=height,           
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL 
    )

    arm_object_ids = [1]


    seg_array = np.reshape(seg_img, (height, width))
    arm_mask = np.isin(seg_array, arm_object_ids).astype(np.uint8) * 255
    seg_mask = cv2.cvtColor(arm_mask, cv2.COLOR_GRAY2BGR)
    cv2.imwrite(file_name, seg_mask)

additional_image_dir = "seg_mask_images"
os.makedirs(additional_image_dir, exist_ok=True)



# NOTE : Use the cam_pos and angles from the output of the get_base_to_cam_col_opt_frame.py file
camera_position = [0.07349011, 0.55714059 ,0.40947183]
camera_orientation_euler = [-1.50299657 ,-0.02699565 ,-2.13372858]
camera_orientation = p.getQuaternionFromEuler(camera_orientation_euler)




def draw_camera_direction(camera_position, camera_orientation):

    rot_matrix = R.from_quat(camera_orientation).as_matrix()
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 2])
    print("hello")
    p.addUserDebugLine(camera_position, camera_target_position, lineColorRGB=[1, 0, 0], lineWidth=2)

draw_camera_direction(camera_position, camera_orientation)



visual_shape_id = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[0.05, 0.05, 0.05],
    rgbaColor=[0, 1, 0, 1]  
)

camera_body_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=-1,
    baseVisualShapeIndex=visual_shape_id,
    basePosition=camera_position,
    baseOrientation=camera_orientation
)


def draw_camera_direction(camera_position, camera_orientation):
    rot_matrix = R.from_quat(camera_orientation).as_matrix()
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 2])
    p.addUserDebugLine(camera_position, camera_target_position, lineColorRGB=[1, 0, 0], lineWidth=2)

# draw_camera_direction(camera_position, camera_orientation)

def extract_number(filename):
    """
    Extracts the first number found in the filename
    """

    numbers = re.findall(r'\d+', filename)
    return int(numbers[0]) if numbers else float('inf')


def overlay_images(folder1, folder2, output_folder, alpha=0.5):

    if os.path.exists(output_folder):
        shutil.rmtree(output_folder)  # Remove the entire output directory
    os.makedirs(output_folder)  

    image_filenames1 = sorted(os.listdir(folder1), key=extract_number)
    image_filenames2 = sorted(os.listdir(folder2), key=extract_number)


    if len(image_filenames1) != len(image_filenames2):
        print("Error: The number of images in both folders does not match.")
        return

    for index in range(len(image_filenames1)):
        img1_path = os.path.join(folder1, image_filenames1[index])
        img2_path = os.path.join(folder2, image_filenames2[index])

        print(img1_path)
        # Read the images
        img1 = cv2.imread(img1_path)
        img2 = cv2.imread(img2_path)

        if img1 is not None and img2 is not None:
            img2 = cv2.resize(img2, (img1.shape[1], img1.shape[0]))

            blended = cv2.addWeighted(img1, 1 - alpha, img2, alpha, 0)

            output_path = os.path.join(output_folder, f"overlay_{index}.png")
            cv2.imwrite(output_path, blended)

            print(f"Saved overlay image: {output_path}")
        else:
            print(f"Could not load images: {img1_path} or {img2_path}")


def frames_to_video(frames_folder, output_video, fps):
    frame_files = [f for f in os.listdir(frames_folder) if f.endswith('.png')]
    frame_files.sort(key=lambda x: int(re.search(r'(\d+)', x).group()))

    if not frame_files:
        print("No frame files found in the folder.")
        return
    
    first_frame_path = os.path.join(frames_folder, frame_files[0])
    frame = cv2.imread(first_frame_path)
    height, width, layers = frame.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    # Write each frame to the video
    for frame_file in frame_files:
        frame_path = os.path.join(frames_folder, frame_file)
        img = cv2.imread(frame_path)
        video.write(img)

    video.release()
    print(f"Video saved as {output_video}")



for idx, joint_angles in enumerate(joint_states_array):

    # print(f"Joint angle to {joint_angles}")

    move_to_joint_angles(joint_angles)
    image_name = os.path.join(additional_image_dir, f"image_{idx:04d}.png")
    capture_and_filter_arm(camera_position, camera_orientation, image_name)
    draw_camera_direction(camera_position, camera_orientation)



folder1 ="cleaned_image_today"  
folder2 = "seg_mask_images" 
output_folder = 'final_result'  


overlay_images(folder1, folder2, output_folder, alpha=0.5)

frames_to_video(output_folder , output_video = 'final_overlay' , fps= 50)

p.disconnect()




