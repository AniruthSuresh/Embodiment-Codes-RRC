"""
This file considers Franka Panda and joint position 

NOTE : In case of Franka Panda, we are using the joint positions directly whereas in case of Xarm, we are using the inverse kinematics to get the joint positions !!
"""

import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import os
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import shutil
import re
import ast


# when the .txt files are properly formatted

def get_joint_positions(joint_ps_file):

    joint_positions = []
    with open(joint_ps_file, "r") as file:
        for line in file:
            try:
                joint_positions.append(eval(line.strip()))  # Read joint positions as a list
            except SyntaxError as e:
                print(f"Syntax error in line: {line.strip()}")
                print(e)    
    return joint_positions

def get_cartesians_positions(cart_ps_file):

    cart_positions = []
    with open(cart_ps_file, "r") as file:
        for line in file:
            cart_positions.append(eval(line.strip()))

    return cart_positions

def move_to_joint_position_with_feedback(robot_id,joint_positions):
    # Use the joint positions directly and move the robot arm accordingly.
    for i in range(len(joint_positions)):
        p.resetJointState(robot_id, i, joint_positions[i])

    for _ in range(500):  # Step through the simulation to allow motion.
        p.stepSimulation()

def move_to_position_with_feedback(robot_id,end_effector_link_index,target_position, target_orientation):

    ik_joint_positions = p.calculateInverseKinematics(
        robot_id, 
        end_effector_link_index, 
        target_position, 
        target_orientation)
    
    for i in range(len(ik_joint_positions)):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=i+1,
            controlMode=p.POSITION_CONTROL,
            targetPosition=ik_joint_positions[i]
        )

    for _ in range(3000): 
        p.stepSimulation()
    


def update_intrinsic_matrix(K, old_dims, new_dims):
    """
    Update the intrinsic matrix K based on new image dimensions.
    """
    
    # NOTE :  Mention the site later !

    old_height, old_width = old_dims
    new_height, new_width = new_dims

    scale_w = new_width / old_width
    scale_h = new_height / old_height

    K_updated = K.copy()
    K_updated[0, 0] *= scale_w  # Scale fx
    K_updated[1, 1] *= scale_h  # Scale fy
    K_updated[0, 2] *= scale_w  # Scale cx
    K_updated[1, 2] *= scale_h  # Scale cy

    return K_updated

def cvK2BulletP(K,h=180,w=320):
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

    old_dims = (720 , 1280)
    new_dims = (h , w)

    K = np.array(K)
    
    K = update_intrinsic_matrix(K = K , old_dims = old_dims , new_dims = new_dims)
    print(K)


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

def capture_image(camera_position, camera_orientation, file_name, file_name_rgb, K ,height=180, width=320):

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

    proj_matrix = cvK2BulletP(K, height, width)

    _, _, rgb_img, depth_img, seg_img = p.getCameraImage(
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

    rgb_array = np.reshape(rgb_img, (height, width, 4))[:, :, :3]
    rgb_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
    cv2.imwrite(file_name_rgb, rgb_array)

def prepare_output_directories(base_path, arm = "franka", direction = "left", full_res = False):

    if full_res == True:
        suffix = "_hd"
    else:
        suffix = ""

    masks_dir = os.path.join(base_path, "simulation", arm, direction, "masks" + suffix)
    img_dir = os.path.join(base_path, "simulation", arm, direction, "rgb_images" + suffix)

    if os.path.exists(masks_dir):
        shutil.rmtree(masks_dir)

    os.makedirs(masks_dir)

    if os.path.exists(img_dir):
        shutil.rmtree(img_dir)

    os.makedirs(img_dir)

    return masks_dir,img_dir

def draw_camera_direction(camera_position, camera_orientation):
    rot_matrix = R.from_quat(camera_orientation).as_matrix()
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 0.1])
    p.addUserDebugLine(camera_position, camera_target_position, lineColorRGB=[1, 0, 0], lineWidth=2)

def overlay_images_to_video(folder1, folder2, output_folder='overlayed_images', video_output='overlayed_video.mp4', frame_rate=30):
    """
    Overlay images from two folders, save the result, and create a video from the overlayed images.
    """

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    images1 = [f for f in os.listdir(folder1) if f.endswith('.png')]
    images2 = [f for f in os.listdir(folder2) if f.endswith('.png')]

    images1.sort(key=lambda x: int(re.search(r'(\d+)', x).group()))
    images2.sort(key=lambda x: int(re.search(r'(\d+)', x).group()))

    num_images = min(len(images1), len(images2))

    if num_images == 0:
        print("Error: No images to overlay.")
        return

    img1 = cv2.imread(os.path.join(folder1, images1[0]))
    if img1 is None:
        print("Error: Could not read the first image in the folder.")
        return
    height, width, _ = img1.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    video_writer = cv2.VideoWriter(video_output, fourcc, frame_rate, (width, height))

    for i in range(num_images):
        img1_path = os.path.join(folder1, images1[i])
        img2_path = os.path.join(folder2, images2[i])

        img1 = cv2.imread(img1_path)
        img2 = cv2.imread(img2_path)

        if img1 is None or img2 is None:
            print(f"Error: Could not read images {images1[i]} or {images2[i]}")
            continue

        # print(img1.shape)
        # print(img2.shape)

        overlay = cv2.addWeighted(img1, 0.5, img2, 0.5, 0)

        output_img_name = f"overlayed_{images1[i]}"
        output_img_path = os.path.join(output_folder, output_img_name)
        cv2.imwrite(output_img_path, overlay)

        video_writer.write(overlay)

    video_writer.release()

    print(f"Overlayed video saved as '{video_output}'.")

def get_camera_intrinsics(intrinsics_txt):

    with open(intrinsics_txt, 'r') as f:
        content = f.read()

    sections = content.split('Camera Intrinsics (Left):')
    if len(sections) < 2:
        raise ValueError("Could not find 'Camera Intrinsics (Left)' in file")

    left_str = sections[1].split('\n\n')[0].strip()

    left_str = left_str.replace('array', 'np.array')

    left_intrinsics = eval(left_str, {'np': np})

    camera_matrix = left_intrinsics['cameraMatrix']
    dist_coeffs = left_intrinsics['distCoeffs']

    return camera_matrix

def get_camera_pose(file_path):

    with open(file_path, 'r') as f:
        content = f.read().strip()  # Read and remove leading/trailing whitespace

    data_list = ast.literal_eval(content)

    return data_list[:3],data_list[3:]

def get_gripper_position(file_path):
    with open(file_path, 'r') as f:
        content = f.read().strip()  # Read and remove leading/trailing whitespace

    data_list = ast.literal_eval(content)

    return data_list


def franka_main(base_path, camera_position, camera_orientation, K, direction = "left", full_res = False):

    if full_res == True:
        h,w = 720,1280
    else:
        h,w = 180,320

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    robot_id = p.loadURDF("/home/rrcadmin/cross-emb/Embodiment-Codes-RRC/URDF/franka_panda/panda_with_2F85_sec.urdf",[0, 0, 0], useFixedBase=True)

    end_effector_link_index = 7

    joint_file = os.path.join(base_path,"joint_ps.txt")
    
    joint_positions = get_joint_positions(joint_ps_file=joint_file)

    camera_orientation = p.getQuaternionFromEuler(camera_orientation)

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

    draw_camera_direction(camera_position, camera_orientation)

    masks_dir, img_dir = prepare_output_directories(base_path,"franka",direction,full_res)

    for idx, joint_pos in enumerate(joint_positions):
        move_to_joint_position_with_feedback(robot_id,joint_pos)  # Move to the joint positions directly
        image_name = os.path.join(masks_dir, f"camera_position_{idx}.png")
        image_name_rgb = os.path.join(img_dir, f"camera_position_{idx}.png")
        capture_image(camera_position, camera_orientation, image_name, image_name_rgb, K, h, w)

    p.disconnect()


    folder1 = masks_dir

    if full_res == True:
        folder2 = os.path.join(base_path, f"images_{direction}")
        overlay_path = os.path.join(base_path,"simulation", "franka", direction+"_overlay_images_hd")
        video_path = os.path.join(base_path, "simulation", "franka", direction,"overlay_hd.mp4")

    else:
        folder2 = os.path.join(base_path, f"images_{direction}_resized")
        overlay_path = os.path.join(base_path,"simulation", "franka", direction+"_overlay_images")
        video_path = os.path.join(base_path, "simulation", "franka", direction,"overlay.mp4")


    overlay_images_to_video(folder1,folder2,overlay_path,video_path)    

def xarm_main(base_path, camera_position, camera_orientation, K, direction = "left", full_res = False):

    if full_res == True:
        h,w = 720,1280
    else:
        h,w = 180,320

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)

    robot_id = p.loadURDF("/home/rrcadmin/cross-emb/Embodiment-Codes-RRC/URDF/src_xarm/airobot/urdfs/xarm7_robot.urdf",[0, 0, 0], useFixedBase=True)

    end_effector_link_index = 7

    cart_file = os.path.join(base_path,"cart_pos.txt")
    
    cart_positions = get_cartesians_positions(cart_ps_file=cart_file)

    camera_orientation = p.getQuaternionFromEuler(camera_orientation)

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

    draw_camera_direction(camera_position, camera_orientation)

    masks_dir, img_dir = prepare_output_directories(base_path,"xarm",direction,full_res)

    for idx, pos in enumerate(cart_positions):

        target_position = pos[:3]
        target_orientation = p.getQuaternionFromEuler(pos[3:])
        move_to_position_with_feedback(robot_id,end_effector_link_index,target_position, target_orientation)
        image_name = os.path.join(masks_dir, f"camera_position_{idx}.png")
        image_name_rgb = os.path.join(img_dir, f"camera_position_{idx}.png")
        capture_image(camera_position, camera_orientation, image_name, image_name_rgb, K, h, w)

    p.disconnect()


    folder1 = masks_dir

    if full_res == True:
        folder2 = os.path.join(base_path, f"images_{direction}")
        overlay_path = os.path.join(base_path,"simulation", "xarm", direction+"_overlay_images_hd")
        video_path = os.path.join(base_path, "simulation", "xarm", direction,"overlay_hd.mp4")

    else:
        folder2 = os.path.join(base_path, f"images_{direction}_resized")
        overlay_path = os.path.join(base_path,"simulation", "xarm", direction+"_overlay_images")
        video_path = os.path.join(base_path, "simulation", "xarm", direction,"overlay.mp4")


    overlay_images_to_video(folder1,folder2,overlay_path,video_path)


def generation_loop(base_path, camera_position, camera_orientation, K):

    full_res_list = [True,False]
    direction = 'left'

    for res in full_res_list:
        xarm_main(base_path=base_path,camera_position=camera_position,camera_orientation=camera_orientation,K=K,direction=direction,full_res=res)
        franka_main(base_path=base_path,camera_position=camera_position,camera_orientation=camera_orientation,K=K,direction=direction,full_res=res)



    

if __name__ == "__main__":

    base_path = "/scratch/darshil/cross-emb-data"
    # output_base = '/scratch/darshil/cross-emb-data'

    svo_list = [
            ('TRI/success/2023-10-24/Tue_Oct_24_15:05:46_2023','28451778.svo','TRI+52ca9b6a+2023-10-24-15h-05m-46s'),
            ('TRI/success/2023-10-24/Tue_Oct_24_18:23:27_2023','28451778.svo','TRI+52ca9b6a+2023-10-24-18h-23m-27s'),
            ('TRI/success/2023-10-25/Wed_Oct_25_10:21:18_2023','28451778.svo','TRI+52ca9b6a+2023-10-25-10h-21m-18s'),
            ('TRI/success/2023-10-25/Wed_Oct_25_17:06:19_2023','28451778.svo','TRI+52ca9b6a+2023-10-25-17h-06m-19s'),
            ('TRI/success/2023-10-12/Thu_Oct_12_12:17:09_2023','28451778.svo','TRI+30510ef3+2023-10-12-12h-17m-09s'),
            ('TRI/success/2023-11-27/Mon_Nov_27_16:30:40_2023','28451778.svo','TRI+52ca9b6a+2023-11-27-16h-30m-40s'),
            ('TRI/success/2023-11-27/Mon_Nov_27_16:33:29_2023','28451778.svo','TRI+52ca9b6a+2023-11-27-16h-33m-29s'),
            ('TRI/success/2023-11-27/Mon_Nov_27_17:27:03_2023','28451778.svo','TRI+52ca9b6a+2023-11-27-17h-27m-03s'),
            ('TRI/success/2023-11-27/Mon_Nov_27_17:29:06_2023','28451778.svo','TRI+52ca9b6a+2023-11-27-17h-29m-06s'),
            ('TRI/success/2024-01-08/Mon_Jan__8_13:59:49_2024','28451778.svo','TRI+52ca9b6a+2024-01-08-13h-59m-49s')
        ]

    svo_list = [
            ('TRI/success/2023-11-27/Mon_Nov_27_16:30:40_2023','28451778.svo','TRI+52ca9b6a+2023-11-27-16h-30m-40s'),
            ('TRI/success/2023-11-27/Mon_Nov_27_17:27:03_2023','28451778.svo','TRI+52ca9b6a+2023-11-27-17h-27m-03s'),

    ]


    for relative_path,svo_file,metadata_path in svo_list:
        
        rel_base = os.path.basename(relative_path)

        data_dir = os.path.join(base_path,rel_base+svo_file[:-4])
        
        direction = "left"
        K = get_camera_intrinsics(os.path.join(data_dir,"camera_params.txt"))
        camera_position, camera_orientation = get_camera_pose(os.path.join(data_dir,"extrinsics_left.txt"))

        generation_loop(data_dir,camera_position,camera_orientation,K)


