"""
This file considers xArm7 and cartesian positions (does IK and then moves the arm accordingly)
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


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)


robot_id = p.loadURDF("../URDF/src_xarm/airobot/urdfs/xarm7_robot.urdf")
end_effector_link_index = 7
positions = []
    


with open("../data/scene_4/cart_pos.txt", "r") as file:
    for line in file:
        positions.append(eval(line.strip()))



def move_to_position_with_feedback(target_position, target_orientation):

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




    for _ in range(1000): 
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

    h = 180
    w = 320

    old_dims = (720 , 1280)
    new_dims = (180 , 320)


    """
    NOTE : These are col - no 
    """

    """
    RPL setup 
    """
    # K_old = np.array([
    #     [522.06170654, 0, 661.82672119],
    #     [0, 522.06170654, 355.39929199],
    #     [0, 0, 1]
    # ])

    """
    scene - 12 
    """
#     K_old = np.array([
#     [522.845, 0, 648.825],
#     [0, 522.845, 354.744],
#     [0, 0, 1]
# ])



    """
    All AutoLab setup 
    """
    # K_old = np.array([[524.12890625 , 0 , 639.77941895] , 
    # [0,524.12890625 , 370.27819824] ,
    # [0,0,1]] )
    K_old = np.array([[524.24609375,   0.        , 639.77758789],
       [  0.        , 524.24609375, 370.27789307],
       [  0.        ,   0.        ,   1.        ]])


    K = update_intrinsic_matrix(K = K_old , old_dims = old_dims , new_dims = new_dims)

    # print(K)


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




def capture_image(camera_position, camera_orientation, file_name):

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


    height = 180
    width = 320


    proj_matrix = cvK2BulletP()


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



filtered_image_dir = "filtered_arm_pics_fin_rlds"

if os.path.exists(filtered_image_dir):
    shutil.rmtree(filtered_image_dir)

os.makedirs(filtered_image_dir)



"""
scene - 4 - GOOD !
"""
# left

camera_position = [0.085036	,0.563473	,0.416859]
camera_orientation = p.getQuaternionFromEuler([-1.95721,	-0.0233935	,-2.11812])



#right
# camera_position = [0.221758	,-0.31568,	0.405043]
# camera_orientation = p.getQuaternionFromEuler([-2.04106,	0.0181328,	-0.47797])



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
    camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 0.1])
    p.addUserDebugLine(camera_position, camera_target_position, lineColorRGB=[1, 0, 0], lineWidth=2)

draw_camera_direction(camera_position, camera_orientation)



for idx, pos in enumerate(positions):
    target_position = pos[:3]
    target_orientation = p.getQuaternionFromEuler(pos[3:])

    move_to_position_with_feedback(target_position, target_orientation)
    # Capture and save an image of the current camera position
    image_name = os.path.join(filtered_image_dir, f"camera_position_{idx}.png")
    capture_image(camera_position, camera_orientation, image_name)



p.disconnect()


