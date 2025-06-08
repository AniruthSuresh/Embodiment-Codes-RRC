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


p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)

robot_id = p.loadURDF("/home/aniruth/Embodiment-Codes-RRC/URDF/franka_panda/panda_with_2F85_sec.urdf",[0, 0, 0], useFixedBase=True)

end_effector_link_index = 7
positions = []



joint_positions = []

# when the .txt files are not properly formatted
# with open("./RLDS_Data/scene_4/joint_ps.txt", "r") as file:
#     for line in file:
#         try:
#             # Split line into float numbers
#             joint = list(map(float, line.strip().split()))
#             joint_positions.append(joint)
#         except ValueError as e:
#             print(f"Value error in line: {line.strip()}")
#             print(e)


# when the .txt files are properly formatted
with open("../data/scene_4/joint_ps.txt", "r") as file:
    for line in file:
        try:
            joint_positions.append(eval(line.strip()))  # Read joint positions as a list
        except SyntaxError as e:
            print(f"Syntax error in line: {line.strip()}")
            print(e)


# def move_to_position_with_feedback(target_position, target_orientation):

#     ik_joint_positions = p.calculateInverseKinematics(
#         robot_id, 
#         end_effector_link_index, 
#         target_position, 
#         target_orientation)
    

#     for i in range(len(ik_joint_positions)):
#         p.setJointMotorControl2(
#             bodyUniqueId=robot_id,
#             jointIndex=i,
#             controlMode=p.POSITION_CONTROL,
#             targetPosition=ik_joint_positions[i]
#         )


#     joint_angles = p.calculateInverseKinematics(robot_id, end_effector_link_index, target_position, targetOrientation=target_orientation , 
#                                              maxNumIterations=1000 )

#     for i in range(len(joint_angles)):
#         p.resetJointState(robot_id , i,joint_angles[i])


    # joint_positions = p.calculateInverseKinematics(
    #     robot_id,
    #     end_effector_link_index,
    #     target_position,
    #     target_orientation
    # )

    # # Ignore physics by directly resetting joint states
    # for i in range(len(joint_positions)):
    #     p.resetJointState(robot_id, i, joint_positions[i])


    # for _ in range(500): 
    #     p.stepSimulation()
    


def move_to_joint_position_with_feedback(joint_positions):
    # Use the joint positions directly and move the robot arm accordingly.
    for i in range(len(joint_positions)):
        p.resetJointState(robot_id, i, joint_positions[i])

    for _ in range(500):  # Step through the simulation to allow motion.
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
    K_old = np.array([
        [522.845, 0, 648.825],
        [0, 522.845, 354.744],
        [0, 0, 1]
    ])



    """
    All AutoLab setup 
    """
    # K_old = np.array([[524.12890625 , 0 , 639.77941895] , 
    # [0,524.12890625 , 370.27819824] ,
    # [0,0,1]] )


    # K_old = np.array([[530.3782959 ,   0.        , 646.08825684],
    #    [  0.        , 530.3782959 , 368.5713501 ],
    #    [  0.        ,   0.        ,   1.        ]])
    
    K = update_intrinsic_matrix(K = K_old , old_dims = old_dims , new_dims = new_dims)

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
scene - 2
"""
# the below one is for camera - 29431508 left 
# camera_position = [0.438195 ,0.504532 ,0.295449,]
# camera_orientation = p.getQuaternionFromEuler([-1.77092, -0.0652779, -2.76283])

# the below one is for camera - 29431508 right 
# camera_position = [0.326989,	0.459881	,0.303022]
# camera_orientation = p.getQuaternionFromEuler([-1.77039	,-0.0659621	,-2.76027])

"""
scene - 3
	
"""
# camera_position = [0.358499	,0.436766,	0.228544]
# camera_orientation = p.getQuaternionFromEuler([-1.85161	,0.0533807	,-3.01635])


"""
scene - 4 - GOOD !
"""
# # left
camera_position = [0.085036	,0.563473	,0.416859]
camera_orientation = p.getQuaternionFromEuler([-1.95721,	-0.0233935	,-2.11812])


#right
# camera_position = [0.221758	,-0.31568,	0.405043]
# camera_orientation = p.getQuaternionFromEuler([-2.04106,	0.0181328,	-0.47797])


"""
scene - 5
"""
# left 
# camera_position = [0.190077	,0.678763,	0.397614]
# camera_orientation = p.getQuaternionFromEuler([-2.00402,	0.0932804	,-2.29782])
	
	
# right 
# camera_position = [0.107339,	0.590132,	0.383264]
# camera_orientation = p.getQuaternionFromEuler([-2.00627,	0.0890864	,-2.28867])
	

"""
scene - 6
"""   

# 0.0315108	0.368786	0.436813	-2.12647	-0.138592	-1.66187
# camera_position = [0.0315108	,0.368786,	0.436813]
# camera_orientation = p.getQuaternionFromEuler([	-2.12647,-0.138592	,-1.66187])

"""
scene - 7
"""
# left cam -> good !
# camera_position = [0.167891,	0.447045,	0.488312]
# camera_orientation = p.getQuaternionFromEuler([-1.75215	,-0.0124033,-2.05865])


# right cam 0.236917	-0.298693	0.514985	-1.81304	0.00407764	-0.9203
# camera_position = [0.236917,	-0.298693,	0.514985]
# camera_orientation = p.getQuaternionFromEuler([-1.81304	,0.00407764,-0.9203])


"""
scene - 8 
	
"""
# camera_position = [0.421479	,0.689858,	0.510168]
# camera_orientation = p.getQuaternionFromEuler([-2.15771	,-0.150342	,-2.98978])

"""
scene -9 
"""

# camera_position = [0.225655	,0.60718	,0.286266]
# camera_orientation = p.getQuaternionFromEuler([-1.98394	,-0.0132214	,-2.42917])

"""
scene - 10 	
"""
# camera_position = [0.225655	,0.60718	,0.286266]
# camera_orientation = p.getQuaternionFromEuler([-1.98394,	-0.0132214	,-2.42917])

"""
scene - 11
	
"""
# camera_position = [0.421479	,0.689858,	0.510168]
# camera_orientation = p.getQuaternionFromEuler([-2.15771,	-0.150342,	-2.98978])

"""
Scene -12 : RPL 
	
"""
# camera_position = [0.0282164	,0.456743,	0.321209]
# camera_orientation = p.getQuaternionFromEuler([-1.37115	,0.00832112	,-1.82577 ])


"""
Scene -18 : RPL 
	
"""
# camera_position = [0.403975	,0.473188	,0.271706506]
# camera_orientation = p.getQuaternionFromEuler([-1.68271,	0.0755023,	-2.66829])

	


"""
Scene -19
"""
# camera_position = [0.421555,	0.428342,	0.340607]
# camera_orientation = p.getQuaternionFromEuler([-2.0116	,-0.102344	,-2.309])



	

"""
Scene -20
"""
# camera_position = [0.459738,	0.500735,	0.41065]
# camera_orientation = p.getQuaternionFromEuler([-1.8838,	0.0567629,	-2.71197])

"""
Scene -21
"""
# camera_position = [0.0957822	,0.523629,	0.502617]
# camera_orientation = p.getQuaternionFromEuler([-1.8843,	-0.17777	,-1.65197])



	
"""
Scene -21
"""
# camera_position = [0.624621,	-0.318757,	0.423166]
# camera_orientation = p.getQuaternionFromEuler([-2.4857,	0.0435594	,0.185743])




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

# def draw_camera_direction(camera_position, camera_orientation):
#     rot_matrix = R.from_quat(camera_orientation).as_matrix()
#     camera_target_position = camera_position + rot_matrix @ np.array([0, 0, 0.1])
#     p.addUserDebugLine(camera_position, camera_target_position, lineColorRGB=[1, 0, 0], lineWidth=2)

# draw_camera_direction(camera_position, camera_orientation)


# for idx, pos in enumerate(positions):
#     target_position = pos[:3]
#     target_orientation = p.getQuaternionFromEuler(pos[3:])

#     move_to_position_with_feedback(target_position, target_orientation)

#     image_name = os.path.join(filtered_image_dir, f"camera_position_{idx}.png")
#     capture_image(camera_position, camera_orientation, image_name)

for idx, joint_pos in enumerate(joint_positions):
    move_to_joint_position_with_feedback(joint_pos)  # Move to the joint positions directly

    # Capture and save an image of the current camera position
    image_name = os.path.join(filtered_image_dir, f"camera_position_{idx}.png")
    capture_image(camera_position, camera_orientation, image_name)


p.disconnect()

