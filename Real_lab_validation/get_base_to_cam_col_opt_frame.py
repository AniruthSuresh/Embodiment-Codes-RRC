import numpy as np
from scipy.spatial.transform import Rotation as R

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a rotation matrix.
    """
    return R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()

def combine_transformations(translation_a_to_b, rpy_a_to_b, translation_b_to_c, rpy_b_to_c):
    """
    Combine transformations from A to B and B to C to get transformation from A to C.
    """
    R_a_to_b = euler_to_rotation_matrix(*rpy_a_to_b)
    R_b_to_c = euler_to_rotation_matrix(*rpy_b_to_c)

    R_a_to_c = R_a_to_b @ R_b_to_c  
    translation_a_to_c = translation_a_to_b + R_a_to_b @ translation_b_to_c

    rpy_a_to_c = R.from_matrix(R_a_to_c).as_euler('xyz')
    return translation_a_to_c, rpy_a_to_c


def invert_transformation(translation, rpy):
    """
    Compute the inverse of a transformation.
    """
    R_mat = euler_to_rotation_matrix(*rpy)
    R_mat_inv = R_mat.T
    
    translation_inv = -R_mat_inv @ np.array(translation)
    
    rpy_inv = R.from_matrix(R_mat_inv).as_euler('xyz')
    
    return translation_inv, rpy_inv

def quat_to_euler(quat):
    rotation = R.from_quat(quat)
    euler = rotation.as_euler('xyz', degrees=False)
    return euler


# <launch>
#   <!-- The rpy in the comment uses the extrinsic XYZ convention, which is the same as is used in a URDF. See
#        http://wiki.ros.org/geometry2/RotationMethods and https://en.wikipedia.org/wiki/Euler_angles for more info. -->
#   <!-- xyz="1.15097 0.395431 0.19678" rpy="3.12081 3.07245 0.400267" -->
#   <node pkg="tf2_ros" type="static_transform_publisher" name="camera_link_broadcaster"
#       args="1.15097 0.395431 0.19678   0.0359361 0.00330512 0.979473 -0.198319 link_base camera_link" />
# </launch>

"""
From this launch file , use the quat and not the RPY !!
"""

translation_a_to_b = [1.15097, 0.395431 ,0.19678]  
quat =  (0.0359361 ,0.00330512, 0.979473,-0.198319)

euler_angles = quat_to_euler(quat)
print("Euler Angles (rad):", euler_angles)

"""
U can double check that the euler angles computed using the quat_to_euler function is "not same" as the one in the launch file . 
So pls use the quat from the launch file and not the RPY angles. !
"""


rpy_a_to_b = euler_angles

# from camera_link to camera_optical_colour_frame -> found using the rosrun query 
# account for the internal camera rotation 
translation_b_to_c = [0 , -0.059 , 0]  
rpy_b_to_c = [-1.567 , -0.002 , -1.570]  # pi/2 rot clear here !!

# so , now we get camera base => camera_optical_colour_frame
translation_a_to_c, rpy_a_to_c = combine_transformations(
    translation_a_to_b, rpy_a_to_b, translation_b_to_c, rpy_b_to_c)


print(f"\n\nFinal Transformation from Base to Camera Optical Colour Frame:")
print(translation_a_to_c , rpy_a_to_c)

# this output should ideally match with the extrinsic values present in the dataset for this specific scene . 



