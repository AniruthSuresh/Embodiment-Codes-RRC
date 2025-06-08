from yacs.config import CfgNode as CN

_C = CN()

# prefix of the class name of the ARM
# if it's for pybullet simulation, the name will
# be augemented to be '<Prefix>Pybullet'
# if it's for the real robot, the name will be
# augmented to be '<Prefix>Real'
_C.CLASS = 'XARM7'
_C.MOVEGROUP_NAME = 'manipulator'
_C.ROSTOPIC_JOINT_STATES = '/joint_states'

# https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/max-joint-torques-17260/
_C.MAX_TORQUES = [50, 50, 30, 30, 30, 20, 20]
_C.JOINT_NAMES = [
    'joint1', 'joint2', 'joint3', 'joint4',
    'joint5', 'joint6', 'joint7'
]
# base frame for the arm
_C.ROBOT_BASE_FRAME = 'link_base'
# end-effector frame of the arm
_C.ROBOT_EE_FRAME = 'link7'
_C.ROBOT_EE_FRAME_JOINT = 'joint7'
# _C.JOINT_SPEED_TOPIC = '/joint_speed'
# _C.URSCRIPT_TOPIC = '/ur_driver/URScript'
# inverse kinematics position tolerance (m)
_C.IK_POSITION_TOLERANCE = 0.01
# inverse kinematics orientation tolerance (rad)
_C.IK_ORIENTATION_TOLERANCE = 0.05
_C.HOME_POSITION = [0, 0, 0, 0, 0, 0, 0]
#[-0.19, 0.08, 0.23, 0.43, 0.03, 0.52, 0.86]
#[-0.19, 0.08, 0.23, -2.43, 0.03, 2.52, 0.86]
_C.MAX_JOINT_ERROR = 0.01
_C.MAX_JOINT_VEL_ERROR = 0.05
_C.MAX_EE_POS_ERROR = 0.01
# real part of the quaternion difference should be
# greater than 1-error
_C.MAX_EE_ORI_ERROR = 0.02
_C.TIMEOUT_LIMIT = 10

# reset position for the robot in pybullet
_C.PYBULLET_RESET_POS = [0, 0, 1]
# reset orientation (euler angles) for the robot in pybullet
_C.PYBULLET_RESET_ORI = [0, 0, 0]
_C.PYBULLET_IK_DAMPING = 0.0005


def get_xarm7_arm_cfg():
    return _C.clone()
