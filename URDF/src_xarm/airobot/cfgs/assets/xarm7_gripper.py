from yacs.config import CfgNode as CN

_C = CN()
# joint angle when the gripper is fully open
_C.OPEN_ANGLE = 0.0
# joint angle when the gripper is fully closed
_C.CLOSE_ANGLE = 0.85

# time in seconds to wait for new gripper state before exiting
_C.UPDATE_TIMEOUT = 5.0

# default maximum values for gripper state varibles
# minimum values are all 0
_C.POSITION_RANGE = 255
# scaling factor to convert from URScript range to Robotiq range
_C.POSITION_SCALING = (255 / 0.7)

# for the gripper in pybullet
_C.JOINT_NAMES = [
    'left_finger_joint', 'left_outer_knuckle_joint',
    'left_inner_knuckle_joint', 'right_finger_joint', 'right_outer_knuckle_joint',
    'right_inner_knuckle_joint',
]
_C.MIMIC_COEFF = [1, 1, 1, 1, 1, 1]
_C.MAX_TORQUE = 25.0


def get_xarm7gripper_cfg():
    return _C.clone()