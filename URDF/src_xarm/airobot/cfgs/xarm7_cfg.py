from airobot.cfgs.assets.default_configs import get_cfg_defaults
from airobot.cfgs.assets.pybullet_camera import get_sim_cam_cfg
from airobot.cfgs.assets.realsense_camera import get_realsense_cam_cfg
from airobot.cfgs.assets.xarm7_gripper import get_xarm7gripper_cfg
from airobot.cfgs.assets.xarm7_arm import get_xarm7_arm_cfg

_C = get_cfg_defaults()
# whether the robot has an arm or not
_C.HAS_ARM = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a end effector tool or not
_C.HAS_EETOOL = True

_C.ROBOT_DESCRIPTION = '/robot_description'
_C.PYBULLET_URDF = 'xarm7_robot.urdf'

_C.ARM = get_xarm7_arm_cfg()

_C.CAM.SIM = get_sim_cam_cfg()
_C.CAM.REAL = get_realsense_cam_cfg()
_C.CAM.CLASS = 'RGBDCamera'

_C.EETOOL = get_xarm7gripper_cfg()
_C.EETOOL.CLASS = 'XARM7Gripper'


def get_cfg():
    return _C.clone()
