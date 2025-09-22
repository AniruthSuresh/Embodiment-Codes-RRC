import pybullet as p
import pybullet_data
import time

# -------------------- Connect --------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# -------------------- Load Robot --------------------
robot_id = p.loadURDF(
    "./lite-6-updated-urdf/lite_6_new.urdf",
    [0, 0, 0],
    useFixedBase=True
)

num_joints = p.getNumJoints(robot_id)
print(f"Total number of joints: {num_joints}\n")

# -------------------- Inspect joints --------------------
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}:")
    print(f"  Name: {joint_info[1].decode('utf-8')}")
    print(f"  Type: {joint_info[2]}")   # Revolute = 0, Prismatic = 1, Fixed = 4, etc.
    print(f"  Link name: {joint_info[12].decode('utf-8')}")
    print("------")

# -------------------- Add Sliders --------------------
joint_sliders = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_type = joint_info[2]

    if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
        joint_limit_lower = joint_info[8]
        joint_limit_upper = joint_info[9]

        # if limits are invalid, set a safe default
        if joint_limit_lower < -3.14 or joint_limit_upper > 3.14:
            joint_limit_lower, joint_limit_upper = -3.14, 3.14

        slider = p.addUserDebugParameter(joint_name, joint_limit_lower, joint_limit_upper, 0.0)
        joint_sliders.append((i, slider))

# -------------------- Simulation loop --------------------
while True:
    for joint_idx, slider in joint_sliders:
        target_pos = p.readUserDebugParameter(slider)
        p.setJointMotorControl2(
            bodyIndex=robot_id,
            jointIndex=joint_idx,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_pos,
            force=500
        )
    p.stepSimulation()
    time.sleep(1.0 / 240.0)
