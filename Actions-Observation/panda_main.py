import os
import time
import pybullet as p
import pybullet_data
import numpy as np
import h5py

# ============================================================
# PyBullet Setup
# ============================================================
def setup_sim():
    """Initialize PyBullet and load the Franka Panda robot."""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(1.0 / 240.0)

    robot_id = p.loadURDF(
        "../../../Exact_Panda/bullet3/examples/pybullet/gym/pybullet_data/franka_panda/panda_with_2F85_sec.urdf",
        [0, 0, 0],
        useFixedBase=True
    )
    return robot_id

# ============================================================
# Data and Robot Helpers
# ============================================================
def set_home_pose(robot_id, controlled_joints, home_joint_angles):
    """Reset robot joints to a given home configuration."""
    for j, angle in zip(controlled_joints, home_joint_angles):
        p.resetJointState(robot_id, j, angle)

def load_actions_from_h5(filepath, dataset_path):
    """Load action data (joint positions) from an HDF5 file."""
    print(f"Loading actions from '{filepath}'...")
    try:
        with h5py.File(filepath, 'r') as f:
            if dataset_path in f:
                actions = f[dataset_path][:]
                print(f"Successfully loaded {len(actions)} actions from dataset '{dataset_path}'.")
                return actions
            else:
                print(f"Error: Dataset '{dataset_path}' not found in '{filepath}'.")
                return None
    except Exception as e:
        print(f"An error occurred while reading the HDF5 file: {e}")
        return None

def save_to_txt(filepath, data_log):
    """Save a list of lists to a text file."""
    print(f"Saving data to '{filepath}'...")
    with open(filepath, 'w') as f:
        for row in data_log:
            # Convert each item in the row to a string and join with spaces
            line = ' '.join(map(str, row))
            f.write(line + '\n')
    print(f"Successfully wrote {len(data_log)} lines.")

# ============================================================
# Main Simulation Replay
# ============================================================
def replay_and_record_trajectory(robot_id, controlled_joints, joint_positions_actions):
    """
    Replay a joint position trajectory and record the commanded actions
    and resulting observations (the actual joint positions).
    """
    control_freq = 15.0
    sim_freq = 240.0
    steps_per_action = int(sim_freq / control_freq)

    actions_log = []
    observations_log = []

    for step, target_positions in enumerate(joint_positions_actions):
        # 1. Log the commanded action (the target joint positions)
        actions_log.append(target_positions)

        # 2. Apply the position command to the robot's motors
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=controlled_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=target_positions,
            # These forces/velocities act as limits
            # forces=[87.0] * len(controlled_joints) 
        )

        # # 3. Step the simulation forward to allow the robot to move
        for _ in range(steps_per_action):
            p.stepSimulation()
            time.sleep(1.0 / sim_freq)

        # 4. Get and log the resulting observation (actual joint positions)
        joint_states = p.getJointStates(robot_id, controlled_joints)
        current_positions_observation = [state[0] for state in joint_states]
        observations_log.append(current_positions_observation)

        print(f"Step {step+1}/{len(joint_positions_actions)} executed.")

    return actions_log, observations_log

if __name__ == "__main__":
    robot_id = setup_sim()
    controlled_joints = [0, 1, 2, 3, 4, 5, 6]

    # Set an initial "home" pose for the robot
    home_joint_angles = [-0.222267,	-0.567925,	-0.0911155,-2.71721,	-0.195372,	2.06842	,0.0430781]
    set_home_pose(robot_id, controlled_joints, home_joint_angles)

    # --- Load Actions from H5 file ---
    h5_filepath = './trajectory.h5'

    action_dataset = 'action/joint_position' 
    actions_to_replay = load_actions_from_h5(h5_filepath, action_dataset)
    
    if actions_to_replay is None:
        p.disconnect()
        exit()

    # --- Replay the trajectory and record the data ---
    logged_actions, logged_observations = replay_and_record_trajectory(
        robot_id,
        controlled_joints,
        actions_to_replay
    )

    p.disconnect()

    # --- Save the recorded data to .txt files ---
    save_to_txt("actions.txt", logged_actions)
    save_to_txt("observations.txt", logged_observations)

    print("\nProcess finished. 'actions.txt' and 'observations.txt' have been created.")