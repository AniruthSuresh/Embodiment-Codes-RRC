## RRC - Embodiment Codes

### 0. Setup

Two conda environments are used:

- `sim_cam`: for all general motion runs. Use `/sim_cam.yml` to set it up and activate it before running any robot motion code.
- `svo_reader`: for reading `.svo` files. Use `/Intrinsics_and_Image_extraction/svo_reader_env.yml` and activate only when extracting intrinsics or raw images.

### 1. Data Extraction

We need the following:
- Joint positions (for Franka Panda)
- Cartesian positions (for xArm7)
- Left and right camera images
- Camera intrinsics

Refer to `/Data Extraction/RLDS_data_extraction.ipynb` for how data is pulled from RLDS. This includes joint positions, Cartesian positions, and images (lower resolution). Extrinsics can be found in the `trajectory.h5` file downloaded in the notebook.

Accessing raw data is still mostly manual — ideally, explore each scene and extract what's needed.

For intrinsics, they are embedded in the `.svo` file, so make sure to extract from it.

### 2. Intrinsics and RAW Image Extraction

RLDS provides left/right images directly, but I *think* it's possible to extract higher-resolution versions from the `.svo` file as well. However, I'm not very confident about this , it's been a while since I worked on it, so please double-check this part.

Also, note that I'm currently resizing the images to a lower resolution during extraction, but I'm not entirely sure if higher-resolution data is actually available inside the `.svo`. 


To extract intrinsics and images, run `/Intrinsics_and_Image_extraction/main.py` with the path to the corresponding `.svo` file set.

This setup may be tricky , I had to downgrade my CUDA to match the version supported by the dependencies. Check `/Intrinsics_and_Image_extraction/svo_reader_env.yml` for exact package requirements.

### 3. Robot Arm Motion

After cloning the repo, make sure to pull submodules.

For xArm7: use `/Robot_arm_motion/xarm_motion.py`  
For Franka: use `/Robot_arm_motion/franka_motion.py`

You’ll need to update:
- The Cartesian position in the file (contains end-effector positions)
- The projection matrix in `cvK2BulletP()` (from `/Intrinsics_and_Image_extraction/main.py`)
- Camera position and orientation (from the `trajectory.h5` file — this acts as extrinsics)


**NOTE:** I've attached one `data` folder corresponding to a specific RLDS scene (SCEM-4 to be precise) and have already run the motion for it. You can check this out, replicate, and improve.
