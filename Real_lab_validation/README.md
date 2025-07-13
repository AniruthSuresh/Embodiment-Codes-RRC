## Pipeline Instructions

### Step 1: Extract Camera Info and Validate Topics

Begin by running `ros_convert.py`. This script first checks that all required topics are available in the ROS bag, specifically `/camera/color/image_raw`, `/camera/color/camera_info`, and `/xarm/joint_states`. Once validated, it extracts the intrinsic camera parameters from the `/camera/color/camera_info` topic and saves them into a `camera_info.csv` file. This includes the projection matrix `K` (later used in step 4)
### Step 2: Synchronize Image and Joint State Timestamps

Since the camera and the robot publish messages at different rates, the image timestamps and joint state timestamps do not naturally align. Typically, the ROS bag contains more frequent joint state updates than image frames. To synchronize them, run `time_synchronize.py`. This script matches each image timestamp with the closest joint state timestamp, ensuring a one-to-one mapping. As output, it produces `cleaned_js.csv`, a file that contains the joint angles aligned exactly to the saved image frames. The corresponding images are written to the `cleaned_image_final_check/` directory. 

### Step 3: Compute Camera Extrinsics

To compute the final extrinsic parameters of the camera, we need the transformation from the robot base to the `camera_color_optical_frame`. This is given by:

```bash
T_base_to_camera_optical = T_base_to_camera_link × T_camera_link_to_camera_optical
```

and in here : 

- T_base_to_camera_link comes from your robot’s launch file or URDF setup.

- T_camera_link_to_camera_optical is a fixed internal transform defined by the camera, which can be obtained by running:

``` bash
rosrun tf tf_echo camera_link camera_color_optical_frame
```

Combining these gives you the full extrinsic transformation . (this should ideally match with the extrinsics present in the dataset for this specific scene.)

### Step 4: Generate Overlays

To generate accurate overlays of the robot onto the real image frames, ensure the following components are correctly set:

- **Camera Intrinsics (`K` matrix)**:  
  Obtained from `camera_info.csv` generated in **Step 1**. 

- **Joint Angles**:  
  Use `cleaned_js.csv` from **Step 2**, which contains joint positions synchronized with each saved image.

- **Camera Pose (Extrinsics)**:  
  Compute this using the method described in **Step 3**. The full transform from the robot base to the `camera_color_optical_frame` must be used.

Once these three components are set, you should observe a clean overlay of the images with the mask . 