import os
from camera_utils import SVOReader
import cv2


svo_files = ["../Intrinsics_and_Image_extraction/data/22246076.svo",]
# NOTE : The aspect ratio is the same 
# (780 ,1280 ) -> (180 , 320)

target_height = 180
target_width = 320

for svo_file in svo_files:
    serial_number = svo_file.split("/")[-1][:-4]
    print(f"Processing SVO file: {serial_number}")

    # Create folders for left and right images
    left_folder = os.path.join(os.getcwd(), "images_left")
    right_folder = os.path.join(os.getcwd(), "images_right")
    os.makedirs(left_folder, exist_ok=True)
    os.makedirs(right_folder, exist_ok=True)

    reader = SVOReader(svo_file, serial_number)

    reader.set_reading_parameters(image=True, concatenate_images=False)

    camera_intrinsics = reader.get_camera_intrinsics()
    camera_baseline = reader.get_camera_baseline()

    print(f"Camera Intrinsics (Left): {camera_intrinsics[serial_number + '_left']}")
    print(f"Camera Intrinsics (Right): {camera_intrinsics[serial_number + '_right']}")
    print(f"Camera Baseline: {camera_baseline} meters")

    frame_count = reader.get_frame_count()
    print(f"Total frames in the SVO file: {frame_count}")

    for frame_index in range(frame_count):
        reader.set_frame_index(frame_index)

        data = reader.read_camera(ignore_data=False)
        left_image = data["image"].get(serial_number + "_left")
        right_image = data["image"].get(serial_number + "_right")

        if left_image is not None and right_image is not None:
            # Downscale the images
            left_image_resized = cv2.resize(left_image, (target_width, target_height), interpolation=cv2.INTER_AREA)
            right_image_resized = cv2.resize(right_image, (target_width, target_height), interpolation=cv2.INTER_AREA)

            # Save the downscaled images
            left_image_path = os.path.join(left_folder, f"{serial_number}_left_{frame_index:04d}.png")
            right_image_path = os.path.join(right_folder, f"{serial_number}_right_{frame_index:04d}.png")
            cv2.imwrite(left_image_path, left_image_resized)
            cv2.imwrite(right_image_path, right_image_resized)

    print("Finished extracting, downscaling, and saving images.")
