import os
from camera_utils import SVOReader
import cv2

svo_files = ["Data-Extraction/29838012.svo"]
# NOTE : The aspect ratio is the same 
# (780 ,1280 ) -> (180 , 320)



def get_images_intrinsics(svo_directory):

    target_height = 180
    target_width = 320

    serial_number = svo_directory[-8:]
    print(f"Processing SVO file: {serial_number}")

    # Create folders for left and right images
    left_folder_resized = os.path.join(svo_directory,"images_left_resized")
    right_folder_resized = os.path.join(svo_directory, "images_right_resized")

    left_folder = os.path.join(svo_directory,"images_left")
    right_folder = os.path.join(svo_directory, "images_right")

    os.makedirs(left_folder, exist_ok=True)
    os.makedirs(right_folder, exist_ok=True)
    os.makedirs(left_folder_resized, exist_ok=True)
    os.makedirs(right_folder_resized, exist_ok=True)

    reader = SVOReader(os.path.join(svo_directory,serial_number+'.svo'), serial_number)

    reader.set_reading_parameters(image=True, concatenate_images=False)

    camera_intrinsics = reader.get_camera_intrinsics()
    camera_baseline = reader.get_camera_baseline()

    print(f"Camera Intrinsics (Left): {camera_intrinsics[serial_number + '_left']}")
    print(f"Camera Intrinsics (Right): {camera_intrinsics[serial_number + '_right']}")
    print(f"Camera Baseline: {camera_baseline} meters")

    camera_params_path = os.path.join(svo_directory, "camera_params.txt")
    with open(camera_params_path, 'w') as f:
        f.write("Camera Intrinsics (Left):\n")
        f.write(str(camera_intrinsics[serial_number + '_left']) + "\n\n")

        f.write("Camera Intrinsics (Right):\n")
        f.write(str(camera_intrinsics[serial_number + '_right']) + "\n\n")

        f.write("Camera Baseline (meters):\n")
        f.write(f"{camera_baseline:.6f}\n")

    frame_count = reader.get_frame_count()
    print(f"Total frames in the SVO file: {frame_count}")

    for frame_index in range(frame_count-1):
        reader.set_frame_index(frame_index)

        data = reader.read_camera(ignore_data=False)
        
        left_image = data["image"].get(serial_number + "_left")
        right_image = data["image"].get(serial_number + "_right")

        if left_image is not None and right_image is not None:
            # Downscale the images
            left_image_resized = cv2.resize(left_image, (target_width, target_height), interpolation=cv2.INTER_AREA)
            right_image_resized = cv2.resize(right_image, (target_width, target_height), interpolation=cv2.INTER_AREA)

            # Save the downscaled images
            left_image_path_res = os.path.join(left_folder_resized, f"{frame_index:04d}.png")
            right_image_path_res = os.path.join(right_folder_resized, f"{frame_index:04d}.png")

            left_image_path = os.path.join(left_folder, f"{frame_index:04d}.png")
            right_image_path = os.path.join(right_folder, f"{frame_index:04d}.png")

            cv2.imwrite(left_image_path_res, left_image_resized)
            cv2.imwrite(right_image_path_res, right_image_resized)

            cv2.imwrite(left_image_path, left_image)
            cv2.imwrite(right_image_path, right_image)

    print("Finished extracting, downscaling, and saving images.")

if __name__ == "__main__":
    paths = [
    "/scratch/darshil/cross-emb-data/Thu_Nov_30_16:30:12_202328834630",
    "/scratch/darshil/cross-emb-data/Wed_Jul_12_21:35:14_202323897859",
    "/scratch/darshil/cross-emb-data/Tue_Jun__6_11:52:15_202329838012",
    "/scratch/darshil/cross-emb-data/Tue_Oct_24_18:22:52_202328451778",
    "/scratch/darshil/cross-emb-data/Tue_Nov_28_14:05:05_202320252535",
    "/scratch/darshil/cross-emb-data/Wed_Jan_10_15:00:51_202428451778",
    "/scratch/darshil/cross-emb-data/Wed_Jan_10_15:12:19_202428451778",
    "/scratch/darshil/cross-emb-data/Wed_Jan_10_15:13:01_202428451778",
    "/scratch/darshil/cross-emb-data/Wed_Jan_10_15:20:58_202428451778",
    "/scratch/darshil/cross-emb-data/Wed_Jan_10_15:53:38_202428451778",
    "/scratch/darshil/cross-emb-data/Wed_Jan_10_15:54:03_202428451778",
    "/scratch/darshil/cross-emb-data/Tue_Nov_28_14:02:40_202320252535"
    ]

    for svo_path in paths:
        get_images_intrinsics(svo_path)

    # svo_directory = "Data-Extraction/Thu_May_11_14:08:19_202329838012"
    # get_images_intrinsics(svo_directory=svo_directory)