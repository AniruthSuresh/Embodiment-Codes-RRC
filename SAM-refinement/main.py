import os
import cv2
import shutil
import numpy as np
import torch
import re
from segment_anything import SamPredictor, sam_model_registry
from tqdm import tqdm 

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")


"""
U need to get the SAM model checkpoint file and place it in the specified path.
"""
sam_checkpoint = "./segment-anything/sam_vit_h_4b8939.pth"
model_type = "vit_h"  # SAM model type

sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
sam.to(device)
predictor = SamPredictor(sam)

rgb_folder = "../../Complete_Data/xArm/RGB_Now/"
mask_folder = "../../Complete_Data/xArm/Mask_now/"

output_folder = "../../Complete_Data/xArm/Mask_SAM/"
os.makedirs(output_folder, exist_ok=True)


num_space = 400  # Spacing for selecting points

for filename in tqdm(os.listdir(rgb_folder), desc="Processing files", unit="file"):

        if filename.endswith(".png"):

            rgb_image_path = os.path.join(rgb_folder, filename)
            mask_image_path = os.path.join(mask_folder, filename)

            print(rgb_image_path , mask_image_path)
            rgb_image = cv2.imread(rgb_image_path)
            mask_image = cv2.imread(mask_image_path, cv2.IMREAD_GRAYSCALE)

            if rgb_image is None or mask_image is None:
                print(f"Error loading {filename}. Skipping...")
                continue

            _, binary_mask = cv2.threshold(mask_image, 50, 255, cv2.THRESH_BINARY)

            skeleton = cv2.ximgproc.thinning(binary_mask)
            bold_skeleton = cv2.dilate(skeleton, np.ones((3, 3), np.uint8), iterations=3)

            skeleton_points = np.column_stack(np.where(bold_skeleton > 0))

            input_points = np.array([[point[1], point[0]] for point in skeleton_points])  # (x, y) format
            input_labels = np.ones(input_points.shape[0])  # Foreground labels

            for point in input_points[::num_space]:
                cv2.circle(rgb_image, (point[0], point[1]), radius=3, color=(0, 0, 255), thickness=1)

            predictor.set_image(rgb_image)

            masks, _, _ = predictor.predict(
                point_coords=input_points[::num_space],
                point_labels=input_labels[::num_space],
                multimask_output=False,  
            )

            segmented_mask = masks[0]
            segmented_mask_uint8 = (segmented_mask * 255).astype(np.uint8) if segmented_mask.max() == 1 else segmented_mask.astype(np.uint8)

            kernel = np.ones((5, 5), np.uint8)  
            closed_mask = cv2.morphologyEx(segmented_mask_uint8, cv2.MORPH_CLOSE, kernel)

            output_path = os.path.join(output_folder, filename)
            cv2.imwrite(output_path, closed_mask)
            print(f"Processed and saved mask for {filename}.")

print("Processing complete. All predicted masks are saved.")

def extract_number(filename):
    """
    Extracts the first number found in the filename
    """
    numbers = re.findall(r'\d+', filename)
    return int(numbers[0]) if numbers else float('inf')

def overlay_images(folder1, folder2, output_folder, alpha=0.5):

    if os.path.exists(output_folder):
        shutil.rmtree(output_folder)  
    os.makedirs(output_folder)

    image_filenames1 = sorted(os.listdir(folder1), key=extract_number)
    image_filenames2 = sorted(os.listdir(folder2), key=extract_number)

    if len(image_filenames1) != len(image_filenames2):
        print("Warning: The number of images in both folders does not match.")
        return

    for filename1, filename2 in zip(image_filenames1, image_filenames2):
        if filename1.endswith(".png") and filename2.endswith(".png"):
            print(f"Overlaying {filename1} and {filename2}")

            image1 = cv2.imread(os.path.join(folder1, filename1))
            image2 = cv2.imread(os.path.join(folder2, filename2))

            if image1 is None or image2 is None:
                print(f"Error loading images {filename1} or {filename2}. Skipping...")
                continue

            if image1.shape != image2.shape:
                image2 = cv2.resize(image2, (image1.shape[1], image1.shape[0]))

            overlay = cv2.addWeighted(image1, alpha, image2, 1 - alpha, 0)

            output_path = os.path.join(output_folder, filename1)
            cv2.imwrite(output_path, overlay)
            print(f"Saved overlaid image for {filename1}.")

    print("Overlaying complete. All overlaid images are saved.")

def frames_to_video(frames_folder, output_video, fps):
    frame_files = [f for f in os.listdir(frames_folder) if f.endswith('.png')]
    frame_files.sort(key=lambda x: int(re.search(r'(\d+)', x).group()))

    if not frame_files:
        print("No frame files found in the folder.")
        return
    
    first_frame_path = os.path.join(frames_folder, frame_files[0])
    frame = cv2.imread(first_frame_path)
    height, width, layers = frame.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    # Write each frame to the video
    for frame_file in frame_files:
        frame_path = os.path.join(frames_folder, frame_file)
        img = cv2.imread(frame_path)
        video.write(img)

    video.release()
    print(f"Video saved as {output_video}")


overlay_folder1 = "../Calibration - Lab/cleaned_image_rp_3/"
overlay_folder2 = "../../Complete_Data/xArm/Predicted_Masks/"
overlay_output_folder = "../../Complete_Data/xArm/Overlayed_Images/"

overlay_images(overlay_folder1, overlay_folder2, overlay_output_folder, alpha=0.5)


frames_to_video(overlay_output_folder , output_video = 'sam_model_scene_3.mp4' , fps= 50)

