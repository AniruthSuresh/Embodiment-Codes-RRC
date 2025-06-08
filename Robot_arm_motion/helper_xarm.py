import cv2
import os
import re


def overlay_images_to_video(folder1, folder2, output_folder='overlayed_images', video_output='overlayed_video.mp4', frame_rate=30):
    """
    Overlay images from two folders, save the result, and create a video from the overlayed images.
    """

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    images1 = [f for f in os.listdir(folder1) if f.endswith('.png')]
    images2 = [f for f in os.listdir(folder2) if f.endswith('.png')]

    images1.sort(key=lambda x: int(re.search(r'(\d+)', x).group()))
    images2.sort(key=lambda x: int(re.search(r'(\d+)', x).group()))

    num_images = min(len(images1), len(images2))

    # print(images1,images2)

    num_images = min(len(images1), len(images2))

    if num_images == 0:
        print("Error: No images to overlay.")
        return

    img1 = cv2.imread(os.path.join(folder1, images1[0]))
    if img1 is None:
        print("Error: Could not read the first image in the folder.")
        return
    height, width, _ = img1.shape

    fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    video_writer = cv2.VideoWriter(video_output, fourcc, frame_rate, (width, height))

    for i in range(num_images):
        img1_path = os.path.join(folder1, images1[i])
        img2_path = os.path.join(folder2, images2[i])

        img1 = cv2.imread(img1_path)
        img2 = cv2.imread(img2_path)

        if img1 is None or img2 is None:
            print(f"Error: Could not read images {images1[i]} or {images2[i]}")
            continue


        overlay = cv2.addWeighted(img1, 0.5, img2, 0.5, 0)

        output_img_name = f"overlayed_{images1[i]}"
        output_img_path = os.path.join(output_folder, output_img_name)
        cv2.imwrite(output_img_path, overlay)

        video_writer.write(overlay)

    video_writer.release()

    print(f"Overlayed video saved as '{video_output}'.")
 

masked_pics = "../Robot_arm_motion/filtered_arm_pics_fin_rlds/"

original_pics_path = "../data/scene_4/images_left/"

overlay_images_to_video(original_pics_path , masked_pics , video_output = "scene_7_overlay.mp4")
