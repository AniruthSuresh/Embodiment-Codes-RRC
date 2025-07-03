import os
import shutil
import re

def natural_sort_key(s):
    return int(re.findall(r'\d+', s)[0])

def get_next_index(dst_dir):
    if not os.path.exists(dst_dir):
        return 0
    files = [f for f in os.listdir(dst_dir) if f.endswith(".png")]
    indices = [int(re.findall(r'\d+', f)[0]) for f in files if re.findall(r'\d+', f)]
    return max(indices) + 1 if indices else 0

def rename_and_copy_synchronized(
    a_img_dir, a_mask_dir,
    b_img_dir, b_mask_dir,
    dst_a_img_dir, dst_a_mask_dir,
    dst_b_img_dir, dst_b_mask_dir,
    start, end
):
    # Create output directories
    os.makedirs(dst_a_img_dir, exist_ok=True)
    os.makedirs(dst_a_mask_dir, exist_ok=True)
    os.makedirs(dst_b_img_dir, exist_ok=True)
    os.makedirs(dst_b_mask_dir, exist_ok=True)

    # Load all file lists
    a_imgs = sorted([f for f in os.listdir(a_img_dir) if f.endswith(".png")], key=natural_sort_key)
    a_masks = sorted([f for f in os.listdir(a_mask_dir) if f.endswith(".png")], key=natural_sort_key)
    b_imgs = sorted([f for f in os.listdir(b_img_dir) if f.endswith(".png")], key=natural_sort_key)
    b_masks = sorted([f for f in os.listdir(b_mask_dir) if f.endswith(".png")], key=natural_sort_key)

    # Compute usable range
    total = min(len(a_imgs), len(a_masks), len(b_imgs), len(b_masks))
    dst_index = get_next_index(dst_a_img_dir)

    for i in range(total):
        if i < start or i >= end:
            # Generate output name
            new_name = f"image_{dst_index:04d}.png"

            # Copy A (xArm sim)
            shutil.copy(os.path.join(a_img_dir, a_imgs[i]), os.path.join(dst_a_img_dir, new_name))
            shutil.copy(os.path.join(a_mask_dir, a_masks[i]), os.path.join(dst_a_mask_dir, new_name))

            # Copy B (Franka real)
            shutil.copy(os.path.join(b_img_dir, b_imgs[i]), os.path.join(dst_b_img_dir, new_name))
            shutil.copy(os.path.join(b_mask_dir, b_masks[i]), os.path.join(dst_b_mask_dir, new_name))

            dst_index += 1


# ✏️ Change start and end here
start_index = 92
end_index = 500


base = "/scratch/darshil/cross-emb-data/Wed_Oct_25_17:06:19_202328451778"
output = "/scratch/darshil/sim_2_real_dataset"

rename_and_copy_synchronized(
    a_img_dir=os.path.join(base, "simulation/xarm/left/rgb_images"),
    a_mask_dir=os.path.join(base, "simulation/xarm/left/masks"),
    b_img_dir=os.path.join(base, "images_left_resized"),
    b_mask_dir=os.path.join(base, "simulation/franka/left/masks"),

    dst_a_img_dir=os.path.join(output, "A"),
    dst_a_mask_dir=os.path.join(output, "A-Masks"),
    dst_b_img_dir=os.path.join(output, "B"),
    dst_b_mask_dir=os.path.join(output, "B-Masks"),

    start=start_index,
    end=end_index
)
