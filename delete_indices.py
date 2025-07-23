import os
import shutil

def copy_filtered_dataset(src_root, dst_root, skip_start=1050, skip_end=1300):
    os.makedirs(dst_root, exist_ok=True)
    subfolders = ["A", "A-Masks", "B", "B-Masks"]

    for sub in subfolders:
        src_dir = os.path.join(src_root, sub)
        dst_dir = os.path.join(dst_root, sub)
        os.makedirs(dst_dir, exist_ok=True)

        for file in sorted(os.listdir(src_dir)):
            if not file.endswith(".png"):
                continue

            try:
                idx = int(file.split("_")[1].split(".")[0])
            except:
                continue  # skip if filename doesn't match image_XXXX.png format

            if skip_start <= idx < skip_end:
                continue  # skip this file

            src_path = os.path.join(src_dir, file)
            dst_path = os.path.join(dst_dir, file)
            shutil.copy(src_path, dst_path)

    print(f"✅ Copied filtered dataset to '{dst_root}' (skipped images {skip_start}–{skip_end - 1})")

# Example usage
copy_filtered_dataset("/scratch/darshil/sim_2_real_dataset", "/scratch/darshil/sim_2_real_dataset_filtered", skip_start=1052, skip_end=1210)
