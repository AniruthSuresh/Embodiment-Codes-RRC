import os
import shutil
import random
from pathlib import Path

def make_dirs(base, splits=["train", "val", "test"]):
    for split in splits:
        for subdir in ["A", "A-Masks", "B", "B-Masks"]:
            os.makedirs(os.path.join(base, split, subdir), exist_ok=True)

def split_dataset(src_root, dst_root, val_pct=0.1, test_pct=0.1, seed=42):
    random.seed(seed)
    make_dirs(dst_root)

    a_files = sorted(os.listdir(os.path.join(src_root, "A")))
    total = len(a_files)
    indices = list(range(total))
    random.shuffle(indices)

    val_count = int(total * val_pct)
    test_count = int(total * test_pct)
    train_count = total - val_count - test_count

    splits = {
        "train": indices[:train_count],
        "val": indices[train_count:train_count + val_count],
        "test": indices[train_count + val_count:]
    }

    for split, idxs in splits.items():
        for i in idxs:
            name = a_files[i]
            for subdir in ["A", "A-Masks", "B", "B-Masks"]:
                src = os.path.join(src_root, subdir, name)
                dst = os.path.join(dst_root, split, subdir, name)
                shutil.copy(src, dst)

    print(f"Done! Total: {total} → Train: {train_count}, Val: {val_count}, Test: {test_count}")

# Example usage
split_dataset("/scratch/darshil/sim_2_real_dataset", "/scratch/darshil/sim_2_real_split")
