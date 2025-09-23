import os
import re

def parse_min_max(line):
    """Extract X, Y, Z values from a line like 'Minimum : X: 0.2537, Y: 0.0044, Z: 0.2275'."""
    nums = re.findall(r"[-+]?\d*\.\d+|\d+", line)
    return list(map(float, nums))

def compute_translation(axis, min_val, max_val, min_limit_x=0.09, max_limit=0.40):
    """
    Compute the minimum safe translation:
      - For all axes: max <= 0.40
      - For X only: min >= 0.09
    """
    if axis == "X":
        if min_val >= min_limit_x and max_val <= max_limit:
            return 0.0, "already safe"
        # compute shifts needed
        shift_for_min = min_limit_x - min_val
        shift_for_max = max_limit - max_val
        # pick the one that satisfies both (max of lower bounds)
        shift = max(shift_for_min, shift_for_max)
        # check feasibility
        if min_val + shift >= min_limit_x and max_val + shift <= max_limit:
            return shift, "translated"
        else:
            return None, "infeasible"
    else:  # Y and Z
        if max_val <= max_limit:
            return 0.0, "already safe"
        shift = max_limit - max_val  # minimal negative shift
        return shift, "translated"

def process_file(txt_path, dst_root):
    with open(txt_path, "r") as f:
        lines = f.readlines()

    folder_data = {}
    folder_id = None

    for line in lines:
        line = line.strip()
        if line.startswith("Folder"):
            folder_id = line.split()[1]
            folder_data[folder_id] = {}
        elif line.startswith("Minimum"):
            folder_data[folder_id]["min"] = parse_min_max(line)
        elif line.startswith("Max"):
            folder_data[folder_id]["max"] = parse_min_max(line)

    # Compute safe translations and corrected values
    results = {}
    for fid, vals in folder_data.items():
        mins, maxs = vals["min"], vals["max"]
        trans_info = {}
        corrected = {"min": [], "max": []}

        for axis, m, M in zip("XYZ", mins, maxs):
            shift, status = compute_translation(axis, m, M)
            trans_info[axis] = {"shift": shift, "status": status}
            if shift is not None:
                corrected["min"].append(m + shift)
                corrected["max"].append(M + shift)
            else:
                corrected["min"].append(None)
                corrected["max"].append(None)

        results[fid] = {"translations": trans_info, "corrected": corrected}

        # --- Save translations inside subfolder ---
        dst_folder = os.path.join(dst_root, fid)
        os.makedirs(dst_folder, exist_ok=True)
        trans_file = os.path.join(dst_folder, "translation.txt")
        with open(trans_file, "w") as f:
            for axis in "XYZ":
                shift = trans_info[axis]["shift"]
                status = trans_info[axis]["status"]
                f.write(f"{axis}: {status}, shift={shift}\n")
        print(f"Saved translations to {trans_file}")

    return results


if __name__ == "__main__":
    txt_file = "good-traj.txt"   # path to your .txt
    dst_root = "./lite-6-execute-traj"  # where subfolders already exist
    results = process_file(txt_file, dst_root)

    for fid, vals in results.items():
        print(f"\nFolder {fid}:")
        for axis, info in vals["translations"].items():
            print(f"  {axis}: {info['status']}, shift={info['shift']}")
