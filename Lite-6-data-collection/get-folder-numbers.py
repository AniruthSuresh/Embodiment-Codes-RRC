import re

input_file = "good-traj.txt"   # your input file with Folder blocks
output_file = "folders.txt"       # output with just folder numbers

folder_numbers = []

with open(input_file, "r") as f:
    for line in f:
        match = re.match(r"Folder\s+(\d+)", line.strip())
        if match:
            folder_numbers.append(match.group(1))

with open(output_file, "w") as f:
    for num in folder_numbers:
        f.write(num + "\n")

print(f"Extracted folder numbers saved to {output_file}")
