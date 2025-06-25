import ast
import numpy as np

# Path to your text file
file_path = '/scratch/darshil/cross-emb-data/Tue_Jun__6_11:52:15_202329838012/camera_params.txt'  # Change this to your actual file path


with open(file_path, 'r') as f:
    content = f.read()

# Isolate the 'Camera Intrinsics (Left)' section
sections = content.split('Camera Intrinsics (Left):')
if len(sections) < 2:
    raise ValueError("Could not find 'Camera Intrinsics (Left)' in file")

left_str = sections[1].split('\n\n')[0].strip()  # Stop at next section if exists

# Replace 'array' with 'np.array' to evaluate correctly
left_str = left_str.replace('array', 'np.array')

# Evaluate the dictionary string
left_intrinsics = eval(left_str, {'np': np})  # Limit eval scope to only numpy

# Access data
camera_matrix = left_intrinsics['cameraMatrix']
dist_coeffs = left_intrinsics['distCoeffs']

print("Camera Matrix (Left):")
print(camera_matrix)
print("\nDistortion Coefficients (Left):")
print(dist_coeffs)
