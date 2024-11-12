import os
import shutil
from glob import glob

# Path to the directory you want to scan
source_dir = "output"  # Replace with the path to your source directory
target_dir = "datasets/1112_2"

# Create the target directory if it doesn't exist
if not os.path.exists(target_dir):
    os.makedirs(target_dir)

# Find all .png files in the source directory and its subdirectories
png_files = glob(os.path.join(source_dir, '**', '*.png'), recursive=True)

# Move each .png file to the target directory
for file_path in png_files:
    try:
        # Move file to the target directory
        shutil.move(file_path, target_dir)
        print(f"Moved {file_path} to {target_dir}")
    except Exception as e:
        print(f"Failed to move {file_path}: {e}")

print("All .png files have been moved successfully.")
