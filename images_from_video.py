import cv2
import os

# Set the path for the input video and output directory
video_path = 'datasets/1112/1112.mp4'  # Replace with your video file path
output_dir = 'datasets/1112'  # Directory where frames will be saved

# Create the output directory if it doesn't exist
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Open the video file
cap = cv2.VideoCapture(video_path)

# Check if the video file opened successfully
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Get the video's frames per second (fps)
fps = cap.get(cv2.CAP_PROP_FPS)
print(fps)
frame_interval = int(fps * 0.5)  # Number of frames to skip to get 0.1-second intervals

# Frame counter for output filenames
frame_number = 0
frame_count=0
# Read and save frames at the specified interval
while True:
    # Set the current position in the video
    ret, frame = cap.read()

    # If the frame is not successfully read, break the loop
    if not ret:
        break

    # Check if this frame is at the required interval
    if frame_number % frame_interval == 0:
        # Construct the output filename and save the frame
        frame_filename = os.path.join(output_dir, f"frame_{frame_count:04d}.png")
        cv2.imwrite(frame_filename, frame)
        print(f"Saved {frame_filename}")
        frame_count+=1

    # Increment the frame counter
    frame_number += 1
    
# Release the video capture object
cap.release()
print("Video has been converted to images at 0.1-second intervals.")
