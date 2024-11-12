import rospy
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from glob import glob

# Global variables
left_image_rect = None
bridge = CvBridge()

# Output directory setup
output_dir = "datasets/1112_2"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Function to get the next sequential filename
def get_next_filename_count(directory):
    files = glob(os.path.join(directory, "*.png"))
    if not files:
        return 1  # Start numbering from 1
    latest_file = max(files, key=lambda x: int(os.path.splitext(os.path.basename(x))[0]))
    latest_num = int(os.path.splitext(os.path.basename(latest_file))[0])
    return latest_num + 1

# Callback function for the left camera rectified image
def left_image_rect_callback(msg):
    global left_image_rect
    left_image_rect = bridge.imgmsg_to_cv2(msg, "bgr8")

# Function to save the left image
def save_left_image(event):
    global left_image_rect
    if left_image_rect is not None:
        # Get the next sequential filename
        count = get_next_filename_count(output_dir)
        left_rect_filename = os.path.join(output_dir, f"{count:04d}.png")

        # Save the image
        cv2.imwrite(left_rect_filename, left_image_rect)
        
        print(f"Saved {left_rect_filename}")
    else:
        print("No rectified left image received yet.")

if __name__ == "__main__":
    rospy.init_node('left_image_saver')

    # Subscribe to the left camera rectified image
    rospy.Subscriber("/zedm/zed_node/left/image_rect_color", Image, left_image_rect_callback)

    # Set a timer to save the image every 0.5 seconds
    rospy.Timer(rospy.Duration(0.5), save_left_image)

    # Keep the node running
    rospy.spin()
