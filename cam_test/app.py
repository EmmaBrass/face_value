import cv2
import os
import datetime

print("Hello from the Raspberry Pi!")

# Create Pictures directory if it doesn't exist
destination_dir = os.path.expanduser("~/Pictures")
os.makedirs(destination_dir, exist_ok=True)

# Initialize webcam (device 0 is usually the first webcam)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam")
    exit(1)

# Set the resolution to maximum supported by the webcam
# First try 1080p (1920x1080)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Get actual resolution
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Capturing at resolution: {width}x{height}")

# Capture frame
ret, frame = cap.read()

if not ret:
    print("Error: Could not read frame")
    cap.release()
    exit(1)

# Generate filename with timestamp
filename = os.path.join(destination_dir, f"webcam_capture_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg")

# Save the image
cv2.imwrite(filename, frame)
print(f"Image saved to: {filename}")

# Release the webcam
cap.release()
cv2.destroyAllWindows()

print("Capture complete!")

