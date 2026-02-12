from ultralytics import YOLO
import cv2
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
import numpy as np
# Load the YOLOv8 model
cam = CameraServer.startAutomaticCapture()
cam.setResolution(640, 480)

output_stream = CameraServer.putVideo("Fuel Detection", 640, 480)
model = YOLO('best_ncnn_model')  # You can choose a different model size if needed
stream = model.predict(source="1", stream=True, conf=0.9)  # Use the default camera and display the results
# cap = cv2.VideoCapture(0)  # Use the default camera
for i in stream:
    frame = i.plot()
    # Perform any additional processing on the frame if needed
    output_stream.putFrame(frame)  # Send the processed frame to the output stream


img = np.empty((480, 640, 3), dtype=np.uint8)  # Create an empty image
while True:
    img = np.ascontiguousarray(img)  # Ensure the image is contiguous in memory
    time, img = cam.grabFrame(img)  # Grab a frame from the camera
    if time == 0:
        print("Error grabbing frame")
        continue
    output_stream.putFrame(img)  # Send the frame to the output stream