from imutils.video import FPS
from imutils import face_utils
import datetime
import argparse
import imutils
import time
import dlib
import cv2
import pyrealsense2 as rs
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("-p", "--shape-predictor", required=True,
    help="path to facial landmark predictor")
ap.add_argument("-r", "--picamera", type=int, default=-1,
	help="whether or not the Raspberry Pi camera should be used")
args = vars(ap.parse_args())

print("[INFO] loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(args["shape_predictor"])

# Initialize camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
fps = FPS().start()
time.sleep(2.0)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        # Use imutils to perform image processing, e.g., resizing
        color_image = imutils.resize(color_image, width=450)
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        rects = detector(gray, 0)

        for rect in rects:
            shape = predictor(gray, rect)
            shape = face_utils.shape_to_np(shape)

            for (x, y) in shape:
                cv2.circle(color_image, (x, y), 1, (0, 0, 255), -1)

        # Show images
        cv2.imshow('RealSense', color_image)
        fps.update()

        # Break loop on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    fps.stop()
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))