import rclpy
import os
from rclpy.node import Node
from imutils.video import FPS
from imutils import face_utils
import datetime
import argparse
import imutils
import time
import dlib
import cv2
import pyrealsense2 as rs
import copy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory


class FaceDetection(Node):
    def __init__(self):
        super().__init__("face_detection")
        self.bridge = CvBridge()
        self.intrinsics = None
        self._depth_info_topic = "/camera/depth/camera_info"
        self._depth_image_topic = "/camera/depth/image_rect_raw"
        self.inital_lips_points = None  # only for upper lip points now
        self._latest_depth_img = None
        self._latest_color_img = None
        self._latest_color_img_ts = None
        self.inference_ts = None
        self.x1 = None
        self.y1 = None
        self.z1 = None
        self.lip_pose_pub = self.create_publisher(Point, "lip_pose", 10)
        self.sub_depth = self.create_subscription(
            msg_Image, self._depth_image_topic, self.imageDepthCallback, 1
        )
        self.sub_info = self.create_subscription(
            CameraInfo, self._depth_info_topic, self.imageDepthInfoCallback, 1
        )
        self.sub1 = self.create_subscription(
            msg_Image, "/camera/color/image_raw", self.get_latest_frame, 1
        )
        self.depth_publisher = self.create_publisher(
            msg_Image, "/depth_mask", qos_profile=10
        )
        # create a timer
        self.declare_parameter("frequency", 100.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)

        # argument parser
        # self.ap = argparse.ArgumentParser()
        # self.ap.add_argument("-p", "--shape-predictor", required=True,
        #     help="path to facial landmark predictor")
        # self.ap.add_argument("-r", "--picamera", type=int, default=-1,
        #     help="whether or not the Raspberry Pi camera should be used")
        # self.args = vars(self.ap.parse_args())

        # define the facial landmark predictor and detector
        print("[INFO] loading facial landmark predictor...")
        # Define the base directory where your data file is located
        data_directory = "/home/jialuyu/Winter_Project/src/Winter_Project/face_recognition_ros_node/data"

        # Append the filename to create the full path
        shape_predictor_path = os.path.join(
            data_directory, "shape_predictor_68_face_landmarks.dat"
        )

        self.predictor = dlib.shape_predictor(shape_predictor_path)
        self.detector = dlib.get_frontal_face_detector()
        # self.predictor = dlib.shape_predictor(self.args["shape_predictor"])

        # Initialize camera
        # self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        # self.pipeline.start(self.config)
        self.fps = FPS().start()
        time.sleep(2.0)

    def timer_callback(self):
        # Wait for a coherent pair of frames: depth and color
        # frames = self.pipeline.wait_for_frames()
        color_frame = self._latest_color_img
        if color_frame is None:
            self.get_logger().info("No frame received, skipping...")
            return  # Skip the current iteration if frame is None or invalid

        # # Use imutils to perform image processing, e.g., resizing
        color_image = imutils.resize(color_frame, width=450)
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        rects = self.detector(gray, 0)

        for rect in rects:
            shape = self.predictor(gray, rect)
            shape = face_utils.shape_to_np(shape)
            # Extract lips points
            upper_lip_outer = shape[48:55]
            upper_lip_inner = shape[60:65]
            lower_lip_outer = shape[54:60]
            lower_lip_inner = shape[64:68]
            self.inital_lips_points = upper_lip_outer  # just for first testing

            # Example: Draw the lips points
            for x, y in upper_lip_outer:
                cv2.circle(color_image, (x, y), 1, (0, 255, 0), -1)  # Green
            for x, y in upper_lip_inner:
                cv2.circle(color_image, (x, y), 1, (255, 0, 0), -1)  # Blue
            for x, y in lower_lip_outer:
                cv2.circle(color_image, (x, y), 1, (0, 255, 0), -1)  # Green
            for x, y in lower_lip_inner:
                cv2.circle(color_image, (x, y), 1, (255, 0, 0), -1)
            self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(color_image))
            # for (x, y) in shape:
            #     cv2.circle(color_image, (x, y), 1, (0, 0, 255), -1)

        # Show images
        # cv2.imshow('RealSense', color_image)
        self.fps.update()

        # using the lips points to get the first set of xy in upper lips in the world frame
        # just for 1 point now!
        if self.inital_lips_points is not None:
            x1, y1, z1 = self.depth_world(
                self.inital_lips_points[0][0], self.inital_lips_points[0][1]
            )
            self.x1 = x1
            self.y1 = y1
            self.z1 = z1
            print(f"X: {x1}, Y: {y1}, Z: {z1}")
            self.lip_pose_pub.publish(Point(x=x1, y=y1, z=z1))

        # Break loop on 'q'
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.timer.cancel()  # Optionally cancel the timer
            return

    def depth_world(self, x, y):
        """
        Convert pixel coordinates to real-world coordinates using depth information.

        Args:
        ----
            x (int): X-coordinate.
            y (int): Y-coordinate.

        Returns
        -------
            Tuple[float, float, float]: Real-world coordinates (x, y, z).

        """
        
        if (
            self.intrinsics
            and self._latest_depth_img is not None
            and self._latest_color_img is not None
        ):
            
            self.get_logger().info("processing request")

            depth_x = int(x)
            depth_y = int(y)
            depth = self._latest_depth_img[depth_x, depth_y]
            
            result = rs.rs2_deproject_pixel_to_point(self.intrinsics, [y, x], depth)
            print(self.intrinsics)
            x_new, y_new, z_new = result[0], result[1], result[2]

            return x_new, y_new, z_new

    def imageDepthInfoCallback(self, cameraInfo):
        """
        Obtain depth camera information.

        Args:
        ----
            cameraInfo (CameraInfo): Camera information message.

        Returns
        -------
            None

        """
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == "plumb_bob":
                self.intrinsics.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == "equidistant":
                self.intrinsics.model = rs.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return

    def imageDepthCallback(self, data):
        """
        Obtain latest depth image.

        Args:
        ----
            data (Image): Depth image message.

        Returns
        -------
            None

        """
        self.get_logger().info("Into image depth call back!!!!!!!!!!!!!!")
        try:
            self.get_logger().info("Received depth image!!!!!!!!!!!!!!")
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.get_logger().info(str(type(cv_image)))
            self._latest_depth_img = cv_image
        except CvBridgeError as e:
            self.get_logger().error("CvBridgeError in imageDepthCallback: {}".format(e))

        except ValueError as e:
            self.get_logger().error("ValueError in imageDepthCallback: {}".format(e))
            return

    def get_latest_frame(self, data):
        """
        # Other resource cleanup if needed
        Call for the latest color image.

        Args:
        ----
            data (Image): Color image message.

        Returns
        -------
            None

        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self._latest_color_img = cv_image
            self._latest_color_img_ts = data.header.stamp
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return

    # def cleanup(self):
    #     self.pipeline.stop()
    #     self.fps.stop()
    #     # Other resource cleanup if needed


def main():
    rclpy.init()
    face_detection = FaceDetection()

    try:
        rclpy.spin(face_detection)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C here if needed
    finally:
        # Stop streaming and other cleanup
        # face_detection.cleanup()
        rclpy.shutdown()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
