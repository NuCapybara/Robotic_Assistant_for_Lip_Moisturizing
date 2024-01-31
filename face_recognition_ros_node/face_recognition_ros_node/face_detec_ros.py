import rclpy
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


class FaceDetection(Node):

    def __init__(self):
        super().__init__('face_detection')
        self.bridge = CvBridge()
        self._depth_image_topic = (
            "camera/aligned_depth_to_color/image_raw"
        )
        self._depth_info_topic = "/camera/depth/camera_info"
        self.inital_lips_points = None #only for upper lip points now
        self._latest_depth_img = None
        self._latest_color_img = None
        self._latest_color_img_ts = None
        self.inference_ts = None
        self.intrinsics = None
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
        
    def face_detection(self):
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
                                # Extract lips points
                    upper_lip_outer = shape[48:55]
                    upper_lip_inner = shape[60:65]
                    lower_lip_outer = shape[54:60]
                    lower_lip_inner = shape[64:68]
                    self.inital_lips_points = upper_lip_outer #just for first testing

                    # Example: Draw the lips points
                    for (x, y) in upper_lip_outer:
                        cv2.circle(color_image, (x, y), 1, (0, 255, 0), -1)  # Green
                    for (x, y) in upper_lip_inner:
                        cv2.circle(color_image, (x, y), 1, (255, 0, 0), -1)  # Blue
                    for (x, y) in lower_lip_outer:
                        cv2.circle(color_image, (x, y), 1, (0, 255, 0), -1)  # Green
                    for (x, y) in lower_lip_inner:
                        cv2.circle(color_image, (x, y), 1, (255, 0, 0), -1)  
                    # for (x, y) in shape:
                    #     cv2.circle(color_image, (x, y), 1, (0, 0, 255), -1)

                # Show images
                cv2.imshow('RealSense', color_image)
                fps.update()
                
                #using the lips points to get the first set of xy in upper lips in the world frame
                #just for 1 point now!
                if self.inital_lips_points:
                    x1, y1, z1 = self.depth_world(self.inital_lips_points[0][0], self.inital_lips_points[0][1])
                    self.x1 = x1   
                    self.y1 = y1
                    self.z1 = z1
                    print(f"X: {x1}, Y: {y1}, Z: {z1}")
                    self.lip_pose_pub.publish(Point(x=x1, y=y1, z=z1))

                # Break loop on 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            # Stop streaming
            pipeline.stop()
            fps.stop()
            print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))




            
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
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self._latest_depth_img = cv_image
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return


    def get_latest_frame(self, data):
        """
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

def main():
    rclpy.init()

    face_detection = FaceDetection()

    rclpy.spin(face_detection)

    rclpy.shutdown()


if __name__ == '__main__':
    main()