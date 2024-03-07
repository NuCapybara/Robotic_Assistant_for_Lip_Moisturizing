import rclpy
import os
from rclpy.node import Node
from imutils.video import FPS
from imutils import face_utils
import math
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
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseArray, Pose

class FaceDetection(Node):
    def __init__(self):
        super().__init__("face_detection")
        self.bridge = CvBridge()
        self.intrinsics = None
        self._depth_info_topic = "/camera/color/camera_info"
        self._depth_image_topic = "/camera/aligned_depth_to_color/image_raw"
        self.inital_lips_points = None  # only for upper lip points now
        self._latest_depth_img = None
        self._latest_color_img = None
        self._latest_color_img_ts = None
        self.inference_ts = None
        self.x1 = None
        self.y1 = None
        self.z1 = None
        self.left_x = None
        self.left_y = None
        self.x_point_lip = [0, 0, 0, 0, 0, 0, 0]
        self.y_point_lip = [0, 0, 0, 0, 0, 0, 0]
        self.poseArray = PoseArray()

        self.lip_pose_pub = self.create_publisher(PoseArray, "lip_pose", 10)
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

        self.original_publisher = self.create_publisher(
            msg_Image, "/original_mask", qos_profile=10
        )
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.make_transforms()

        # create a timer
        self.declare_parameter("frequency", 100.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.timer = self.create_timer(1 / self.frequency, self.timer_callback)


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



        # DEBUG PROCESS
        # Original width and height
        self.scale_factor = 0.0
        self.original_x = 0.0
        self.original_y = 0.0
        
        

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
            # for x, y in upper_lip_outer:
            #     cv2.circle(color_image, (x, y), 1, (0, 255, 0), -1)  # Green
            # for x, y in upper_lip_inner:
            #     cv2.circle(color_image, (x, y), 1, (255, 0, 0), -1)  # Blue
            # for x, y in lower_lip_outer:
            #     cv2.circle(color_image, (x, y), 1, (0, 255, 0), -1)  # Green
            # for x, y in lower_lip_inner:
            #     cv2.circle(color_image, (x, y), 1, (255, 0, 0), -1)

            cv2.circle(color_image, (self.inital_lips_points[0][0], self.inital_lips_points[0][1]), 1, (0, 0, 255), -1)  # DEBUG RED
            cv2.circle(color_image, (self.inital_lips_points[1][0], self.inital_lips_points[1][1]), 1, (0, 255, 0), -1)  # DEBUG GREEN
            cv2.circle(color_image, (self.inital_lips_points[2][0], self.inital_lips_points[2][1]), 1, (0, 255, 0), -1)
            cv2.circle(color_image, (self.inital_lips_points[3][0], self.inital_lips_points[3][1]), 1, (0, 255, 0), -1)
            cv2.circle(color_image, (self.inital_lips_points[4][0], self.inital_lips_points[4][1]), 1, (0, 255, 0), -1)
            cv2.circle(color_image, (self.inital_lips_points[5][0], self.inital_lips_points[5][1]), 1, (0, 255, 0), -1)
            cv2.circle(color_image, (self.inital_lips_points[6][0], self.inital_lips_points[6][1]), 1, (0, 255, 0), -1)

        
            self.depth_publisher.publish(self.bridge.cv2_to_imgmsg(color_image))
            
            #Append the seven points to the x,y list
            for i in range(7):
                scale_x = int(self.inital_lips_points[i][0]*self.scale_factor)
                scale_y = int(self.inital_lips_points[i][1]*self.scale_factor)
                self.x_point_lip[i] = scale_x
                self.y_point_lip[i] = scale_y

            self.left_x = int(self.inital_lips_points[0][0] * self.scale_factor)
            self.left_y = int(self.inital_lips_points[0][1] * self.scale_factor)


            #DEBUG: Draw the lips point on the original image    
            for x, y in upper_lip_outer:
                self.original_x = x * self.scale_factor
                self.original_y = y * self.scale_factor
            
            for i in range(7):
                # cv2.circle(self._latest_color_img, (self.left_x, self.left_y), 1, (0, 255, 0), -1)  # DEBUG GREEN
                cv2.circle(self._latest_color_img, (self.x_point_lip[i], self.y_point_lip[i]), 1, (0, 255, 0), -1)  # DEBUG GREEN
            #after draw the points on rescaled image, publish the image
            self.original_publisher.publish(self.bridge.cv2_to_imgmsg(self._latest_color_img))
            
        # Show images
        self.fps.update()

        # using the lips points to get the first set of xy in upper lips in the world frame
        # just for 1 point now!
        # if self.inital_lips_points is not None:
        for i in range(7):
            if self.x_point_lip[i] is not None and self.y_point_lip[i] is not None:
                # self.get_logger().info(
                #     f"left x: {self.left_x}, left y: {self.left_y}"
                # )
                x1, y1, z1 = self.depth_world(
                    self.x_point_lip[i], self.y_point_lip[i]
                )
                self.x1 = x1
                self.y1 = y1
                self.z1 = z1

                if self.x1 != 0.0 and self.y1 != 0.0 and self.z1 != 0.0:
                    if(i == 0):
                        #broadcast the leftmost point of the lip
                        self.tf_broadcaster_func(self.x1, self.y1, self.z1, 0.0)
                    # self.get_logger().info(
                    #     f"Real world coordinates x: {x1}, y: {y1}, z: {z1}"
                    # )
                    self.poseArray.header.stamp = self.get_clock().now().to_msg()
                    self.poseArray.header.frame_id = "head_link"
                    
                    pose_push = Pose()
                    pose_push.position.x = x1
                    pose_push.position.y = y1
                    pose_push.position.z = z1
                    self.poseArray.poses.append(pose_push)

                    # self.lip_pose_pub.publish(Point(x=x1, y=y1, z=z1))
                    

        self.lip_pose_pub.publish(self.poseArray)
        
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
        # self.get_logger().info(f"i went in depth_world function!!!!")
        # received = (self._latest_depth_img)
        # self.get_logger().info(f"received: {received}")
        # self.get_logger().info(f"i went in depth_world function!!!!")
        # received = (self._latest_depth_img)
        # self.get_logger().info(f"received: {received}")
        if (
            self.intrinsics
            and self._latest_depth_img is not None
            and self._latest_color_img is not None
        ):

            depth_x = int(x)
            depth_y = int(y)
            
            
            # self.get_logger().info(f"depth_x: {depth_x}, depth_y: {depth_y}")
            # depth = self._latest_depth_img[depth_x, depth_y]
            depth = self._latest_depth_img[depth_y, depth_x]
            result = rs.rs2_deproject_pixel_to_point(self.intrinsics, [x, y], depth)

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

            #DEBUG
            self.scale_factor = self.intrinsics.width / 450
            self.get_logger().info(f"intrinsics_width: {self.intrinsics.width}, depth_y: { self.intrinsics.height}")
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

    def quaternion_from_euler(ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q
    def tf_broadcaster_func(self, x, y, z, theta):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'head_link'

        

        # t.transform.translation.x = x/1000
        # t.transform.translation.y = y/1000
        # t.transform.translation.z = z/1000
        
        t.transform.translation.x = z/1000
        t.transform.translation.y = -x/1000
        t.transform.translation.z = -y/1000

        self.get_logger().info(f"translation.x: {t.transform.translation.x}, translation.y: {t.transform.translation.y}, translation.z: {t.transform.translation.z}")
        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = FaceDetection.quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
    

    #define static transform broadcaster
    def make_transforms(self):
        tran = TransformStamped()

        tran.header.stamp = self.get_clock().now().to_msg()
        tran.header.frame_id = 'wx200/base_link'
        tran.child_frame_id = 'camera_link'
        
        # tran.transform.translation.x = 0.0
        tran.transform.translation.x = 0.015
        # tran.transform.translation.y = -0.155
        tran.transform.translation.y = -0.165
        # tran.transform.translation.z = 0.16
        tran.transform.translation.z = 0.175
        quat = FaceDetection.quaternion_from_euler(0, 0, 0)
        tran.transform.rotation.x = quat[0]
        tran.transform.rotation.y = quat[1]
        tran.transform.rotation.z = quat[2]
        tran.transform.rotation.w = quat[3]

        self.tf_static_broadcaster.sendTransform(tran)

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