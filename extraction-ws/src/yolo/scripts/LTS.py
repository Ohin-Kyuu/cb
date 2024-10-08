#!/usr/bin/env python3
## Vision
from ultralytics import YOLO

## ROS
import tf2_ros as tf
import tf2_geometry_msgs
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray

## Other tools
import numpy as np
from cv_bridge import CvBridge
from threading import Thread, Event
import sys
import time
import threading

# Define variables
WEIGHT_PATH = "src/yolo/weight/new_cb.pt"
VERBOSE = False  # YOLO verbose (showing detection output)

def log_same_line():
    while not rospy.is_shutdown():
        sys.stdout.write("\033[2J\033[H")
        rospy.loginfo("Processing YOLO .  ")
        time.sleep(1)
        
        sys.stdout.write("\033[F\033[K")
        rospy.loginfo("Processing YOLO .. ")
        time.sleep(1)
        
        sys.stdout.write("\033[F\033[K")
        rospy.loginfo("Processing YOLO ...")
        time.sleep(1)

class Node:
    def __init__(self):
        rospy.init_node("CB_Server")

        ### YOLO model ###
        self.model = YOLO(WEIGHT_PATH)
        # self.model.fuse() # Fuse for speed
        self.results_img = None
        self.yolo_thread_started = False

        ### Publisher ###
        # GUI Publisher
        self.yolo_result_pub = rospy.Publisher("/cbcam/objects/yolo_result", Image, queue_size=10)
        # Camera Point Publisher
        self.world_point_array_pub = rospy.Publisher("/cbcam/objects/world_point_array", Float32MultiArray, queue_size=10)
        self.world_points = []

        self.camera_point = PointStamped()
        self.camera_point.header.frame_id = "realsense_camera"
        self.camera_point.header.stamp = rospy.Time.now()

        ### Subscriber ###
        self.col1_msg = None
        self.dep1_msg = None
        self.sub_col1 = rospy.Subscriber("/cbcam/color/image_raw", Image, self.col_callback1)
        self.sub_dep1 = rospy.Subscriber("/cbcam/aligned_depth_to_color/image_raw", Image, self.dep_callback1)

        ### Other tools ###
        # CvBridge
        self.bridge = CvBridge()
        # tf_listener
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

    def col_callback1(self, msg):
        self.col1_msg = msg

    def dep_callback1(self, msg):
        self.dep1_msg = msg

    def preprocess(self, col1_msg: Image, dep1_msg: Image) -> np.ndarray:
        # Convert col_msg
        cv_col1_img = self.bridge.imgmsg_to_cv2(col1_msg, desired_encoding="bgr8")
        np_col1_img = np.asanyarray(cv_col1_img, dtype=np.uint8)

        # Convert dep_msg
        cv_dep1_img = self.bridge.imgmsg_to_cv2(dep1_msg, desired_encoding="passthrough")
        np_dep1_img = np.asanyarray(cv_dep1_img, dtype=np.uint16)

        return np_col1_img, np_dep1_img

    def yolo(self):
        while rospy.is_shutdown() is False:
            if self.col1_msg is not None and self.dep1_msg is not None:
                color_img, depth_img = self.preprocess(self.col1_msg, self.dep1_msg)

                # YOLO detection
                results = self.model.predict(source=color_img, verbose=VERBOSE)

                for obj in results:
                    self.results_img = obj.plot()
                    self.yolo_result_pub.publish(self.bridge.cv2_to_imgmsg(self.results_img, encoding="bgr8"))
                    boxes = obj.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        pixel_x, pixel_y = round((x1 + x2) / 2), round((y1 + y2) / 2)
                        depth = depth_img[pixel_y, pixel_x]
                        # Transform coordinates
                        world_x, world_y = self.transform_coordinates(pixel_x, pixel_y, depth)

                        self.collect_world_points(world_x, world_y)

                self.publish_world_points()

                if not self.yolo_thread_started:
                    log_thread = threading.Thread(target=log_same_line)
                    log_thread.daemon = True
                    log_thread.start()
                    self.yolo_thread_started = True

    def transform_coordinates(self, x, y, depth):
        self.camera_point.point.x = (depth * (x - 436.413) / 604.357) / 1000
        self.camera_point.point.y = (depth * (y - 245.459) / 604.063) / 1000
        self.camera_point.point.z = depth / 1000
        self.camera_point.header.stamp = rospy.Time.now()

        try:
            self.tf_buffer.can_transform('map', 'realsense_camera', rospy.Time(0), rospy.Duration(1.0))
            world_point = tf2_geometry_msgs.do_transform_point(self.camera_point,
                                                               self.tf_buffer.lookup_transform('map', 'realsense_camera',
                                                                                                rospy.Time(0)))
            return world_point.point.x, world_point.point.y

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logerr("Transform error: %s", str(ex))
            return None
        
    def collect_world_points(self, world_x, world_y):
        world_point = PointStamped()
        world_point.header.frame_id = "map"
        world_point.point.x = world_x
        world_point.point.y = world_y
        world_point.point.z = 0.0

        self.world_points.append(world_point)

    def publish_world_points(self):
        world_point_array_msg = Float32MultiArray()

        for point in self.world_points:
            world_point_array_msg.data.append(point.point.x)
            world_point_array_msg.data.append(point.point.y)
            world_point_array_msg.data.append(point.point.z)

        self.world_point_array_pub.publish(world_point_array_msg)
        self.world_points.clear()

if __name__ == '__main__':
    try:
        vision_node = Node()
        rospy.loginfo("Starting CB detection...")
        vision_node.yolo()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass