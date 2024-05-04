#!/usr/bin/env python3
## Vision
from ultralytics import YOLO
import pyrealsense2 as rs

## ROS
import tf2_ros as tf
import tf2_geometry_msgs
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int8MultiArray

## Other tools
import numpy as np
from cv_bridge import CvBridge
from threading import Thread, Event

# Define variables
WIN_WIDTH, WIN_HEIGHT = 848, 480 # camera resolution
WEIGHT_PATH = "src/yolo/weight/new_cb.pt"
VERBOSE = False # YOLO verbose (showing detection ouput)
WAITING_TIME = 0 # waiting time for first publish

six_region_map = {
    # "Region": [x1, x2, y1, y2]
    "1": [-0.625, -0.375, -0.425, -0.175],
    "2": [-0.625, -0.375, 0.175, 0.425],
    "3": [-0.125, 0.125, -0.625, -0.375],
    "4": [-0.125, 0.125, 0.375, 0.625],
    "5": [0.375, 0.625, -0.425, -0.175],
    "6": [0.375, 0.625, 0.175, 0.425]
}

class Node:
    def __init__(self, realsense_camera):
        rospy.init_node('CB_Server')

        ### YOLO model ###
        self.model = YOLO(WEIGHT_PATH)
        #self.model.fuse() # Fuse for speed
        self.results_img = None

        ### Publisher ###
        # CB detection topic
        self.pub = rospy.Publisher('/robot/objects/global_info', Int8MultiArray, queue_size=10)
        self.six_plant_info = Int8MultiArray()
        self.six_plant_info.data = [0] * 6
        # GUI Publisher
        self.yolo_result_pub = rospy.Publisher('/cb/objects/yolo_result', Image, queue_size=10)
        # Camera Point Publisher
        self.camera_point_pub = rospy.Publisher('/cb/objects/camera_point', PointStamped, queue_size=10)
        self.camera_point = PointStamped()
        self.camera_point.header.frame_id = "realsense_camera" 
        self.camera_point.header.stamp = rospy.Time.now()
        
        ### Other tools ###
        # CvBridge
        self.bridge = CvBridge()
        # tf_listener
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        ###### YOLO STARTS HERE ######
        self.yolo_thread = Thread(target=self.yolo, args=(realsense_camera,))
        self.yolo_thread.start()

    def yolo(self, realsense_camera):
        while not rospy.is_shutdown():
            color_img, depth_img = realsense_camera.wait_for_frames()
            #YOLO detection
            results = self.model(source = color_img, verbose = VERBOSE)

            for object in results.pred:
                self.results_img = object.plot()
                self.yolo_result_pub.publish(self.bridge.cv2_to_imgmsg(self.results_img, encoding="bgr8")) 
                boxes = object.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    pixel_x, pixel_y = round((x1 + x2) / 2), round((y1 + y2) / 2)
                    depth = depth_img[pixel_y, pixel_x]
                    # Transform coordinates
                    world_x, world_y = self.transform_coordinates(pixel_x, pixel_y, depth)
                    # Check 6 plant info
                    self.six_plant_info_check(world_x, world_y)

            # Publish 6 plant info
            self.pub.publish(self.six_plant_info)
            self.six_plant_info.data = [0] * 6

    def transform_coordinates(self, x, y, depth):
        self.camera_point.point.x = (depth * (x - 436.413) / 604.357) / 1000
        self.camera_point.point.y = (depth * (y - 245.459) / 604.063) / 1000
        self.camera_point.point.z = depth/1000
        self.camera_point.header.stamp = rospy.Time.now()

        try:
            self.tf_buffer.can_transform('map', 'realsense_camera', rospy.Time(0), rospy.Duration(1.0))
            world_point = tf2_geometry_msgs.do_transform_point(self.camera_point, self.tf_buffer.lookup_transform('map', 'realsense_camera', rospy.Time(0)))
            # Publish world point
            self.camera_point_pub.publish(world_point)
            return world_point.point.x, world_point.point.y
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
            rospy.logerr("Transform error: %s", str(ex))
            return None
    
    def six_plant_info_check(self, x, y):
        # if match the region, set the corresponding value to 1
        for region, value in six_region_map.items():
            if (value[0] < x < value[1]) and (value[2] < y < value[3]):
                self.six_plant_info.data[int(region)-1] = 1

class RealsenseCamera:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device('215222079970')
        self.config.enable_stream(rs.stream.color, WIN_WIDTH, WIN_HEIGHT, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, WIN_WIDTH, WIN_HEIGHT, rs.format.z16, 30)
        self.profile = self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        self.intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    def wait_for_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        color_img = np.asanyarray(color_frame.get_data())
        depth_img = np.asanyarray(depth_frame.get_data())
        return color_img, depth_img
        
if __name__ == '__main__':
    try:
        realsense_camera = RealsenseCamera()
        vision_node = Node(realsense_camera)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
