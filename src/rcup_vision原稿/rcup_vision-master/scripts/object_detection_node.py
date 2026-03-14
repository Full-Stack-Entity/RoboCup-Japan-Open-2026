#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Int32MultiArray
from cv_bridge import CvBridge
from math import *
import numpy as np
import os
import tempfile
from PIL import Image as PILImage
import cv2

import tf2_ros
import tf
from geometry_msgs.msg import Pose, PoseStamped

# from roboflow import Roboflow
from ultralytics import YOLO

# TODO: visualize the detection for easier debugging later

# the dining-table and cup model

# rf = Roboflow(api_key="9aItLmXFlLysf5Lp08E9")
# project = rf.workspace().project("cup__dining-table")
# model = project.version(1).model

# latest model
# TODO: get the finalized model when training is completed
# rf = Roboflow(api_key="4wb6F4QuEBppNC9K9cXX")
# project = rf.workspace().project("robocup-home")
# model = project.version(1).model

# get the absolute path to the model, 
# relative paths cause a mess with ros
model_name = "last.pt"
current_directory = os.path.dirname(__file__)
file_path = os.path.join(current_directory, model_name)

model = YOLO(file_path)

print("Model loaded successfully")

head_cam_topic = '/hsrb/head_rgbd_sensor/rgb/image_raw'
hand_cam_topic = '/hsrb/hand_camera/image_raw' 
depth_cam_topic = '/hsrb/head_rgbd_sensor/depth_registered/image_raw'

class_names = [
    "apple",
    "bear_doll",
    "canned_juice",
    "cigarette",
    "clock",
    "dog_doll",
    "empty_ketchup",
    "empty_plastic_bottle",
    "filled_ketchup",
    "filled_plastic_bottle",
    "game_controller",
    "ground_pepper",
    "hourglass",
    "matryoshka",
    "nursing_bottle",
    "piggy_bank",
    "pink_cup",
    "rabbit_doll",
    "rubik-s_cube",
    "salt",
    "sauce",
    "soysauce",
    "spray_bottle",
    "sugar",
    "toy_car",
    "toy_duck",
    "toy_penguin",
    "tumbler",
    "white_cup",
    "white_side_table"
]

# YOLO uses indices to refer to the classes for some reason
name_to_index = {name: index for index, name in enumerate(class_names)}
index_to_name = {index: name for index, name in enumerate(class_names)}


class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('object_detection_node')

        self.rgb_image = None
        self.hand_image = None
        self.depth_image = None
        self.unused_depth = False
        self.hand_ready = False
        self.detected = False

        self.detection_target = 'sugar' # random initial target from my behind

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(head_cam_topic, Image, self.image_callback,  queue_size=1)
        rospy.Subscriber(hand_cam_topic, Image, self.hand_cam_callback, queue_size=1)
        rospy.Subscriber(depth_cam_topic, Image, self.depth_image_callback,  queue_size=1)
        rospy.Subscriber('/detection_target', String, self.set_detection_target)

        self.depth_pub = rospy.Publisher('/detection_depth', Float32, queue_size=10)
        self.pose_publisher = rospy.Publisher('/vision', PoseStamped, queue_size=10)
        self.hand_pub = rospy.Publisher('/hand_detection',Int32MultiArray,queue_size = 10)

        # Initialize the object detection model
        self.model = model

        # Initialize CvBridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")
            self.rgb_image = cv2.cvtColor(self.rgb_image,cv2.COLOR_RGB2BGR)
            
            # target_class = int(name_to_index.get(self.detection_target))
            # predictions = self.model.predict(self.rgb_image, conf=0.5, classes=target_class ,verbose = False, max_det=1) 
            # prediction = predictions[0] # only have one image at a time
            # rospy.loginfo("Head cam detection...")

            # for box in prediction.boxes:
            #     cls_name = index_to_name.get( int(box.cls[0].item()) )
            #     conf = round(box.conf[0].item(), 2)
            #     box_dims = [int(x) for x in box.xywh[0].tolist()]
            #     box_debug = [int(x) for x in box.xyxy[0].tolist()] 
            #     # cv2.rectangle(self.rgb_image,(box_debug[0],box_debug[1]),(box_debug[2],box_debug[3]),(255,0,0))
            #     # print("\n")
            #     # print("---------------------------------------------------")
            #     # print("head detection...")
            #     # print(f"detected class: {cls_name}")
            #     # print(f"confidence     : {conf}")
            #     # print(f"x: {box_dims[0]}, y: {box_dims[1]}, width: {box_dims[2]}, height: {box_dims[3]}")

            #     # can't get depth unless a frame is received from the depth topic
            #     if self.unused_depth:
            #         self.unused_depth = False

            #         center_x = box_dims[0]
            #         center_y = box_dims[1]
            #         detection_width = box_dims[2]
            #         detection_height = box_dims[3]

            #         # Calculate the coordinates of the rectangle's top-left and bottom-right corners
            #         top_left_x = int(center_x - detection_width / 2)
            #         top_left_y = int(center_y - detection_height / 2)
            #         bottom_right_x = int(center_x + detection_width / 2)
            #         bottom_right_y = int(center_y + detection_height / 2)

            #         # Extract the depth values from the rectangle region
            #         depth_values = self.depth_image[top_left_y:bottom_right_y, top_left_x:bottom_right_x]

            #         # Calculate the mean depth value
            #         mean_depth = np.mean(depth_values) # * self.depth_scale

            #         transform = self.tf_buffer.lookup_transform("odom", "base_footprint", rospy.Time())
            #         translation = transform.transform.translation
            #         rotation = transform.transform.rotation

            #         euler_angles = tf.transformations.euler_from_quaternion(
            #             [rotation.x, rotation.y, rotation.z, rotation.w]
            #         )
            #         theta = euler_angles[2]  # Get the yaw angle

            #         x_distance = translation.x + (mean_depth * cos(theta)) / 1000
            #         y_distance = translation.y + (mean_depth * sin(theta)) / 1000

            #         pose_msg = PoseStamped()
            #         pose_msg.header.stamp = rospy.Time.now()
            #         pose_msg.header.frame_id = "map"
            #         pose_msg.pose.position.x = x_distance
            #         pose_msg.pose.position.y = y_distance

            #         # useless filler values
            #         pose_msg.pose.position.z = 0
            #         pose_msg.pose.orientation = rotation

            #         # Publish the new position
            #         self.pose_publisher.publish(pose_msg)

            #         # print(f"depth: {mean_depth}")
            #         # print("---------------------------------------------------", end="\n\n")

            #         # Publish the depth of the rectangle's center
            #         self.depth_pub.publish(mean_depth)

            # cv2.imshow("detection_head",self.rgb_image)
            # cv2.waitKey(1)

        except Exception as e:
            rospy.logerr('Error processing image: {}'.format(e))

    def hand_cam_callback(self, msg):
        try: 
            # the hand camera is rotated 90deg to the right, rectify first then use
            
            # if(rospy.Time.now() - msg.header.stamp < rospy.Duration(4)):
                self.hand_ready = False
                bombed_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")
                self.hand_image = cv2.rotate(bombed_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
                self.hand_image = cv2.cvtColor(self.hand_image,cv2.COLOR_RGB2BGR)
                self.hand_ready = True
                self.detected = False
    

        except Exception as e: 
            rospy.logerr(f'Hand cam error: {e}')


    def depth_image_callback(self, msg):
        # Convert the depth image message to a numpy array
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.unused_depth = True
    
    def set_detection_target(self, target):
        self.detection_target = target.data
        rospy.loginfo("target set to %s", self.detection_target)

if __name__ == '__main__':
    rospy.init_node('object_detection_node')
    node = ObjectDetectionNode()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if node.hand_ready and not node.detected:
            try:
                target_class = int(name_to_index.get(node.detection_target))
                predictions = node.model.predict(node.hand_image, conf=0.1, classes=target_class, verbose=False, max_det=1)
                rospy.loginfo("Hand cam detection...")
                prediction = predictions[0]

                for box in prediction.boxes:
                    cls_name = index_to_name.get(int(box.cls[0].item()))
                    conf = round(box.conf[0].item(), 2)
                    box_dims = [int(x) for x in box.xywh[0].tolist()]
                    box_debug = [int(x) for x in box.xyxy[0].tolist()]

                    detection = Int32MultiArray()
                    detection.data = box_dims[:4]
                    cv2.rectangle(node.hand_image, (box_debug[0], box_debug[1]), (box_debug[2], box_debug[3]), (255, 0, 0))
                    node.hand_pub.publish(detection)

                cv2.imshow("detection_hand", node.hand_image)
                if cv2.waitKey(1) == ord('c'):
                    cv2.destroyAllWindows()
                    break

                node.detected = True
            except Exception as e:
                rospy.logerr(f"Detection error: {e}")
        rate.sleep()

