#!/usr/bin/env python3
"""
object_detection_node.py — ROS 2 Humble  /  YOLOv12 (ultralytics)

Upgraded from YOLOv8 to YOLOv12 (ultralytics>=8.3.0).
YOLOv12 uses the same YOLO class and predict() API as v8/v11 —
only the weights file name changes (yolo12n.pt / yolo12s.pt / etc.).

All original functionality preserved:
  - Head RGB camera subscription  (/hsrb/head_rgbd_sensor/rgb/image_raw)
  - Hand camera subscription      (/hsrb/hand_camera/image_raw)
  - Depth camera subscription     (/hsrb/head_rgbd_sensor/depth_registered/image_raw)
  - Detection target subscription (/detection_target)
  - /hand_detection publisher     (Int32MultiArray: [cx, cy, w, h])
  - /vision publisher             (PoseStamped)
  - /detection_depth publisher    (Float32)
"""

import os
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String, Int32MultiArray
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import tf2_ros
from tf_transformations import euler_from_quaternion

from ultralytics import YOLO

# ---------------------------------------------------------------------------
# Camera topics
# ---------------------------------------------------------------------------
HEAD_CAM_TOPIC  = '/hsrb/head_rgbd_sensor/rgb/image_raw'
HAND_CAM_TOPIC  = '/hsrb/hand_camera/image_raw'
DEPTH_CAM_TOPIC = '/hsrb/head_rgbd_sensor/depth_registered/image_raw'

# ---------------------------------------------------------------------------
# Class names (identical to original)
# ---------------------------------------------------------------------------
CLASS_NAMES = [
    "apple", "bear_doll", "canned_juice", "cigarette", "clock",
    "dog_doll", "empty_ketchup", "empty_plastic_bottle", "filled_ketchup",
    "filled_plastic_bottle", "game_controller", "ground_pepper", "hourglass",
    "matryoshka", "nursing_bottle", "piggy_bank", "pink_cup", "rabbit_doll",
    "rubik-s_cube", "salt", "sauce", "soysauce", "spray_bottle", "sugar",
    "toy_car", "toy_duck", "toy_penguin", "tumbler", "white_cup",
    "white_side_table",
]

NAME_TO_INDEX = {name: idx for idx, name in enumerate(CLASS_NAMES)}
INDEX_TO_NAME = {idx: name for idx, name in enumerate(CLASS_NAMES)}

# ---------------------------------------------------------------------------
# Model loading
# Bug fix: ROS2安装后脚本在 lib/rcup_vision/，模型应在 share/rcup_vision/models/
# ---------------------------------------------------------------------------
def _find_model_path() -> str:
    # 1. ament 包共享目录（ROS2 colcon install 路径）
    try:
        share_dir = get_package_share_directory('vision_ros2')
        candidate = os.path.join(share_dir, 'models', 'last.pt')
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    # 2. 脚本同层 ../models/（开发源码目录直接运行）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidate  = os.path.normpath(os.path.join(script_dir, '..', 'models', 'last.pt'))
    if os.path.isfile(candidate):
        return candidate
    # 3. 回落到 yolo12n.pt（ultralytics 自动下载预训练权重）
    return 'yolo12n.pt'

MODEL_PATH = _find_model_path()
model      = YOLO(MODEL_PATH)
print(f'[object_detection_node] Model loaded: {MODEL_PATH}')


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.rgb_image    = None
        self.hand_image   = None
        self.depth_image  = None
        self.unused_depth = False
        self.hand_ready   = False
        self.detected     = False
        self.detection_target = 'sugar'

        self.bridge = CvBridge()

        # Bug fix: headless 环境下禁用 imshow 防止崩溃
        self._has_display = bool(os.environ.get('DISPLAY', ''))

        # TF2
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS: 传感器数据 best effort, keep last 1
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriptions
        self.create_subscription(Image, HEAD_CAM_TOPIC,
            self.image_callback, sensor_qos)
        self.create_subscription(Image, HAND_CAM_TOPIC,
            self.hand_cam_callback, sensor_qos)
        self.create_subscription(Image, DEPTH_CAM_TOPIC,
            self.depth_image_callback, sensor_qos)
        self.create_subscription(String, '/detection_target',
            self.set_detection_target, 10)

        # Publishers
        self.depth_pub = self.create_publisher(Float32,         '/detection_depth', 10)
        self.pose_pub  = self.create_publisher(PoseStamped,     '/vision',          10)
        self.hand_pub  = self.create_publisher(Int32MultiArray, '/hand_detection',  10)

        self.model = model
        self.get_logger().info(
            f'ObjectDetectionNode started (YOLOv12, model={MODEL_PATH})')

    # ------------------------------------------------------------------
    # Head camera — commented-out depth+pose logic preserved from original
    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
            # Head-camera detection kept commented out (matches original structure)
            # Activate as needed:
            # target_class = NAME_TO_INDEX.get(self.detection_target)
            # predictions  = self.model.predict(self.rgb_image, conf=0.5,
            #                    classes=target_class, verbose=False, max_det=1)
            # prediction   = predictions[0]
            # for box in prediction.boxes:
            #     cls_name  = INDEX_TO_NAME.get(int(box.cls[0].item()))
            #     box_dims  = [int(x) for x in box.xywh[0].tolist()]
            #     box_debug = [int(x) for x in box.xyxy[0].tolist()]
            #     if self.unused_depth and self.depth_image is not None:
            #         self.unused_depth = False
            #         cx, cy, w, h = box_dims
            #         depth_roi  = self.depth_image[
            #             max(0,cy-h//2):cy+h//2, max(0,cx-w//2):cx+w//2]
            #         mean_depth = float(np.nanmean(depth_roi))
            #         try:
            #             tf_s = self.tf_buffer.lookup_transform(
            #                 'odom', 'base_footprint', rclpy.time.Time())
            #             tx  = tf_s.transform.translation.x
            #             ty  = tf_s.transform.translation.y
            #             rot = tf_s.transform.rotation
            #             _, _, theta = euler_from_quaternion(
            #                 [rot.x, rot.y, rot.z, rot.w])
            #             pose_msg = PoseStamped()
            #             pose_msg.header.stamp    = self.get_clock().now().to_msg()
            #             pose_msg.header.frame_id = 'map'
            #             pose_msg.pose.position.x = tx + (mean_depth * math.cos(theta)) / 1000
            #             pose_msg.pose.position.y = ty + (mean_depth * math.sin(theta)) / 1000
            #             pose_msg.pose.position.z = 0.0
            #             pose_msg.pose.orientation = rot
            #             self.pose_pub.publish(pose_msg)
            #             self.depth_pub.publish(Float32(data=mean_depth))
            #         except Exception as tf_ex:
            #             self.get_logger().warn(f'TF lookup failed: {tf_ex}')
        except Exception as e:
            self.get_logger().error(f'Error processing head image: {e}')

    # ------------------------------------------------------------------
    # Hand camera
    # ------------------------------------------------------------------
    def hand_cam_callback(self, msg: Image):
        try:
            self.hand_ready = False
            raw     = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # 手部相机旋转90度顺时针，先逆旋转矫正（与原稿一致）
            rotated = cv2.rotate(raw, cv2.ROTATE_90_COUNTERCLOCKWISE)
            self.hand_image = cv2.cvtColor(rotated, cv2.COLOR_RGB2BGR)
            self.hand_ready = True
            self.detected   = False
        except Exception as e:
            self.get_logger().error(f'Hand cam error: {e}')

    # ------------------------------------------------------------------
    # Depth camera
    # ------------------------------------------------------------------
    def depth_image_callback(self, msg: Image):
        self.depth_image  = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        self.unused_depth = True

    # ------------------------------------------------------------------
    # Detection target
    # ------------------------------------------------------------------
    def set_detection_target(self, target: String):
        self.detection_target = target.data
        self.get_logger().info(f'Detection target set to: {self.detection_target}')

    # ------------------------------------------------------------------
    # Hand-camera detection（主循环调用，对应原稿 while 循环内逻辑）
    # ------------------------------------------------------------------
    def run_hand_detection(self):
        if not (self.hand_ready and not self.detected):
            return
        try:
            target_class = NAME_TO_INDEX.get(self.detection_target)
            if target_class is None:
                self.get_logger().warn(
                    f'Unknown detection target: {self.detection_target}')
                return

            # YOLOv12 predict API — identical to v8/v11
            predictions = self.model.predict(
                self.hand_image, conf=0.1,
                classes=target_class, verbose=False, max_det=1)
            self.get_logger().info('Hand cam detection...')
            prediction = predictions[0]

            for box in prediction.boxes:
                box_dims  = [int(x) for x in box.xywh[0].tolist()]
                box_debug = [int(x) for x in box.xyxy[0].tolist()]

                detection      = Int32MultiArray()
                detection.data = box_dims[:4]  # [cx, cy, w, h]
                cv2.rectangle(
                    self.hand_image,
                    (box_debug[0], box_debug[1]),
                    (box_debug[2], box_debug[3]),
                    (255, 0, 0), 2)
                self.hand_pub.publish(detection)

            # Bug fix: headless 环境跳过 imshow
            if self._has_display:
                cv2.imshow('detection_hand', self.hand_image)
                if cv2.waitKey(1) == ord('c'):
                    cv2.destroyAllWindows()

            self.detected = True
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    # Bug fix: timeout_sec=0.1 确保 spin_once 给 clock/timer 足够时间，
    # rate.sleep() 才能正常工作（timeout_sec=0.0 会导致 rate 计时异常）
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.run_hand_detection()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
