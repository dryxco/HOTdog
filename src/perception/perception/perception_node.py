#!/usr/bin/env python3
"""
ROS2 Perception Node
- Subscribes to RGB and Depth images
- Performs object detection using YOLOv8
- Publishes detection results, distance, and bark commands
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque

from ament_index_python.packages import get_package_share_directory
import os

# Try to import ultralytics for YOLOv8
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("Warning: ultralytics not installed. Run: pip install ultralytics")


class PerceptionNode(Node):
    """Perception node for object detection and distance publishing."""

    # Edible objects that trigger bark
    EDIBLE_CLASSES = ['apple', 'pizza']

    def __init__(self):
        super().__init__('perception_node')

        # Parameters
        self.declare_parameter('model_path', 'models/best_v2.pt')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('distance_threshold', 3.0)
        self.declare_parameter('center_region_ratio', 0.6)

        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.center_ratio = self.get_parameter('center_region_ratio').value
        model_path_param = self.get_parameter('model_path').value

        pkg_share = get_package_share_directory('perception')
        self.model_path = model_path_param if os.path.isabs(model_path_param) else os.path.join(pkg_share, model_path_param)

        # CV Bridge
        self.bridge = CvBridge()

        # YOLO
        self._init_yolo_model()

        # Buffers
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        self.distance_buffer = deque(maxlen=5)

        # QoS
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.rgb_sub = self.create_subscription(Image, '/camera_top/image', self.rgb_callback, sensor_qos)
        self.depth_sub = self.create_subscription(Image, '/camera_top/depth', self.depth_callback, sensor_qos)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera_top/camera_info', self.camera_info_callback, sensor_qos)
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/camera_top/points', self.pointcloud_callback, sensor_qos)

        # Publishers
        self.detection_image_pub = self.create_publisher(Image, '/camera/detections/image', 10)
        self.labels_pub = self.create_publisher(String, '/detections/labels', 10)
        self.distance_pub = self.create_publisher(Float32, '/detections/distance', 10)
        self.speech_pub = self.create_publisher(String, '/robot_dog/speech', 10)
        self.bbox_center_pub = self.create_publisher(Float32, '/detections/target_bbox_center_x', 10)

        # Timer
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

        self.get_logger().info('Perception node initialized')
        self.get_logger().info(f'Distance threshold: {self.distance_threshold}m')
        self.get_logger().info(f'Center region ratio: {self.center_ratio}')

    def _init_yolo_model(self):
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'YOLO model loaded: {self.model_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.model = None
        else:
            self.model = None
            self.get_logger().warn('YOLO not available, using mock detection')

    # --- Callbacks ---
    def rgb_callback(self, msg: Image):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')

    def depth_callback(self, msg: Image):
        try:
            if msg.encoding == '32FC1':
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
            elif msg.encoding == '16UC1':
                depth_raw = self.bridge.imgmsg_to_cv2(msg, '16UC1')
                self.depth_image = depth_raw.astype(np.float32) / 1000.0
            else:
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                if self.depth_image.dtype == np.uint16:
                    self.depth_image = self.depth_image.astype(np.float32) / 1000.0
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def pointcloud_callback(self, msg: PointCloud2):
        pass

    # --- Utilities ---
    def get_distance_at_bbox(self, depth_img: np.ndarray, bbox: tuple) -> float:
        x1, y1, x2, y2 = map(int, bbox)
        h, w = depth_img.shape[:2]
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return float('inf')
        cx, cy = (x1 + x2)//2, (y1 + y2)//2
        bw, bh = (x2 - x1)//4, (y2 - y1)//4
        roi_x1, roi_x2 = max(0, cx-bw), min(w, cx+bw)
        roi_y1, roi_y2 = max(0, cy-bh), min(h, cy+bh)
        depth_roi = depth_img[roi_y1:roi_y2, roi_x1:roi_x2]
        valid_depths = depth_roi[(depth_roi>0.1)&(depth_roi<10.0)&np.isfinite(depth_roi)]
        if len(valid_depths)==0:
            return float('inf')
        return float(np.median(valid_depths))

    def is_centered(self, bbox: tuple, img_width: int) -> bool:
        x1, _, x2, _ = bbox
        bbox_center_x = (x1 + x2)/2
        left_boundary = img_width*(1.0-self.center_ratio)/2
        right_boundary = img_width*(1.0+self.center_ratio)/2
        return left_boundary <= bbox_center_x <= right_boundary

    def is_edible(self, class_name: str) -> bool:
        class_lower = class_name.lower()
        for edible in self.EDIBLE_CLASSES:
            if edible in class_lower or class_lower in edible:
                return True
        return False

    # --- Main loop ---
    def process_frame(self):
        if self.rgb_image is None:
            return

        img = self.rgb_image.copy()
        h, w = img.shape[:2]
        detections = self._run_detection(img)

        labels = []
        min_distance = float('inf')
        should_bark = False
        detected_center_x = -1.0
        bbox_center_msg = Float32()

        for i, det in enumerate(detections):
            bbox = det['bbox']
            class_name = det['class']

            # bbox 중심
            if i==0:
                x1, y1, x2, y2 = bbox
                detected_center_x = float((x1+x2)/2)

            # 거리 계산 (모든 객체)
            distance = float('inf')
            if self.depth_image is not None:
                distance = self.get_distance_at_bbox(self.depth_image, bbox)
            if distance > self.distance_threshold:
                continue
            if distance < min_distance:
                min_distance = distance

            # Edible + 중앙 + distance 조건에서만 bark
            is_edible = self.is_edible(class_name)
            is_close = distance <= self.distance_threshold
            is_in_center = self.is_centered(bbox, w)
            if is_edible and is_close and is_in_center:
                should_bark = True

            # Draw bbox + label (class name only)
            color = (0, 255, 0) if (is_edible and is_close and is_in_center) else (255, 0, 0)
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            label_text = class_name
            (text_w, text_h), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(img, (x1, y1 - text_h - 10), (x1 + text_w, y1), color, -1)
            cv2.putText(img, label_text, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            labels.append(class_name)

        # --- Publish ---
        bbox_center_msg.data = detected_center_x
        self.bbox_center_pub.publish(bbox_center_msg)

        # Guide lines
        left_line = int(w*0.2)
        right_line = int(w*0.8)
        cv2.line(img, (left_line,0),(left_line,h),(0,255,255),1)
        cv2.line(img, (right_line,0),(right_line,h),(0,255,255),1)

        # Status text
        status_text = f'Distance threshold: {self.distance_threshold}m'
        cv2.putText(img, status_text, (10,30), cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,0),2)

        # Detection image
        try:
            det_msg = self.bridge.cv2_to_imgmsg(img,'bgr8')
            self.detection_image_pub.publish(det_msg)
        except Exception as e:
            self.get_logger().error(f'Image publish error: {e}')

        # Labels
        labels_msg = String()
        labels_msg.data = ', '.join(labels) if labels else 'None'
        self.labels_pub.publish(labels_msg)

        # Distance
        distance_msg = Float32()
        if min_distance < float('inf'):
            self.distance_buffer.append(min_distance)
            distance_msg.data = float(np.median(list(self.distance_buffer)))
        else:
            distance_msg.data = -1.0
        self.distance_pub.publish(distance_msg)

        # Bark
        speech_msg = String()
        speech_msg.data = 'bark' if should_bark else 'None'
        if should_bark:
            self.get_logger().info(f'BARK! Object at {min_distance:.2f}m in center')
        self.speech_pub.publish(speech_msg)

    # --- YOLO detection ---
    def _run_detection(self, img: np.ndarray) -> list:
        detections = []
        if self.model is not None:
            try:
                results = self.model(img, conf=self.conf_threshold, verbose=False)
                for result in results:
                    boxes = result.boxes
                    if boxes is None:
                        continue
                    for box in boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        cls_id = int(box.cls[0])
                        cls_name = self.model.names[cls_id]
                        conf = float(box.conf[0])

                        detections.append({
                            'bbox': (x1, y1, x2, y2),
                            'class': cls_name,
                            'conf': conf
                        })
            except Exception as e:
                self.get_logger().error(f'Detection error: {e}')

        if len(detections) == 0:
            return []

        detections = [
            d for d in detections
            if 'eaten_pizza' not in d['class']
        ]

        return detections



def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
