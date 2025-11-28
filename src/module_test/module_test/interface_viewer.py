from __future__ import annotations

import threading
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


class InterfaceViewer(Node):
    """Display annotated detection image plus textual metadata."""

    def __init__(self) -> None:
        super().__init__("interface_viewer")

        self.declare_parameter("image_topic", "/camera/detections/image")
        self.declare_parameter("object_topic", "/detections/labels")
        self.declare_parameter("distance_topic", "/detections/distance")
        self.declare_parameter("speech_topic", "/robot_dog/speech")
        self.declare_parameter("window_title", "Module Test Viewer")
        self.declare_parameter("timer_period", 0.05)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        object_topic = self.get_parameter("object_topic").get_parameter_value().string_value
        distance_topic = self.get_parameter("distance_topic").get_parameter_value().string_value
        speech_topic = self.get_parameter("speech_topic").get_parameter_value().string_value
        self._window_title = self.get_parameter("window_title").get_parameter_value().string_value
        timer_period = self.get_parameter("timer_period").get_parameter_value().double_value

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self._latest_image: Optional[np.ndarray] = None
        self._latest_objects: List[str] = []
        self._latest_distance: Optional[float] = None
        self._latest_speech: Optional[str] = None

        self.create_subscription(Image, image_topic, self._image_callback, 10)
        self.create_subscription(String, object_topic, self._object_callback, 10)
        self.create_subscription(Float32, distance_topic, self._distance_callback, 10)
        self.create_subscription(String, speech_topic, self._speech_callback, 10)

        self._timer = self.create_timer(timer_period, self._render_frame)

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert image: {exc}")
            return
        with self._lock:
            self._latest_image = frame

    def _object_callback(self, msg: String) -> None:
        data = msg.data.strip()
        objects = [obj.strip() for obj in data.replace(";", ",").split(",") if obj.strip()] if data else []
        with self._lock:
            self._latest_objects = objects

    def _distance_callback(self, msg: Float32) -> None:
        with self._lock:
            self._latest_distance = float(msg.data)

    def _speech_callback(self, msg: String) -> None:
        with self._lock:
            self._latest_speech = msg.data.strip() or None

    def _render_frame(self) -> None:
        with self._lock:
            frame = np.copy(self._latest_image) if self._latest_image is not None else None
            objects = list(self._latest_objects)
            distance = self._latest_distance
            speech = self._latest_speech

        if frame is None:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)

        height, width = frame.shape[:2]
        left_line = int(width * 0.2)
        right_line = int(width * 0.8)

        cv2.line(frame, (left_line, 0), (left_line, height), (0, 255, 255), 2)
        cv2.line(frame, (right_line, 0), (right_line, height), (0, 255, 255), 2)

        info_lines = [
            f"Detected objects: {', '.join(objects) if objects else 'None'}",
            f"Distance to object: {distance:.2f} m" if distance is not None else "Distance to object: Unknown",
            f"Robot dog says: {speech if speech else 'None'}",
        ]

        padding = 10
        line_height = 25
        box_height = line_height * len(info_lines) + padding * 2
        y_start = height - box_height - 10
        y_start = max(y_start, 10)

        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (10, y_start),
            (width - 10, y_start + box_height),
            (0, 0, 0),
            thickness=-1,
        )
        cv2.addWeighted(overlay, 0.4, frame, 0.6, 0, frame)

        y_text = y_start + padding + 15
        for line in info_lines:
            cv2.putText(frame, line, (20, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            y_text += line_height

        cv2.imshow(self._window_title, frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Exit requested by user.")
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = InterfaceViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


