import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import message_filters
from cv_bridge import CvBridge
import numpy as np
import torch

DIST_THRESH = 3.0
LIN_KP = 0.4
ANG_KP = 0.002
MAX_LIN = 0.3
MAX_ANG = 0.6

class Perception(Node):
    def __init__(self):
        super().__init__('perception')
        self.bridge = CvBridge()

        img_sub = message_filters.Subscriber(self, Image, '/camera_face/image')
        depth_sub = message_filters.Subscriber(self, Image, '/camera_face/depth')
        caminfo_sub = message_filters.Subscriber(self, CameraInfo, '/camera_face/camera_info')

        sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, depth_sub, caminfo_sub], queue_size=10, slop=0.1
        )
        sync.registerCallback(self.callback)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bark_pub = self.create_publisher(String, '/bark', 10)

        self.get_logger().info("loading YOLO model")
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5s", pretrained=True)

        self.good_food = {"apple", "banana", "pizza"}
        self.bad_food = {"trash", "pill"}

        self.already_barked = False

    def callback(self, img_msg, depth_msg, cammsg):
        img = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        h, w = img.shape[:2]

        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)

        results = self.model(img[:, :, ::-1])
        det = results.pandas().xyxy[0]

        target = None
        best_conf = 0.0
        for _, row in det.iterrows():
            label = row["name"]
            conf = float(row["confidence"])
            if label in self.good_food and conf > best_conf:
                target = row
                best_conf = conf

        if target is None:
            self.explore()
            return

        xmin, ymin, xmax, ymax = map(int, [target.xmin, target.ymin, target.xmax, target.ymax])
        cx = (xmin + xmax) // 2

        roi = depth[ymin:ymax, xmin:xmax]
        vals = roi[np.isfinite(roi)]
        vals = vals[vals > 0]
        if len(vals) == 0:
            self.explore()
            return
        dist = float(np.median(vals))

        centered = (cx >= w/3 and cx <= 2*w/3)

        if not self.already_barked:
            self.control(dist, cx, w, centered)

        if dist <= DIST_THRESH and centered:
            self.do_bark()

    def control(self, dist, cx, w, centered):
        twist = Twist()
        err_x = cx - (w / 2)
        twist.angular.z = float(np.clip(err_x * -ANG_KP, -MAX_ANG, MAX_ANG))
        if dist > 0.6:
            twist.linear.x = float(np.clip((dist - 0.6) * LIN_KP, 0, MAX_LIN))
        self.cmd_pub.publish(twist)

    def do_bark(self):
        if self.already_barked:
            return
        self.already_barked = True
        for _ in range(5):
            msg = String()
            msg.data = "bark"
            self.bark_pub.publish(msg)
            self.get_logger().info("BARK!")
            rclpy.sleep(1.0)
        stop = Twist()
        self.cmd_pub.publish(stop)
        self.get_logger().info("Robot stopped")

    def explore(self):
        twist = Twist()
        twist.angular.z = 0.3
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = Perception()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
