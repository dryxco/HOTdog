import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import math

class RotateAroundNurse(Node):
    def __init__(self):
        super().__init__('rotate_around_nurse')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/camera/detections/image',
            self.image_callback,
            10
        )
        self.label_sub = self.create_subscription(
            String,
            '/detections/labels',
            self.label_callback,
            10
        )
        self.distance_sub = self.create_subscription(
            Float32,
            '/detections/distance',
            self.distance_callback,
            10
        )
        self.center_sub = self.create_subscription(
            Float32,
            '/detections/bbox_center',
            self.center_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()

        # FSM 상태
        self.state = "NAVIGATE"  # NAVIGATE → SEARCH → ALIGN → APPROACH → ROTATE_AROUND

        # 탐지 정보
        self.detected_label = None
        self.distance = None
        self.center_x = None

        # 현재 위치
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # 목표 위치 (방문 앞)
        self.goal_x = 2.0
        self.goal_y = 1.0

    # -----------------------------
    # Callbacks
    # -----------------------------
    def label_callback(self, msg: String):
        self.detected_label = msg.data

    def distance_callback(self, msg: Float32):
        self.distance = msg.data

    def center_callback(self, msg: Float32):
        self.center_x = msg.data

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # yaw 계산
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        nurse_visible = (self.detected_label == "nurse")

        # FSM 실행
        if self.state == "NAVIGATE":
            self.navigate_to_goal()
        elif self.state == "SEARCH":
            self.search(nurse_visible)
        elif self.state == "ALIGN":
            self.align(nurse_visible, self.center_x)
        elif self.state == "APPROACH":
            self.approach(nurse_visible, self.distance)
        elif self.state == "ROTATE_AROUND":
            self.rotate_around()

    # -----------------------------
    # 0) 목표 위치로 이동 (실제 좌표 기반)
    # -----------------------------
    def navigate_to_goal(self):
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw

        # 각도를 [-pi, pi]로 보정
        while yaw_error > math.pi:
            yaw_error -= 2*math.pi
        while yaw_error < -math.pi:
            yaw_error += 2*math.pi

        twist = Twist()

        # 회전 우선
        if abs(yaw_error) > 0.1:
            twist.angular.z = 0.5 * yaw_error
        elif distance > 0.1:
            twist.linear.x = 0.25

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Navigating: distance={distance:.2f}, yaw_error={yaw_error:.2f}")

        if distance <= 0.1:
            self.get_logger().info("Reached goal → SEARCH 단계로 이동")
            self.state = "SEARCH"
            self.cmd_pub.publish(Twist())

    # -----------------------------
    # 1) 주변 회전하며 nurse 찾기
    # -----------------------------
    def search(self, nurse_visible):
        twist = Twist()
        twist.angular.z = 0.4
        self.cmd_pub.publish(twist)
        if nurse_visible:
            self.get_logger().info("Nurse detected → ALIGN 단계로 이동")
            self.state = "ALIGN"

    # -----------------------------
    # 2) bbox center 기준 정렬
    # -----------------------------
    def align(self, nurse_visible, nurse_center_x):
        if not nurse_visible or nurse_center_x is None:
            self.state = "SEARCH"
            return

        twist = Twist()
        img_center_x = 320  # 카메라 중심
        error = nurse_center_x - img_center_x

        if abs(error) > 20:  # 허용 오차
            twist.angular.z = -0.002 * error
            self.cmd_pub.publish(twist)
        else:
            self.state = "APPROACH"
            self.cmd_pub.publish(Twist())

    # -----------------------------
    # 3) nurse까지 직진
    # -----------------------------
    def approach(self, nurse_visible, distance):
        if not nurse_visible or distance is None:
            self.state = "SEARCH"
            return

        twist = Twist()
        if distance > 1.0:
            twist.linear.x = 0.25
            self.cmd_pub.publish(twist)
        else:
            self.state = "ROTATE_AROUND"
            self.cmd_pub.publish(Twist())

    # -----------------------------
    # 4) nurse 주변 원 회전
    # -----------------------------
    def rotate_around(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.4
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RotateAroundNurse()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()