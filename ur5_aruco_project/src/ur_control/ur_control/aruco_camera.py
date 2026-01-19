import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ArucoCameraPublisher(Node):
    def __init__(self):
        super().__init__('aruco_camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.br = CvBridge()
        self.get_logger().info("Camera Node Started (Raw Mode)")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            try:
                msg = self.br.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "camera_frame"
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def __del__(self):
        if self.cap.isOpened(): self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()