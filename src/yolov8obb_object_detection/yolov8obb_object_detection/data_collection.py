import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import threading

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.current_image = None
        self.frame_count = 1
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.current_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")

    def save_image(self):
        with self.lock:
            if self.current_image is not None:
                filename = f"frame_{self.frame_count}.png"
                cv2.imwrite(filename, self.current_image)
                self.get_logger().info(f"Saved {filename}")
                self.frame_count += 1
            else:
                self.get_logger().warn("No image received yet!")

def wait_for_enter_and_save(node: DataCollector):
    node.get_logger().info("Press ENTER to save image. Press Ctrl+C to exit.")
    while rclpy.ok():
        input_str = sys.stdin.readline()
        if not rclpy.ok():
            break
        node.save_image()

def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()

    # Start a thread to listen for Enter key presses without blocking ROS spinning
    thread = threading.Thread(target=wait_for_enter_and_save, args=(data_collector,), daemon=True)
    thread.start()

    try:
        rclpy.spin(data_collector)
    except KeyboardInterrupt:
        pass

    data_collector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
