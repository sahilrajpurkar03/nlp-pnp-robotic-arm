#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()
img = np.zeros([480, 640, 3], dtype=np.uint8)


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.subscription = self.create_subscription(
            Image,
            'rgb',
            self.camera_callback,
            10
        )

    def camera_callback(self, data):
        global img
        try:
            img = bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


class YoloSubscriber(Node):

    def __init__(self):
        super().__init__('yolo_subscriber')

        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10
        )

        self.img_pub = self.create_publisher(Image, "/inference_result_cv2", 10)

    def yolo_callback(self, data):
        global img
        img_copy = img.copy()  # work on a copy to avoid threading issues

        for r in data.yolov8_inference:
            class_name = r.class_name
            points = np.array(r.coordinates).astype(np.int32).reshape([4, 2])
            cv2.polylines(img_copy, [points], isClosed=True, color=(0, 255, 0), thickness=2)

        try:
            img_msg = bridge.cv2_to_imgmsg(img_copy)
            self.img_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")


def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()
    yolo_subscriber = YoloSubscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(camera_subscriber)
    executor.add_node(yolo_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        camera_subscriber.destroy_node()
        yolo_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
