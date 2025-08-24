# object_picker/target_publisher.py
#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from yolov8_msgs.msg import Yolov8Inference

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.target_class = None  # set via /target_class_cmd
        self.published = False  # flag to publish only once

        # ROS2 subscriber and publisher
        self.sub = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10
        )
        self.cmd_sub = self.create_subscription(
            String,
            '/target_class_cmd',
            self.cmd_callback,
            10
        )
        self.pub = self.create_publisher(Float64MultiArray, '/target_point', 10)

        # Camera parameters
        self.fx = 1920.0
        self.fy = 1246.15
        self.cx = 960
        self.cy = 540
        self.z = 0.7
        self.init_x = 0.59066
        self.init_y = 0.03

    def cmd_callback(self, msg: String):
        self.target_class = msg.data.strip()
        self.published = False  # reset flag when target class changes
        self.get_logger().info(f"Target class set to: {self.target_class}")

    def yolo_callback(self, data: Yolov8Inference):
        if not self.target_class or self.published:
            return

        for detection in data.yolov8_inference:
            if detection.class_name != self.target_class:
                continue

            # Get bbox points -> center
            points = np.array(detection.coordinates).astype(np.int32).reshape([4, 2])
            middle_point = np.sum(points, axis=0) / 4

            # Pixel -> real world
            x = -self.z * (middle_point[1] - self.cy) / self.fy + self.init_x
            y = -self.z * (middle_point[0] - self.cx) / self.fx + self.init_y

            # Orientation
            dist1 = math.hypot(points[0][0]-points[1][0], points[0][1]-points[1][1])
            dist2 = math.hypot(points[1][0]-points[2][0], points[1][1]-points[2][1])
            if dist1 > dist2:
                denom = points[0][0] - points[1][0]
                angle = math.pi/2 if denom == 0 else math.atan2(points[0][1]-points[1][1], denom)
            else:
                denom = points[1][0] - points[2][0]
                angle = math.pi/2 if denom == 0 else math.atan2(points[1][1]-points[2][1], denom)
            angle = math.pi/2 - angle

            # Publish once
            msg = Float64MultiArray(data=[x, y, angle])
            self.pub.publish(msg)
            self.get_logger().info(f'Published target for {self.target_class}: {msg.data}')
            self.published = True  # mark as published
            break  # exit after first matching detection

def main():
    rclpy.init()
    node = TargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
