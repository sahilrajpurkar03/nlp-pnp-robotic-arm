#! /usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from yolov8_msgs.msg import Yolov8Inference
import threading

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.target_class = None  # Initially no class selected

        # ROS2 subscriber and publisher
        self.sub = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10
        )
        self.pub = self.create_publisher(Float64MultiArray, '/target_point', 10)

        # Camera parameters
        self.fx = 1920.0
        self.fy = 1246.15
        self.cx = 960
        self.cy = 540
        self.z = 0.7        # depth of the object of the camera
        self.init_x = 0.59066 #-3.2  # offset from the base of the robot: -0.59066
        self.init_y = 0.03 #-0.03 # offset from the base of the robot: 0.03

    def yolo_callback(self, data):
        if self.target_class is None:
            return  # No class selected yet

        for detection in data.yolov8_inference:
            if detection.class_name != self.target_class:
                continue

            # Get bounding box points
            points = np.array(detection.coordinates).astype(np.int32).reshape([4, 2])
            middle_point = np.sum(points, axis=0) / 4

            # Convert to real-world coordinates
            x = -self.z * (middle_point[1] - self.cy) / self.fy + self.init_x
            y = -self.z * (middle_point[0] - self.cx) / self.fx + self.init_y

            # Compute orientation angle
            dist1 = math.sqrt((points[0][0] - points[1][0])**2 + (points[0][1] - points[1][1])**2)
            dist2 = math.sqrt((points[1][0] - points[2][0])**2 + (points[1][1] - points[2][1])**2)
            
            if dist1 > dist2:
                denominator = points[0][0] - points[1][0]
                angle = math.pi/2 if denominator == 0 else math.atan2(points[0][1] - points[1][1], denominator)
            else:
                denominator = points[1][0] - points[2][0]
                angle = math.pi/2 if denominator == 0 else math.atan2(points[1][1] - points[2][1], denominator)

            angle = math.pi/2 - angle

            # Publish target point
            msg = Float64MultiArray(data=[x, y, angle])
            self.pub.publish(msg)
            self.get_logger().info(f'Published target for {self.target_class}: {msg.data}')

def input_thread(node):
    while True:
        new_class = input("Enter target class: ").strip()
        node.target_class = new_class
        node.get_logger().info(f"Target class set to: {new_class}")

def main():
    rclpy.init()
    node = TargetPublisher()

    # Start a separate thread for user input
    thread = threading.Thread(target=input_thread, args=(node,), daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
