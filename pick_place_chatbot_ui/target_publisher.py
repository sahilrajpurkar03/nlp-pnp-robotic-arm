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
        self.target_class = None   # set via /target_class_cmd
        self.target_box = None     # new: box number (1 or 2)
        self.published = False     # flag to publish only once

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

        # Camera intrinsics (from Isaac Sim)
        self.fx = 1280.0
        self.fy = 1280.0
        self.cx = 959.59595
        self.cy = 647.61407

        # Camera extrinsics (position in robot base frame)
        self.C = np.array([0.46076, -0.03, 1.25071])  # (x, y, z)

        # Rotation matrix: camera frame → robot base frame
        self.R = np.array([
            [0, -1, 0],
            [-1, 0, 0],
            [0, 0, -1]
        ])

        # Manual calibration offset (meters)
        self.offset_x = -0.28
        self.offset_y = -0.31

    def cmd_callback(self, msg: String):
        """Parse incoming command message."""
        parts = msg.data.strip().split(",")
        self.target_class = parts[0]
        self.target_box = parts[1] if len(parts) > 1 else None
        self.published = False  # reset flag when target changes

        if self.target_box:
            self.get_logger().info(f"Target set: {self.target_class}, box {self.target_box}")
        else:
            self.get_logger().info(f"Target set: {self.target_class}, box not specified")

    def pixel_to_world(self, u, v):
        """Convert pixel (u,v) to world (x,y) in robot base frame (table z=0)."""
        xn = (u - self.cx) / self.fx
        yn = (v - self.cy) / self.fy
        ray_cam = np.array([xn, yn, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)

        ray_base = self.R @ ray_cam
        t = -self.C[2] / ray_base[2]
        P = self.C + t * ray_base

        return P[0], P[1]  # (x,y) in robot base

    def yolo_callback(self, data: Yolov8Inference):
        if not self.target_class or self.published:
            return

        for detection in data.yolov8_inference:
            if detection.class_name != self.target_class:
                continue

            # bbox center
            points = np.array(detection.coordinates, dtype=float).reshape([4, 2])
            middle_point = np.mean(points, axis=0)
            u, v = middle_point

            # Pixel -> world
            x, y = self.pixel_to_world(u, v)
            x += self.offset_x
            y += self.offset_y

            # Orientation (rough from bbox)
            dist1 = math.hypot(points[0][0] - points[1][0], points[0][1] - points[1][1])
            dist2 = math.hypot(points[1][0] - points[2][0], points[1][1] - points[2][1])
            if dist1 > dist2:
                denom = points[0][0] - points[1][0]
                angle = math.pi / 2 if denom == 0 else math.atan2(points[0][1] - points[1][1], denom)
            else:
                denom = points[1][0] - points[2][0]
                angle = math.pi / 2 if denom == 0 else math.atan2(points[1][1] - points[2][1], denom)

            angle = math.pi/2 + angle

            # Publish [x, y, yaw, box]
            msg = Float64MultiArray()
            msg.data = [
                round(x, 4),
                round(y, 4),
                round(angle, 4),
                float(self.target_box) if self.target_box else -1.0  # -1 if box not set
            ]
            self.pub.publish(msg)

            self.get_logger().info(
                f'Published target: class={self.target_class}, box={self.target_box}, '
                f'data={msg.data}'
            )
            self.published = True
            break


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
