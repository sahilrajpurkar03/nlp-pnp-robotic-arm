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
        self.offset_x = -0.26
        self.offset_y = -0.30

    def cmd_callback(self, msg: String):
        self.target_class = msg.data.strip()
        self.published = False  # reset flag when target class changes
        self.get_logger().info(f"Target class set to: {self.target_class}")

    def pixel_to_world(self, u, v):
        """Convert pixel (u,v) to world (x,y) in robot base frame (table z=0)."""

        # 1. pixel to normalized camera ray
        xn = (u - self.cx) / self.fx
        yn = (v - self.cy) / self.fy
        ray_cam = np.array([xn, yn, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)

        # 2. transform ray to base frame
        ray_base = self.R @ ray_cam

        # 3. intersect with table plane z=0
        t = -self.C[2] / ray_base[2]
        P = self.C + t * ray_base

        return P[0], P[1]  # (x,y) in robot base

    def yolo_callback(self, data: Yolov8Inference):
        if not self.target_class or self.published:
            return

        for detection in data.yolov8_inference:
            if detection.class_name != self.target_class:
                continue

            # Get bbox points -> center
            points = np.array(detection.coordinates, dtype=float).reshape([4, 2])
            middle_point = np.mean(points, axis=0)
            u, v = middle_point

            # Pixel -> world coordinates
            x, y = self.pixel_to_world(u, v)

            # Apply manual correction offset
            x += self.offset_x
            y += self.offset_y

            # Orientation (rough estimation from bbox edges)
            dist1 = math.hypot(points[0][0] - points[1][0], points[0][1] - points[1][1])
            dist2 = math.hypot(points[1][0] - points[2][0], points[1][1] - points[2][1])
            if dist1 > dist2:
                denom = points[0][0] - points[1][0]
                angle = math.pi / 2 if denom == 0 else math.atan2(points[0][1] - points[1][1], denom)
            else:
                denom = points[1][0] - points[2][0]
                angle = math.pi / 2 if denom == 0 else math.atan2(points[1][1] - points[2][1], denom)
            
            # ✅ Add yaw offset (60° = pi/3 radians)
            angle -= math.radians(60)

            # Normalize angle to [-pi, pi]
            angle = (angle + math.pi) % (2 * math.pi) - math.pi

            # Publish once
            msg = Float64MultiArray(
                data=[
                    round(x, 2),
                    round(y, 2),
                    round(angle, 2)
                ]
            )
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
