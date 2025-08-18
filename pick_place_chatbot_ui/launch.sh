#!/bin/bash
# launch.sh

# Activate ROS 2 environment
source /opt/ros/humble/setup.bash   # or galactic/foxy depending on your ROS2 distro

# Go into the script directory
cd "$(dirname "$0")"

# Run target_publisher in background
python3 target_publisher.py &
TP_PID=$!

# Run yolov8_obb_publisher in background
ros2 run yolov8obb_object_detection yolov8_obb_publisher &
YOLO_PID=$!

# Run cmd_bridge in foreground (so Ctrl+C stops both)
python3 cmd_bridge.py

# Cleanup when cmd_bridge exits
kill $TP_PID
kill $YOLO_PID
