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

# Start cmd_bridge in background
python3 cmd_bridge.py &
BRIDGE_PID=$!

# Wait a few seconds for FastAPI server to come up
sleep 3

# Open Firefox in mobile-like window
firefox --new-window "http://localhost:8000" --width=400 --height=700 &

# Wait for cmd_bridge to exit (Ctrl+C)
wait $BRIDGE_PID

# Cleanup when cmd_bridge exits
kill $TP_PID
kill $YOLO_PID
