# FrankaChatPicker


## Commands used:

ros2 launch my_panda_moveit_config demo.launch.py 

ros2 run yolov8obb_object_detection data_collector.py 




Conda env for below: ros2_humble_py310

ros2 run yolov8obb_object_detection yolov8_obb_subscriber

ros2 run yolov8obb_object_detection yolov8_obb_publisher 

ros2 launch yolov8obb_object_detection yolov8_obb_launch.py


dependencies

moveit package
install franka packages
install ultralytics
ros-humble-ros2-controlros-humble-ros2-controllers
ros-humble-gripper-controllers
ros-humble-moveit-py




## Code to extract the fx, fy, cx, cy from Isaac sim 

```python
from pxr import UsdGeom
import omni.usd

# Get the current stage
stage = omni.usd.get_context().get_stage()

# Camera prim path
camera_prim_path = "/World/detection_camera/Camera_SG2_OX03CC_5200_GMSL2_H60YA"
camera_prim = stage.GetPrimAtPath(camera_prim_path)
camera = UsdGeom.Camera(camera_prim)

# Camera parameters from USD
focal_length_mm = camera.GetFocalLengthAttr().Get()              # focal length in mm
horizontal_aperture_mm = camera.GetHorizontalApertureAttr().Get()  # sensor width in mm
vertical_aperture_mm = camera.GetVerticalApertureAttr().Get()      # sensor height in mm

# Image resolution (your render resolution)
width = 1920
height = 1080

# Convert focal length to pixels
fx = (focal_length_mm / horizontal_aperture_mm) * width
fy = (focal_length_mm / vertical_aperture_mm) * height

# Principal point (assume image center)
cx = width / 2
cy = height / 2

print(f"fx={fx}, fy={fy}, cx={cx}, cy={cy}")

# demo launch for panda and motion planning in rviz isaacsim
1. open isaacsim and our .usd and start simulation
2. launch rviz : ros2 launch panda_moveit_config demo.launch.py 
3. ros2 run panda_moveit_config move_arm_to_xyz 0.0 0.0 0.0 (z is hardcoded in code)
```

## Step perform for chatbot
```bash
# install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# pull a small instruct model
ollama pull phi3
```
target_publisher.py    # ROS2 node (same as before)
cmd_bridge.py          # FastAPI + LLM/Ros bridge
llm_mapper.py          # Ollama / transformers wrapper

for executing chatbot
```bash
./pick_place_chatbot_ui/launch.sh 
```