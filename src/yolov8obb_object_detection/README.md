# YOLOv8 OBB Model Training with Pick-and-Place Integration  

This repository provides step-by-step guidance on training a **YOLOv8 Oriented Bounding Box (OBB) model** with a custom dataset and integrating it into a **pick-and-place robotic system**.  

Reference: [Robot Mania YouTube Video](https://www.youtube.com/watch?v=7n6gCqC075g)  

---

## Dependencies  

Make sure you have the following installed:  
1. [Isaac Sim](https://developer.nvidia.com/isaac-sim)  
2. [ROS2 Humble](https://docs.ros.org/en/humble/index.html)  
3. [RViz2](https://index.ros.org/r/rviz/)  
4. Required Python dependencies:  
   ```bash
   pip install -r requirements.txt
   ```

---

## 1. Data Collection  

1. Open **Isaac Sim**.  
2. Load the `SPARC.usd` environment and start the simulation.  
   - (Optional) You may add custom objects into the provided tray.  
3. Run the data collection script:  
   ```bash
   python3 yolov8obb_object_detection/data_collection.py
   ```  
4. Press **Enter** to capture dataset images.  
   - Vary the object’s position and lighting conditions to improve dataset diversity.  

---

## 2. Data Annotation  

1. Clone the **labelImg2** repository:  
   ```bash
   git clone https://github.com/chinakook/labelImg2
   ```  
2. Run the annotation tool:  
   ```bash
   python3 labelImg2/labelImg.py
   ```  
3. Edit `labelImg2/data/predefined_classes.txt` to add your custom object names.  
4. Open the collected dataset directory in LabelImg2:  
   - `File -> Open Dir -> Select Folder`  
5. Annotate each image using **RotatedRBox**.  
   - Labels will be saved in **.xml format**.  

![annotation example](annotation.png)  

---

## 3. Data Formatting  

YOLOv8 OBB requires labels in the following format:  

```
class_index x1 y1 x2 y2 x3 y3 x4 y4
```

(Source: [Ultralytics OBB Format](https://docs.ultralytics.com/datasets/obb/#yolo-obb-format))  

### Step 1: Convert XML → DOTA Format  
```bash
git clone https://github.com/ultralytics/yolov5-utils
python3 yolov5-utils/voc2yolo5_obb.py --path training_data/ --class-file labelImg2/data/predefined_classes.txt
```  

### Step 2: Convert DOTA → YOLOv8 OBB Format  
1. Edit `yolov8obb_training/format_converter.py`  
   - Update the **class mapping** (keep the same order as `predefined_classes.txt`).  
2. Arrange dataset folders as follows:  
   ```
   yolov8obb_training/
   ├── datasets/
   │   └── custom_dataset/
   │       ├── images/
   │       │   ├── train/
   │       │   │   ├── frame_01.png
   │       │   │   ├── frame_02.png
   │       │   │   └── ...
   │       │   └── val/
   │       │       ├── frame_10.png
   │       │       ├── frame_11.png
   │       │       └── ...
   │       └── labels/
   │           ├── train/
   │           │   ├── frame_01.txt
   │           │   ├── frame_02.txt
   │           │   └── ...
   │           └── val/
   │               ├── frame_10.txt
   │               ├── frame_11.txt
   │               └── ...
   ```  
3. Run the format converter:  
   ```bash
   python3 yolov8obb_training/format_converter.py
   ```  

---

## 4. Training the YOLOv8 OBB Model  

1. Edit the dataset configuration file:  
   - `yolov8obb_training/train_info.yaml`  
   - Update dataset paths, image/label directories, and class names.  
2. Open the training script:  
   - `yolov8obb_training/yolov8_obb_train.py`  
   - Configure **epochs** and **image size**.  
3. Start training:  
   ```bash
   python3 yolov8obb_training/yolov8_obb_train.py
   ```  
4. Training results will be saved in the **runs** directory:  
   - Best trained model:  
     ```
     runs/obb/train/weights/best.pt
     ```  

---

## 5. Using the Trained Model  

Use the `best.pt` model for object detection and integration into your pick-and-place pipeline.  

---
