# ARCS WATCHER – Object Detection System (Vision Subsystem)

Real-time YOLO-based object detection for the ARCS WATCHER project.

This repository contains the Vision subsystem for WATCHER.  
It runs YOLOv8 object detection on ZED camera frames (live or from SVO recordings) and publishes detection messages (bounding boxes, class IDs, confidence scores) to ROS 2 topics used by the Depth, Tracking, and Mapping subsystems.

---

## Overview

**Pipeline:**  
ZED Camera / SVO Files → YOLO Detection → `/yolo_detections` → Downstream Modules

**Frameworks & Tools:**
- ROS 2 Jazzy  
- YOLOv8 (Ultralytics)
- Docker (CPU-based for development)  
- OpenCV (for SVO playback)

**Primary Output:**  
- `/yolo_detections` → `vision_msgs/Detection2DArray`  
- `/yolo_annotated` → Annotated images with bounding boxes  
- Bounding boxes, class IDs, confidence scores  

---

## System Architecture

### Input  
- **Live:** `/zed/zed_node/rgb/image_rect_color` – ZED2 / ZED2i RGB frames  
- **Recorded:** `/camera/rgb/image` – SVO file playback  

### Output  
`/yolo_detections` (Detection2DArray), consumed by:  
- **Depth Sensing** → Adds 3D coordinates  
- **Tracking** → Tracks objects across frames  
- **Mapping** → Builds global spatial map  

---

## Data Flow Diagram
~~~~
      ┌────────────────┐
      │   ZED Camera   │
      │  or SVO File   │
      └───────┬────────┘
              │  /camera/rgb/image
              │
      ┌───────▼──────────────────────┐
      │      YOLO Detector Node       │
      │   (YOLOv8 + ROS2 Jazzy)       │
      └───────┬──────────────────────┘
              │  publishes:
              │  /yolo_detections
              │  (Detection2DArray)
              │  /yolo_annotated
              │
   ┌──────────▼─────────────┐
   │   Depth Sensing Node    │
   │ (Detections + depth →   │
   │      3D positions)      │
   └──────────┬──────────────┘
              │  publishes:
              │  /object_positions_3d
              │
   ┌──────────▼─────────────┐
   │     Tracking Node       │
   │  (Track objects across  │
   │         frames)         │
   └──────────┬──────────────┘
              │  publishes:
              │  /object_tracks
              │
   ┌──────────▼─────────────┐
   │     Mapping Node        │
   │   (Global environment   │
   │         map)            │
   └─────────────────────────┘
~~~~

---

## 🚀 For Teammates: Quick Start with Recorded Data

### Step 1: Clone Repository
~~~~bash
git clone https://github.com/amirgshabo/watcher_object_detection.git
cd watcher_object_detection
~~~~

### Step 2: Download Rosbag Dataset  
See `ros2_jazzy_ws/rosbags/README.md` for Google Drive download link.

Dataset includes:
- 26 seconds of HD street scenes (1920x1080)
- 221 camera images
- 102 YOLO detections (cars, pedestrians, traffic)
- 116 annotated visualizations  

### Step 3: Extract Rosbag
~~~~bash
unzip street_detections.zip
mv street_detections ros2_jazzy_ws/rosbags/
~~~~

### Step 4: Play the Rosbag
~~~~bash
ros2 bag play ros2_jazzy_ws/rosbags/street_detections --loop
~~~~

### Step 5: Subscribe to Topics
~~~~bash
ros2 topic echo /yolo_detections
ros2 topic echo /camera/rgb/image
ros2 topic echo /yolo_annotated
~~~~

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/rgb/image` | sensor_msgs/Image | Raw RGB camera frames |
| `/yolo_detections` | vision_msgs/Detection2DArray | Object detections |
| `/yolo_annotated` | sensor_msgs/Image | Annotated images |

---

## Requirements

### For Using Recorded Data
- Ubuntu 24.04 + ROS2 Jazzy  
- ~4 GB disk  
- No hardware required  

### For Live Development
- Docker Desktop  
- ZED2 / ZED2i  
- SVO files  

---

# 🛠️ Vision Team Development Guide

## A. Practice at Home (Recommended)

### Windows Setup

#### Step 1: Clone Repo
~~~~bash
git clone https://github.com/amirgshabo/watcher_object_detection.git
cd watcher_object_detection
git config core.autocrlf false
git rm --cached -r .
git reset --hard
~~~~

#### Step 2: Build Docker Image
~~~~bash
cd docker
docker build -f jazzy-dev.Dockerfile -t arcs_jazzy_ready .
cd ..\ros2_jazzy_ws
~~~~

#### Step 3: Start Container
~~~~bash
docker run -it --rm -v "${PWD}:/ros2_ws" --name arcs_dev arcs_jazzy_ready bash
~~~~

#### Step 4: Setup Environment
~~~~bash
sed -i 's/\r$//' /ros2_ws/start.sh
chmod +x /ros2_ws/start.sh
/ros2_ws/start.sh

pip3 install ultralytics --break-system-packages
pip3 install "numpy<2" --break-system-packages

<<<<<<< HEAD
colcon build --packages-select arcs_yolo_test arcs_yolo_detector
source install/setup.bash
~~~~

#### Step 5: Run Pipeline

**SVO Publisher**
~~~~bash
ros2 run arcs_yolo_test svo_publisher_opencv --ros-args -p svo_file:=/ros2_ws/test_data/ZED2_HD1080_Street_H264.svo
~~~~

**YOLO Detector**
~~~~bash
ros2 run arcs_yolo_detector yolo_detector --ros-args -p input_topic:=/camera/rgb/image
~~~~

**Verify Output**
~~~~bash
ros2 topic echo /yolo_detections
~~~~
=======
# Running the System

## A. Full System (Jetson + ZED)

### Terminal 1 – Start ZED Camera
```
docker start -ai arcs_ready
/ros2_ws/start.sh
ros2 launch zed_wrapper zed2.launch.py
```

### Terminal 2 – Run YOLO Detector
```
docker exec -it arcs_ready bash
/ros2_ws/start.sh
source install/setup.bash
ros2 run arcs_yolo_detector yolo_onnx_node
```

### Terminal 3 – Monitor Detections
```
docker exec -it arcs_ready bash
/ros2_ws/start.sh
source install/setup.bash
ros2 topic echo /yolo_detections
```

---

# B. Practice at Home (No Jetson / No ZED Required)

This allows you to test the ROS2 environment and message flow without physical hardware.

---

##  Windows Setup

### Prerequisites
- Docker Desktop with WSL 2 backend enabled
- Git for Windows
- PowerShell

### Step 1: Clone and Configure Repository
```
git clone https://github.com/amirgshabo/arcs_vision_temp.git
cd arcs_vision_temp
git config core.autocrlf false
git rm --cached -r .
git reset --hard
```

### Step 2: Build Docker Image (One-time setup)
```
cd docker
docker build -f jazzy-dev.Dockerfile -t arcs_jazzy_ready .
cd ..\ros2_jazzy_ws
```

This takes 5-15 minutes depending on your internet speed.

### Step 3: Start Container
```
docker run -it --rm -v "${PWD}:/ros2_ws" --name arcs_dev arcs_jazzy_ready bash
```

### Step 4: Setup Environment (Inside Container)
```
sed -i 's/\r$//' /ros2_ws/start.sh
chmod +x /ros2_ws/start.sh
/ros2_ws/start.sh
colcon build --packages-select arcs_yolo_test arcs_yolo_detector
source install/setup.bash
```

### Step 5: Run Dummy Publisher (Terminal 1)
```
ros2 run arcs_yolo_test yolo_dummy_pub
```
Keep this terminal running.

### Step 6: Monitor Output (Terminal 2 - new PowerShell window)
```
docker exec -it arcs_dev bash
/ros2_ws/start.sh
source install/setup.bash
ros2 topic echo /fake_yolo_detections
```

You should see detection messages streaming! �

---

##  Linux / Mac Setup

### Step 1: Clone Repository
```
git clone https://github.com/amirgshabo/arcs_vision_temp.git
cd arcs_vision_temp
```

### Step 2: Build Docker Image
```
cd docker
docker build -f jazzy-dev.Dockerfile -t arcs_jazzy_ready .
cd ../ros2_jazzy_ws
```
### Step 3: Start Container
```
docker run -it --rm -v $(pwd):/ros2_ws --name arcs_dev arcs_jazzy_ready bash
```
### Step 4: Setup Environment (Inside Container)
```
/ros2_ws/start.sh
colcon build --packages-select arcs_yolo_test arcs_yolo_detector
source install/setup.bash
```

### Step 5: Run Dummy Publisher (Terminal 1)
```
ros2 run arcs_yolo_test yolo_dummy_pub
```

### Step 6: Monitor Output (Terminal 2)
```
docker exec -it arcs_dev bash
/ros2_ws/start.sh
source install/setup.bash
ros2 topic echo /fake_yolo_detections
```

---

## Common Issues

**"bash: /ros2_ws/start.sh: bad interpreter"**
Windows line ending issue. Run inside container:
```
sed -i 's/\r$//' /ros2_ws/start.sh
chmod +x /ros2_ws/start.sh
```

**"docker: command not found"**
Ensure Docker Desktop is running.

**"cannot touch 'yolo_env/COLCON_IGNORE'"**
This directory may not exist in your workspace. Skip this step - it's not needed for the practice environment.

**Build fails**
Try building all packages:
```
colcon build
source install/setup.bash
```
---

## Stopping Your Work

**In Terminal 1 (dummy publisher):**
- Press Ctrl+C to stop the node
- Type exit to leave the container

**In Terminal 2 (topic echo):**
- Press Ctrl+C to stop monitoring
- Type exit to leave the container

The container auto-removes (due to --rm flag) when you exit.

---

## Resuming Work

Simply repeat Steps 3-6. The Docker image is already built, so startup is fast!

Developer can now test message flow, topic publishing, and ROS2 environment.
>>>>>>> 9a18e3ef56778c479f6d10271f408ebb9c1acfe4

---

## Full System (Jetson + Live ZED Camera)

### Start ZED Wrapper
~~~~bash
ros2 launch zed_wrapper zed2.launch.py
~~~~

### Start YOLO
~~~~bash
ros2 run arcs_yolo_detector yolo_detector --ros-args -p input_topic:=/zed/zed_node/rgb/image_rect_color
~~~~

---

## Troubleshooting

### Numpy crash
~~~~bash
pip3 uninstall numpy -y
pip3 install "numpy<2"
~~~~

### Bad interpreter
~~~~bash
sed -i 's/\r$//' /ros2_ws/start.sh
chmod +x /ros2_ws/start.sh
~~~~

---

## Repository Structure
~~~~
watcher_object_detection/
├── docker/
├── ros2_jazzy_ws/
│   ├── src/
│   ├── rosbags/
│   ├── test_data/
│   └── start.sh
└── README.md
~~~~

---

## Contributing

1. `git checkout -b feature/your-feature`  
2. Make changes  
3. Record rosbag if needed  
4. `git commit -m "message"`  
5. `git push`  
6. Create Pull Request  

---

## Project Information

Team: ARCS WATCHER  
Vision Lead: Amir Shabo  
Repository: https://github.com/amirgshabo/watcher_object_detection  

---

## License

This project is part of the ARCS WATCHER
