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

##  For Teammates: Quick Start with Recorded Data

**No ZED camera or Jetson needed!** Use our pre-recorded rosbag dataset.

### Step 1: Clone Repository
~~~~bash
git clone https://github.com/amirgshabo/watcher_object_detection.git
cd watcher_object_detection
~~~~

### Step 2: Download Rosbag Dataset

** Download Link:** [Street Detections Dataset (Google Drive)](https://drive.google.com/file/d/19ERBubKotWDy918HaFgf6tWvkFrFF3Xq/view?usp=sharing)

**Dataset includes:**
- 26 seconds of HD street scenes (1920x1080)
- 221 camera images
- 102 YOLO detections (cars, pedestrians, traffic)
- 116 annotated visualizations
- File size: ~2 GB (compressed)

For more details, see [ros2_jazzy_ws/rosbags/README.md](ros2_jazzy_ws/rosbags/README.md)

### Step 3: Extract Rosbag
~~~~bash
# After downloading street_detections.zip
unzip street_detections.zip
mv street_detections ros2_jazzy_ws/rosbags/
~~~~

### Step 4: Play the Rosbag
~~~~bash
ros2 bag play ros2_jazzy_ws/rosbags/street_detections --loop
~~~~

### Step 5: Subscribe to Topics
~~~~bash
# In another terminal - View detections
ros2 topic echo /yolo_detections

# View raw images
ros2 topic echo /camera/rgb/image

# View annotated images (with bounding boxes)
ros2 topic echo /yolo_annotated
~~~~

---

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/rgb/image` | sensor_msgs/Image | Raw RGB camera frames |
| `/yolo_detections` | vision_msgs/Detection2DArray | Object detections (bbox, class, score) |
| `/yolo_annotated` | sensor_msgs/Image | Annotated images with bounding boxes |

---

## Requirements

### For Using Recorded Data (All Teams)
- Ubuntu 24.04 with ROS2 Jazzy
- ~4 GB disk space for rosbag
- No special hardware needed!

### For Live Development (Vision Team)
- Docker Desktop (Windows/Mac/Linux)
- ZED2 or ZED2i camera (for live capture)
- SVO files (for recorded playback)

---

##  Vision Team Development Guide

### A. Practice at Home (Recommended Starting Point)

Develop and test the pipeline using SVO recordings—no physical hardware required.

#### Windows Setup

**Prerequisites:**
- Docker Desktop with WSL 2
- Git for Windows
- PowerShell

**Step 1: Clone Repository**
~~~~bash
git clone https://github.com/amirgshabo/watcher_object_detection.git
cd watcher_object_detection
git config core.autocrlf false
git rm --cached -r .
git reset --hard
~~~~

**Step 2: Build Docker Image** (One-time, ~10 minutes)
~~~~bash
cd docker
docker build -f jazzy-dev.Dockerfile -t arcs_jazzy_ready .
cd ..\ros2_jazzy_ws
~~~~

**Step 3: Start Container**
~~~~bash
docker run -it --rm -v "${PWD}:/ros2_ws" --name arcs_dev arcs_jazzy_ready bash
~~~~

**Step 4: Setup Environment** (Inside Container)
~~~~bash
sed -i 's/\r$//' /ros2_ws/start.sh
chmod +x /ros2_ws/start.sh
/ros2_ws/start.sh

# Install YOLO
pip3 install ultralytics --break-system-packages
pip3 install "numpy<2" --break-system-packages

# Build packages
colcon build --packages-select arcs_yolo_test arcs_yolo_detector
source install/setup.bash
~~~~

**Step 5: Run the Pipeline**

*Terminal 1 - SVO Publisher:*
~~~~bash
ros2 run arcs_yolo_test svo_publisher_opencv --ros-args -p svo_file:=/ros2_ws/test_data/your_file.svo
~~~~

*Terminal 2 - YOLO Detector:*
~~~~bash
docker exec -it arcs_dev bash
source /ros2_ws/install/setup.bash
ros2 run arcs_yolo_detector yolo_detector --ros-args -p input_topic:=/camera/rgb/image
~~~~

*Terminal 3 - Verify Output:*
~~~~bash
docker exec -it arcs_dev bash
source /ros2_ws/install/setup.bash
ros2 topic echo /yolo_detections
~~~~

**Step 6: Record Rosbag for Team**
~~~~bash
ros2 bag record \
  /camera/rgb/image \
  /yolo_detections \
  /yolo_annotated \
  -o /ros2_ws/rosbags/my_recording
~~~~

Let it record for 30-60 seconds, then press `Ctrl+C`.

---

#### Linux / Mac Setup

Same steps as Windows, but use these commands:

~~~~bash
# Step 3: Start Container
docker run -it --rm -v $(pwd):/ros2_ws --name arcs_dev arcs_jazzy_ready bash

# Step 4: No need for sed command, just:
/ros2_ws/start.sh
pip3 install ultralytics --break-system-packages
pip3 install "numpy<2" --break-system-packages
colcon build --packages-select arcs_yolo_test arcs_yolo_detector
source install/setup.bash
~~~~

---

### B. Full System (Jetson + Live ZED Camera)

**Hardware Setup:**
- NVIDIA Jetson with ZED SDK installed
- ZED2 or ZED2i camera connected via USB 3.0

**Terminal 1 - ZED Camera:**
~~~~bash
ros2 launch zed_wrapper zed2.launch.py
~~~~

**Terminal 2 - YOLO Detector:**
~~~~bash
ros2 run arcs_yolo_detector yolo_detector --ros-args -p input_topic:=/zed/zed_node/rgb/image_rect_color
~~~~

**Terminal 3 - Monitor:**
~~~~bash
ros2 topic echo /yolo_detections
~~~~

---

## Troubleshooting

### Common Issues

**"numpy version conflict" or segmentation fault:**
~~~~bash
pip3 uninstall numpy -y --break-system-packages
pip3 install "numpy<2" --break-system-packages
~~~~

**"bash: /ros2_ws/start.sh: bad interpreter"**
~~~~bash
sed -i 's/\r$//' /ros2_ws/start.sh
chmod +x /ros2_ws/start.sh
~~~~

**"docker: command not found"**
- Ensure Docker Desktop is running

**YOLO detector crashes after ~270 frames:**
- This is a known issue with numpy compatibility
- Record rosbags quickly (15-30 seconds) before crash
- Or restart the detector and continue

**Empty rosbag (0 messages):**
- Ensure both SVO publisher and YOLO detector are running before recording
- Check topics exist: `ros2 topic list`

---

## Repository Structure

~~~~
watcher_object_detection/
├── docker/
│   └── jazzy-dev.Dockerfile
├── ros2_jazzy_ws/
│   ├── src/
│   │   ├── arcs_yolo_detector/      # YOLO detection node
│   │   └── arcs_yolo_test/          # SVO publisher & test nodes
│   ├── rosbags/                     # Recorded datasets
│   │   └── README.md                # Download instructions
│   ├── test_data/                   # SVO files for testing
│   └── start.sh                     # Environment setup script
├── .gitignore
└── README.md
~~~~

---

## Contributing

**Vision Team Workflow:**
1. Create feature branch: `git checkout -b feature/your-feature`
2. Make changes and test locally
3. Record demo rosbag if applicable
4. Commit: `git commit -m "Description"`
5. Push: `git push origin feature/your-feature`
6. Create Pull Request on GitHub

---

## Project Information

**Team:** ARCS WATCHER  
**Project:** Wheelchair Assist Technology and Co-bot Helper Robot  
**Organization:** CSUN ARCS  
**Vision Team Lead:** Amir Shabo  

**Repository:** https://github.com/amirgshabo/watcher_object_detection  
**Dataset:** [Download from Google Drive](https://drive.google.com/file/d/19ERBubKotWDy918HaFgf6tWvkFrFF3Xq/view?usp=sharing)

---

## License

This project is part of the ARCS WATCHER senior design project.