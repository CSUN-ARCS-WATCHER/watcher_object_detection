# ARCS WATCHER - Rosbag Datasets

## Download Rosbag Data

The rosbag files are too large for Git. Download them here:

### Street Detections Dataset
- **Google Drive Link:** [https://drive.google.com/file/d/19ERBubKotWDy918HaFgf6tWvkFrFF3Xq/view?usp=sharing]
- **File:** `street_detections.zip` (3.9 GB)
- **Duration:** 26 seconds
- **Scene:** HD city street with cars, pedestrians, buildings

### What's Inside:
- `/camera/rgb/image` - 221 raw HD images (1920x1080)
- `/yolo_detections` - 102 object detections (bounding boxes, classes, scores)
- `/yolo_annotated` - 116 annotated images with detection boxes

### How to Use:
```bash
# 1. Download and extract
unzip street_detections.zip

# 2. Place in rosbags folder
mv street_detections ros2_jazzy_ws/rosbags/

# 3. Play it back
ros2 bag play rosbags/street_detections

# 4. Subscribe to topics (in another terminal)
ros2 topic echo /yolo_detections
```

## Available Topics:
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/rgb/image` | sensor_msgs/Image | Raw camera frames |
| `/yolo_detections` | vision_msgs/Detection2DArray | Object detections |
| `/yolo_annotated` | sensor_msgs/Image | Visualizations |

## Recording Your Own:
See main [README.md](../README.md) for instructions on recording additional rosbags from SVO files.