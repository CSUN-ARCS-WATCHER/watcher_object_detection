# arcs_positional_tracking

Positional tracking subsystem for the ARCS WATCHER project.

This package subscribes to the ZED camera pose and provides:

- A global path of the camera (`/watcher/pose_path`)
- Heading (yaw) and speed estimates
- CSV logging of pose, yaw, and speed for offline analysis
- An RViz config to visualize pose + path

## Topics

### Subscribed

- `/zed/zed_node/pose` (`geometry_msgs/PoseStamped`)
- `/zed/zed_node/odom` (`nav_msgs/Odometry`) [optional]

### Published

- `/watcher/pose_path` (`nav_msgs/Path`)  
- `/watcher/yaw` (`std_msgs/Float32`)  
- `/watcher/speed` (`std_msgs/Float32`)  

## CSV Logging

Logs are written to:

```text
/ros2_ws/pose_logs/pose_log.csv
