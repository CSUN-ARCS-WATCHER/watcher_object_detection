#!/bin/bash
# Source ROS2 Jazzy environment
source /opt/ros/jazzy/setup.sh

# Source workspace build environment
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
