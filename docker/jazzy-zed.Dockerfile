# Start from ROS2 Jazzy base image
FROM ros:jazzy

# Avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install basic dependencies
RUN apt-get update && apt-get install -y \
    wget \
    git \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python ZED SDK and dependencies (ignore conflicts with system packages)
RUN pip3 install --no-cache-dir \
    pyzed \
    ultralytics \
    opencv-python-headless \
    --break-system-packages \
    --ignore-installed

# Install ROS2 packages we need
RUN apt-get update && apt-get install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep if not already done
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
    rosdep init; \
    fi && \
    rosdep update

# Set working directory
WORKDIR /ros2_ws

# Source ROS2 automatically when container starts
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]